/***************************************************************************
* Implementation of the CTEK charging algorithm
* with modifications and tweakables
* 
* - Victor Liu 2023
* based on INA226_WE example sketch
***************************************************************************/

/**********************************************
 * Configuration settings
 *********************************************/
// Voltage stabilization settings
#define NOLOAD_HYST 0.05 // V, on each side
#define NOLOAD_UPD_INTERVAL 50 // ms
#define NOLOAD_DISCH_DELAY 1000

// Battery charge stage voltages
#define SOFT_V 12.6
#define BULK_V 15.0
#define FLOAT_V 13.6 

// Charge stage end thresholds
#define BATDET_VTHR 100 // battery detect: analogRead unitss
#define NEXTSTAGE_ITHR 100 // not used
#define BULKEND_ITHR 500 // end bulk
#define ABSPEND_ITHR 300 // above the trickle current for a dead-ish battery
#define SOFTEND_VTHR 12.50

// Analyze stage parameters
#define ANALYZE_DELAY 180 // seconds
#define ANALYZE_VTHR 12.00 // volts

// Count of threshold successes to go to next stage
#define NEXTSTAGE_COUNTTHR 20

#define STATUS_PERIOD 3000 // Period with which to show the charger status

/**********************************************
 * End of configuration settings
 *********************************************/

#include <Wire.h>
#include <SPI.h>
#include <INA226_WE.h>
#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN 8
Adafruit_NeoPixel pixels(1, PIXEL_PIN, NEO_GRB+NEO_KHZ800);

#define RELAY 9
#define DIGI_CS 4
#define ENABLE 10
#define BAT_DET A2
#define UPDATE_PERIOD 1000
#define N_SAMPLES 10

#define I2C_ADDRESS 0x40

/* There are several ways to create your INA226 object:
 * INA226_WE ina226 = INA226_WE(); -> uses I2C Address = 0x40 / Wire
 * INA226_WE ina226 = INA226_WE(I2C_ADDRESS);   
 * INA226_WE ina226 = INA226_WE(&Wire); -> uses I2C_ADDRESS = 0x40, pass any Wire Object
 * INA226_WE ina226 = INA226_WE(&Wire, I2C_ADDRESS); 
 */
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);

void waitForINAClear() {
  for (int i = 0; i < 5; i++) {
    ina226.readAndClearFlags();
  }
}

byte setDigipot(byte pos)
{
    digitalWrite(DIGI_CS, LOW );
    SPI.transfer(0x00);          // The command for wiper set position
    SPI.transfer(pos);       // The write command expects a second byte which is the value for the wiper
    digitalWrite(DIGI_CS, HIGH);

    return getDigipot();
}

byte incDigipot()
{
    digitalWrite(DIGI_CS, LOW );
    SPI.transfer(0x04);          // The command for wiper up (00000100 = 0x04)
    digitalWrite(DIGI_CS, HIGH);

    return getDigipot();
}

byte decDigipot()
{
    digitalWrite(DIGI_CS, LOW );
    SPI.transfer(0x08);          // The command for wiper up (00001000 = 0x08)
    digitalWrite(DIGI_CS, HIGH);

    return getDigipot();
}

byte getDigipot()
{
    int res = 0;
    digitalWrite(DIGI_CS, LOW );
    res = SPI.transfer16(0x0C00);   // The command for requesting the wiper position. The second byte sent has no useful info but it request 2 bytes sent and responds with 2 bytes  
    digitalWrite(DIGI_CS, HIGH); 

    return res & 0xff;       // The wiper position is in the low order byte so & with 0xff to remove the extra info.
}
long long lastUpd;
long long lastMillis;
long long timeElapsed;
enum Stage {
  NoBatt,
  SoftStart,
  Bulk,
  Absorption,
  Analyze,
  Float,
  Error,
  Done
};
Stage currentState;

enum ErrorType {
  UnknownErr,
  TimeoutErr,
  BatteryErr
};

void statusColor(int r, int g, int b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

#define showStatus statusColor

void setup() {
  Serial1.begin(57600);
  Serial.begin(9600);
  while(!Serial && !(analogRead(BAT_DET) > 100)); // wait until serial comes up on Arduino Leonardo or MKR WiFi 1010
  pixels.begin();
  pixels.clear();

  SPI.begin();
  pinMode(DIGI_CS, OUTPUT);
  pinMode(13, OUTPUT);
  Wire.begin();
  ina226.init();

  /* Set Number of measurements for shunt and bus voltage which shall be averaged
  * Mode *     * Number of samples *
  AVERAGE_1            1 (default)
  AVERAGE_4            4
  AVERAGE_16          16
  AVERAGE_64          64
  AVERAGE_128        128
  AVERAGE_256        256
  AVERAGE_512        512
  AVERAGE_1024      1024
  */
  //ina226.setAverage(AVERAGE_4); // choose mode and uncomment for change of default

  /* Set conversion time in microseconds
     One set of shunt and bus voltage conversion will take: 
     number of samples to be averaged x conversion time x 2
     
     * Mode *         * conversion time *r
     CONV_TIME_140          140 µs
     CONV_TIME_204          204 µs
     CONV_TIME_332          332 µs
     CONV_TIME_588          588 µs
     CONV_TIME_1100         1.1 ms (default)
     CONV_TIME_2116       2.116 ms
     CONV_TIME_4156       4.156 ms
     CONV_TIME_8244       8.244 ms  
  */
  //ina226.setConversionTime(CONV_TIME_1100); //choose conversion time and uncomment for change of default
  
  /* Set measure mode
  POWER_DOWN - INA226 switched off
  TRIGGERED  - measurement on demand
  CONTINUOUS  - continuous measurements (default)
  */
  //ina226.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default
  
  /* Set Resistor and Current Range
     resistor is 5.0 mOhm
     current range is up to 10.0 A
     default was 100 mOhm and about 1.3 A
  */
  ina226.setResistorRange(0.01,10); // choose resistor 5 mOhm and gain range up to 10 A
  
  /* If the current values delivered by the INA226 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA226
  */
  // correct the error
  //ina226.setCorrectionFactor(0.25);
  
  Serial.println("Test Battery Charger implementing CTEK algorithm");
  pinMode(RELAY, OUTPUT); // output relay control
  digitalWrite(RELAY, LOW); // open relay
  pinMode(ENABLE, OUTPUT); // regulator enable control
  digitalWrite(ENABLE, LOW); // enable regulator
  
  setDigipot(0);
  currentState = NoBatt;
  
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero
  lastMillis = millis();
}

void stabilizeNoLoadV(float targetV) {
  showStatus(255, 127, 0);
  Serial.print("Stabilizing regulator output at "); Serial.print(targetV); Serial.println("V");
  digitalWrite(ENABLE, LOW);
  delay(NOLOAD_DISCH_DELAY);
  digitalWrite(ENABLE, HIGH);
  float currentV = 0;
  while (currentV-NOLOAD_HYST >= targetV || currentV+NOLOAD_HYST <= targetV) {
    digitalWrite(13, !digitalRead(13));
    if (targetV > currentV) {
      incDigipot();
    } else {
      decDigipot();
    }
    currentV = ina226.getBusVoltage_V();
    delay(NOLOAD_UPD_INTERVAL);
  }
  Serial.print("Digipot setpoint:"); Serial.println(getDigipot());
  digitalWrite(13, LOW);
}

void startSoft() {
  statusColor(0, 255, 0);
  Serial.print("Battery detected!");
  timeElapsed = 0;
  currentState = SoftStart;
  delay(2000);
  Serial.println("Starting soft charge...");
  digitalWrite(ENABLE, HIGH);
  stabilizeNoLoadV(SOFT_V);
  Serial.println("Voltage ready - switch on the output...");
  digitalWrite(RELAY, HIGH);
  showStatus(255, 255, 0);
  waitForINAClear();
}

void error(ErrorType type=UnknownErr) {
  statusColor(255, 0, 0);
  Serial.println("Error encountered - shut it down...");
  currentState = Error;
  digitalWrite(RELAY, LOW);
  digitalWrite(ENABLE, LOW);
  setDigipot(0);
}

void startBulk() {
  timeElapsed = 0;
  currentState = Bulk;
  Serial.println("Starting bulk charge...");
  digitalWrite(ENABLE, HIGH);
  digitalWrite(RELAY, LOW);
  stabilizeNoLoadV(BULK_V);
  Serial.println("Voltage ready - switch on the output...");
  digitalWrite(RELAY, HIGH);
  showStatus(127, 255, 0);
  waitForINAClear();
}

void startAbsorption() {
  timeElapsed = 0;
  currentState = Absorption;
  Serial.println("Starting constant voltage charge...");
  digitalWrite(ENABLE, HIGH);
  digitalWrite(RELAY, HIGH);
  waitForINAClear();
}

long long analyzeStart;
void startAnalyze() {
  timeElapsed = 0;
  currentState = Analyze;
  showStatus(255, 255, 0);
  Serial.println("Stopping charge and performing capacity hold test...");
  digitalWrite(RELAY, LOW);
  digitalWrite(ENABLE, LOW);

  analyzeStart = millis();
}

void startFloat() {
  timeElapsed = 0;
  currentState = Float;
  Serial.println("Starting float charge...");
  digitalWrite(RELAY, LOW);
  digitalWrite(ENABLE, HIGH);
  stabilizeNoLoadV(FLOAT_V);
  Serial.println("Voltage ready - switch on the output...");
  digitalWrite(RELAY, HIGH);
  delay(1000);
}

void stopCharging() {
  currentState = Done;
  showStatus(0, 0, 0);
  Serial.println("Charging complete! Disconnecting...");
  digitalWrite(RELAY, LOW);
  digitalWrite(ENABLE, LOW);
  Serial.println("All done!");
}

int lastStatus;
int advanceStageCount;
void loop() {
  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0; 

  ina226.readAndClearFlags();
  shuntVoltage_mV = ina226.getShuntVoltage_mV();
  busVoltage_V = ina226.getBusVoltage_V();
  current_mA = ina226.getCurrent_mA();
  power_mW = ina226.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);

  switch(currentState){
    case NoBatt:
      statusColor(0, 0, 0);
      digitalWrite(RELAY, LOW);
      digitalWrite(ENABLE, LOW);
      if (analogRead(BAT_DET) > BATDET_VTHR) {
        startSoft();
      }
      delay(3000);
      statusColor(0, 255, 0);
      delay(100);
      statusColor(0, 0, 0); 
      break;
    case SoftStart:
      if (busVoltage_V > SOFTEND_VTHR) {
        //Serial.print("Candidate continue...");
        advanceStageCount++;
      }
      if (advanceStageCount > NEXTSTAGE_COUNTTHR) {
        advanceStageCount = 0;
        startBulk();
      }
      if (timeElapsed > 20*60*60*1000) {
        error(TimeoutErr);
      }
      break;
    case Bulk:
      if (current_mA < BULKEND_ITHR) {
        //Serial.print("Candidate continue...");
        advanceStageCount++;
      }
      if (advanceStageCount > NEXTSTAGE_COUNTTHR) {
        advanceStageCount = 0;
        startAbsorption();
      }
      if (timeElapsed > 20*60*60*1000) {
        error(TimeoutErr);
      }   
      break;
    case Absorption:
      if (current_mA < ABSPEND_ITHR) {
        //Serial.print("Candidate continue...");
        advanceStageCount++;
      }
      if (advanceStageCount > NEXTSTAGE_COUNTTHR) {
        advanceStageCount = 0;
        startAnalyze();
      }
      if (timeElapsed > 8*60*60*1000) {
        error(TimeoutErr);
      }
      break;
    case Analyze:
      if ((millis() - analyzeStart) > (ANALYZE_DELAY*1000)) {
        digitalWrite(RELAY, HIGH);
        delay(2000);
        ina226.readAndClearFlags();
        busVoltage_V = ina226.getBusVoltage_V();
        if (busVoltage_V > ANALYZE_VTHR) {
          startFloat();
        } else {
          error(BatteryErr);
        }
      }
    case Float:
      if (timeElapsed > 8*60*60*1000) {
        stopCharging();
      }
      break;
    case Done:
      if (analogRead(BAT_DET < BATDET_VTHR)) {
        currentState = NoBatt;
      }
      delay(1000);
      break;
  }
  timeElapsed += (millis() - lastMillis);
  lastMillis = millis();
  if (millis() >= STATUS_PERIOD+lastStatus) {
    Serial.print(busVoltage_V);
    Serial.print(",");
    Serial.print(current_mA);
    Serial.print(",");
    Serial.println(power_mW);

    Serial1.print(busVoltage_V);
    Serial1.print(",");
    Serial1.print(current_mA);
    Serial1.print(",");
    Serial1.println(power_mW);
    lastStatus = millis();
  }
}

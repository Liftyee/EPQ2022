/***************************************************************************
* Example sketch for the INA226_WE library
*
* This sketch is based on the continuous mode example but uses the function setResistorRange to set a different resistor value. 
*  
* This setup uses a stromsensor6mm board with a 5 mOhm shunt resistor
* More information on this board can be found here: https://github.com/generationmake/stromsensor6mm
*
* More information on the INA226_WE library:
* https://wolles-elektronikkiste.de/en/ina226-current-and-power-sensor (English)
* https://wolles-elektronikkiste.de/ina226 (German)
* 
***************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <INA226_WE.h>

#define RELAY 9
#define DIGI_CS 5
#define ENABLE 10
#define MEAS_SEL 11
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
}
Stage currentState;

void setup() {
  Serial.begin(9600);
  while(!Serial); // wait until serial comes up on Arduino Leonardo or MKR WiFi 1010

  SPI.begin();
  pinMode(DIGI_CS, OUTPUT);
  
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
  ina226.setAverage(AVERAGE_16); // choose mode and uncomment for change of default

  /* Set conversion time in microseconds
     One set of shunt and bus voltage conversion will take: 
     number of samples to be averaged x conversion time x 2
     
     * Mode *         * conversion time *
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
  ina226.setResistorRange(0.01,8.19); // choose resistor 5 mOhm and gain range up to 10 A
  
  /* If the current values delivered by the INA226 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA226
  */
  // ina226.setCorrectionFactor(0.95);
  
  Serial.println("Test Battery Charger implementing CTEK algorithm");
  pinMode(RELAY, OUTPUT); // output relay control
  digitalWrite(RELAY, LOW); // open relay
  pinMode(ENABLE, OUTPUT); // regulator enable control
  digitalWrite(ENABLE, LOW); // enable regulator
  pinMode(MEAS_SEL, OUTPUT); // voltage measurement relay control
  digitalWrite(MEAS_SEL, HIGH); // measure connector voltage
  
  setDigipot(0);
  currentState = NoBatt;
  
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero
  lastMillis = millis();
}

#define NOLOAD_HYST 0.05 // V, on each side
#define NOLOAD_UPD_INTERVAL 50 // ms
void stabilizeNoLoadV(float targetV) {
  Serial.print("Stabilizing regulator output at "); Serial.print(targetV); Serial.print("V");
  digitalWrite(MEAS_SEL, HIGH);
  float currentV = 0;
  while (currentV-NOLOAD_HYST >= targetV || currentV+NOLOAD_HYST <= targetV) {
    if (targetV > currentV) {
      incDigipot();
    } else {
      decDigipot();
    }
    currentV = ina226.getBusVoltage_V();
    delay(NOLOAD_UPD_INTERVAL);
  }
}

void startSoft() {
  currentState = SoftStart;
  Serial.print("Starting charge at 12.6V...");
  digitalWrite(ENABLE, HIGH);
  stabilizeNoLoadV(12.6);
  Serial.print("Voltage ready - switch on the output...")
  digitalWrite(MEAS_SEL, LOW);
  digitalWrite(RELAY, HIGH);
}

void error() {
  Serial.println("Error encountered - shut it down...");
  currentState = Error;
  digitalWrite(RELAY, LOW);
  digitalWrite(MEAS_SEL, LOW);
  digitalWrite(ENABLE, LOW);
  setDigipot(0);
}

void startBulk() {
  currentState = Bulk;
  Serial.print("Starting bulk charge at 14.7V...");
  digitalWrite(ENABLE, HIGH);
  digitalWrite(RELAY, LOW);
  stabilizeNoLoadV(14.7);
  Serial.print("Voltage ready - switch on the output...")
  digitalWrite(MEAS_SEL, LOW);
  digitalWrite(RELAY, HIGH);
}

void startAbsorption() {
  currentState = Absorption;
  Serial.print("Starting constant voltage charge...");
  digitalWrite(ENABLE, HIGH);
  digitalWrite(MEAS_SEL, LOW);
  digitalWrite(RELAY, HIGH);
}

void startAnalyze() {
  currentState = Analyze;
  Serial.print("Stopping charge and performing capacity hold test...");
  digitalWrite(RELAY, LOW);
  digitalWrite(ENABLE, LOW);

  error();
}

#define NEXTSTAGE_ITHR 100
#define BULKEND_ITHR 1000
#define SOFTEND_VTHR 12.50
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
    case Done:
      digitalWrite(RELAY, LOW);
      digitalWrite(ENABLE, LOW);
      digitalWrite(MEAS_SEL, HIGH);
      
      delay(3000);
    case NoBatt:
      digitalWrite(RELAY, LOW);
      digitalWrite(ENABLE, LOW);
      digitalWrite(MEAS_SEL, HIGH);
      if (busVoltage_V > 7.0) {
        startSoft();
      }
      delay(3000); 
      // no break; because we want to go straight to softstart
    case SoftStart:
      if (busVoltage_V > SOFTEND_VTHR) {
        startBulk();
      }
      if (timeElapsed > 20*60*60*1000) {
        error();
      }
      break;
    case Bulk:
      if (current_mA < BULKEND_ITHR) {
        startAbsorption();
      }
      if (timeElapsed > 20*60*60*1000) {
        error();
      }   
      break;
    case Absorption:
      if (current_mA < NEXTSTAGE_ITHR) {
        startAnalyze();
      }
      if (timeElapsed > 8*60*60*1000) {
        error();
      }
      break;
    case Analyze:
    
  }
  timeElapsed += (millis() - lastMillis)
}

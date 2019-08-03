/**************************************************************************/
/*!
  @file takkstrip_test.ino
  @author Tom Badger

  Adapted from the work of Eric Bakan & Yaroslav Tenzer

*/
/**************************************************************************/

// Arduino addressing is using 7bit
// All the addresses were calculated in 8bit
// And were right shifted when actually used

#include <Wire.h>

#define MAX_STRIPS 1
#define MAX_SENSORS 5

#define NUM_SENSORS MAX_STRIPS*MAX_SENSORS // reserve addresses for 8 strips with 6 sensors on each
#define PRECISION 0

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D

#define SENSOR_MIN 10
#define SENSOR_MAX 365

#define DIFF_THRESH 20

#define CALIBRATION_SAMPLES 100

int pwmChannel[5] = {3, 6, 5, 9, 10};

float a0[NUM_SENSORS];
float b1[NUM_SENSORS];
float b2[NUM_SENSORS];
float c12[NUM_SENSORS];

float calibrationPressureOffsets[NUM_SENSORS];
float calibrationTemperatureOffsets[NUM_SENSORS];

byte addressArray[NUM_SENSORS];
byte addressLength;

float pressureHistory[NUM_SENSORS];
boolean flagHistoryExists = false;

boolean flagShowAddress = false;
boolean flagShowPressure = true;
boolean flagShowTemperature = false;

boolean calibrateOn = true;

float oldPressure = 0;

void initialize() {
  // s 0C
  Wire.beginTransmission(SENSOR_ALL_ON >> 1);
  Wire.endTransmission();

  // s C0 12 01
  Wire.beginTransmission(0xC0 >> 1);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();

  // s 0D
  Wire.requestFrom(SENSOR_ALL_ON >> 1, 1);

  delay(5);
}

void readCoeffs(byte addressSensor, byte num) {

  // Select sensor
  Wire.beginTransmission(addressSensor >> 1);
  Wire.endTransmission();

  // Request coefficients
  Wire.beginTransmission(FREESCALE_ADDRESS >> 1);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(FREESCALE_ADDRESS >> 1, 8);
  int16_t a0coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b1coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b2coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t c12coeff = (( (uint16_t) (Wire.read() << 8) | Wire.read())) >> 2;
  // Turn sensor off
  Wire.requestFrom(addressSensor >> 1, 1);

  a0[num] = (float)a0coeff / 8;
  b1[num] = (float)b1coeff / 8192;
  b2[num] = (float)b2coeff / 16384;
  c12[num] = (float)c12coeff;
  c12[num] /= 4194304.0;
}

void setup () {
  Wire.begin();
  Serial.begin(9600);

  checkAddresses(); // check how many sensors are connected

  // for each found sensor, read the coefficients ..
  for (int i = 0; i < addressLength; i++) {
    readCoeffs(addressArray[i], i);
    pinMode(pwmChannel[i], OUTPUT); // set required number of PWM pins to output
  }

  if (calibrateOn) {
    calibrate();
  }

}

void calibrate() {

  memset(calibrationPressureOffsets, 0, sizeof(calibrationPressureOffsets)); // set all the offset values in pressure array to zero
  memset(calibrationTemperatureOffsets, 0, sizeof(calibrationTemperatureOffsets)); // set all the offset values in temperature array to zero

  // Create variables to hold returned temp and pressure data returned from read function
  float calTemp = 0;
  float calPressure = 0;

  // Variables to eventually compute the average temp and pressure before saving
  float runningTempTotal = 0;
  float runningPressureTotal = 0;

  for (int i = 0; i < addressLength; i++)
  {
    for (int j = 0; j < CALIBRATION_SAMPLES; j ++) {
      // Read the data from sensors
      readData(addressArray[i], &calTemp, &calPressure);

      // Update our running totals
      runningTempTotal += calTemp;
      runningPressureTotal += calPressure;
    }

    // Save average of sampled data into offset arrays
    calibrationPressureOffsets[i] = runningPressureTotal / CALIBRATION_SAMPLES;
    calibrationTemperatureOffsets[i] = runningTempTotal / CALIBRATION_SAMPLES;

    runningTempTotal = 0;
    runningPressureTotal = 0;
  }
}

void processSig(float* oPressure) {

  float currentPressure = *oPressure;

  if (abs(currentPressure - oldPressure) < DIFF_THRESH){
//    Serial.print(currentPressure);
//    Serial.print(",");
//    Serial.println(oldPressure);
    *oPressure = 0;  
  }

  if (currentPressure <= SENSOR_MIN || currentPressure >= SENSOR_MAX)
  {
    *oPressure = 0;
  }

  oldPressure = currentPressure;

}


void driveHaptic(int channelNum, float* oPressure) {

  if (*oPressure > 0) {
    float pwmSig = 0.41408450704 * (*oPressure) + 31.8591549296;
    analogWrite(pwmChannel[channelNum], (int) pwmSig);
  }
  else {
    analogWrite(pwmChannel[channelNum], 0);  
  }
}

// driveHaptic(i, &oPressure)


void readData(byte addressSensor, float* oTemp, float* oPressure)
{
  // Select sensor
  Wire.beginTransmission(addressSensor >> 1);
  Wire.endTransmission();

  // Request P/T data
  Wire.beginTransmission(FREESCALE_ADDRESS >> 1);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  Wire.requestFrom(FREESCALE_ADDRESS >> 1, 4);
  uint16_t pressure = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  uint16_t temp = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;

  // Turn sensor off
  Wire.requestFrom(addressSensor >> 1, 1);

  *oPressure = pressure;
  //*oTemp = temp;
}

void checkAddresses()
{
  addressLength = 0;
  int temp_add = 0;
  // check every strip
  for (int strip_n = 0; strip_n < MAX_STRIPS; strip_n++) {
    // check every sensor
    for (int sensor_n = 0; sensor_n < MAX_SENSORS; sensor_n++) {
      temp_add = (strip_n << 4) + sensor_n * 2; // calculate the address

      // check if the Attiny responds with its address
      // this also switches ON the Chip Select line on the desired sensor
      Wire.beginTransmission(temp_add >> 1); // take into account that its 7bit !
      // if response finishes with an ACK then continue
      if (Wire.endTransmission() == 0) {
        // check if there is a sensor on this line
        Wire.beginTransmission(FREESCALE_ADDRESS >> 1);
        // if there is an ACK then there is a sensor
        if (Wire.endTransmission() == 0) {
          addressArray[addressLength] = temp_add;
          addressLength++;
        }
        // Turn off the chip select line
        Wire.requestFrom(temp_add >> 1, 1);
      }
    }

  }
}

void loop() {

  float oTemp = 0;
  float oPressure = 0;
  float p_current = 0;
  float p_history = 0;
  float delta_up = 0;
  float delta_down = 0;

  initialize();

  for (int i = 0; i < addressLength; i++)
  {
    if (i > 0) {
      Serial.print(',');
    }
    readData(addressArray[i], &oTemp, &oPressure);

    if (calibrateOn) {
      oPressure -= calibrationPressureOffsets[i];
      oTemp -= calibrationTemperatureOffsets[i];
    }

    oPressure = abs(oPressure);
    oTemp = abs(oTemp);

    processSig(&oPressure);
    driveHaptic(i, &oPressure);

    // the calculations of the wrapping
    // that are used to calculate the minus signs
    if (flagHistoryExists) {
      p_current = oPressure;
      p_history = pressureHistory[i];
      delta_up = p_current - p_history;
      delta_down = p_history - (p_current - 1024);
      if (delta_up < delta_down) {
        oPressure = p_history + delta_up;
      } else {
        oPressure = p_history - delta_down;
      }
    }
    pressureHistory[i] = oPressure;

    // ------------------------------
    // Start output to the serial port

    // Print out sensor ID value if the flag was set
    if (flagShowAddress) {
      Serial.print(addressArray[i], HEX);
    }

    // Print out Pressure values if the flag was set
    if (flagShowPressure) {
      if (flagShowAddress) {
        Serial.print(',');
      }
      Serial.print(oPressure, PRECISION);
    }

    // Print out Temperature values if the flag was set
    if (flagShowTemperature) {
      if (flagShowPressure) {
        Serial.print(',');
      }
      Serial.print(oTemp, PRECISION);
    }
  }
  Serial.println();

  // End output to the serial port
  // ------------------------------
  flagHistoryExists = true;

  // Listen to the commands from the serial port
  if (Serial.available()) {
    byte inByte = (byte)
                  Serial.read();
    if (inByte == 'n') {
      flagShowAddress = !flagShowAddress;
    }
    if (inByte == 'p') {
      flagShowPressure = !flagShowPressure;
    }
    if (inByte == 't') {
      flagShowTemperature = !flagShowTemperature;
    }

  }
}

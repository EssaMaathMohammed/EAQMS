#include <SensirionI2CSen5x.h>


#define debugSerial SerialUSB
SensirionI2CSen5x sen55;

// Variables for error handling
uint16_t error;
char errorMessage[256];

void setup() {
  // put your setup code here, to run once:

  // Start Sensirion sensor and handle possible errors
  while ((!debugSerial) && (millis() < 10000)){}
  debugSerial.begin(57600);
  debugSerial.println("Start");

  // Start I2C communication
  Wire.begin();
  delay(1000);
  sen55.begin(Wire);
  error = sen55.deviceReset();
    if (error) {
        debugSerial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        debugSerial.println(errorMessage);
    }

   // Start Measurement and handle possible errors
  error = sen55.startMeasurement();
  if (error) {
      debugSerial.print("Error trying to execute startMeasurement(): ");
      errorToString(error, errorMessage, 256);
      debugSerial.println(errorMessage);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
float* sensorValues = getSensorValues();
  float pm1 = sensorValues[0];
  float pm2_5 = sensorValues[1];
  float pm4 = sensorValues[2];
  float pm10 = sensorValues[3];
  float voc = sensorValues[4];
  float temp = sensorValues[5];
  float humidity = sensorValues[6];
  float nox = sensorValues[7];

  // Print the sensor values to the debug serial
  debugSerial.print("MassConcentrationPm1:");
  debugSerial.print(pm1);
  debugSerial.print("\t");
  debugSerial.print("MassConcentrationPm2p5:");
  debugSerial.print(pm2_5);
  debugSerial.print("\t");
  debugSerial.print("MassConcentrationPm4:");
  debugSerial.print(pm4);
  debugSerial.print("\t");
  debugSerial.print("MassConcentrationPm10p0:");
  debugSerial.print(pm10);
  debugSerial.print("\t");
  debugSerial.print("AmbientHumidity:");
  if (isnan(humidity)) {
      debugSerial.print("n/a");
  } else {
      debugSerial.print(humidity);
  }
  debugSerial.print("\t");
  debugSerial.print("AmbientTemperature:");
  if (isnan(temp)) {
      debugSerial.print("n/a");
  } else {
      SerialUSB.print(temp);
  }
  debugSerial.print("\t");
  debugSerial.print("VocIndex:");
  if (isnan(voc)) {
      debugSerial.println("n/a");
  } else {
      debugSerial.println(voc);
  }
  debugSerial.print("\t");
  debugSerial.print("NOIndex:");
  if (isnan(nox)) {
      debugSerial.println("n/a");
  } else {
      debugSerial.println(nox);
  }
  delay(3000);
}


float* getSensorValues() {
  // Array to hold sensor values
  static float sensorValues[8];

  // Variables to hold the measurements
  float massConcentrationPm1p0;
  float massConcentrationPm2p5;
  float massConcentrationPm4p0;
  float massConcentrationPm10p0;
  float ambientHumidity;
  float ambientTemperature;
  float vocIndex;
  float noxIndex;

  // Get the measurements
  for (int i = 0 ; i < 5 ; i++){
    error = sen55.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    delay(2000);
  }

  // Handle errors and set sensor values
  if (error) {
      char errorMessage[256];
      debugSerial.print("Error trying to execute readMeasuredValues(): ");
      errorToString(error, errorMessage, 256);
      debugSerial.println(errorMessage);
      // Set all sensor values to NaN in case of error
      for (int i = 0; i < 5; i++) {
          sensorValues[i] = NAN;
      }
  } else {
      // If no error, set sensor values to the measured values
      sensorValues[0] = massConcentrationPm1p0;
      sensorValues[1] = massConcentrationPm2p5;
      sensorValues[2] = massConcentrationPm4p0;
      sensorValues[3] = massConcentrationPm10p0;
      sensorValues[4] = vocIndex;
      sensorValues[5] = ambientTemperature;
      sensorValues[6] = ambientHumidity;
      sensorValues[7] = noxIndex;
      
  }
  return sensorValues;
}
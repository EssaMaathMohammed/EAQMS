// connection code
#include <Sodaq_RN2483.h>
#include <TheThingsNetwork.h>
#include <CayenneLPP.h>
#include <MQ131.h>
#include <MQ135.h> 
#include <SensirionI2CSen5x.h>
#include <ArduinoLowPower.h>

#define debugSerial SerialUSB
#define loraSerial Serial2
#define freqPlan TTN_FP_EU868

SensirionI2CSen5x sen55;
// Variables for error handling
uint16_t error;
char errorMessage[256];
int led = 13;
int digitalPin = 7;
int FlameVal; // digital readings

// Initialize CayenneLPP library
CayenneLPP lpp(51);
// Initialize MQ135 
MQ135 gasSensor = MQ135(A4);

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

static uint8_t DevEUI[8] { 0x00, 0x04, 0xA3, 0x0B, 0x00, 0xE8, 0xBC, 0xA6 };

const uint8_t AppEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const uint8_t AppKey[16] = { 0xCA, 0xC8, 0x2D, 0xD5, 0xAC, 0xDD, 0xC7, 0xD9, 0x2B, 0xB2, 0xD1, 0x9F, 0x46, 0x8E, 0xC3, 0x24 };

void setup() {

    while ((!debugSerial) && (millis() < 10000)) {}

    debugSerial.begin(57600);
    debugSerial.println("Start");
    loraSerial.begin(LoRaBee.getDefaultBaudRate());

    LoRaBee.setDiag(debugSerial);
    LoRaBee.init(loraSerial, LORA_RESET);

    setupLoRa();

    //sensors
    MQ131.begin(2, A5, LOW_CONCENTRATION, 1000000);  
    SerialUSB.println("Calibration in progress...");
    MQ131.calibrate();
    SerialUSB.println("Calibration done!");

    pinMode(led, OUTPUT);
    pinMode(digitalPin, INPUT);

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

void setupLoRa() {
    setupLoRaOTAA();

    LoRaBee.setFsbChannels(1);
    LoRaBee.setSpreadingFactor(9);
}

void setupLoRaOTAA() {
    if (LoRaBee.initOTA(loraSerial, DevEUI, AppEUI, AppKey, true)) {
        debugSerial.println("Network connection successful.");
    }
    else {
        debugSerial.println("Network connection failed!");
    }
}

void loop() {

  float ozoneval = ozone();
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
  SerialUSB.print("Concentration O3 : ");
  SerialUSB.print(ozoneval);
  SerialUSB.println(" ppb");
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
  delay(1000);

  FlameVal = digitalRead(digitalPin); 
  if(FlameVal == HIGH) // if flame is detected
  {
    digitalWrite(led, HIGH); // turn ON Arduino's LED
  }
  else
  {
    digitalWrite(led, LOW); // turn OFF Arduino's LED
  }

  // Prepare data for sending over LoRa
  //restting the payload
  lpp.reset();
    // Measure CO2 and print the value to the debug serial
  float co2Val = co2();
  debugSerial.println(co2Val);

  debugSerial.println(FlameVal);

  lpp.addTemperature(1, co2Val);
  lpp.addDigitalInput(2, FlameVal);
  lpp.addAnalogInput(3, ozoneval);
  lpp.addTemperature(4, voc);
  lpp.addTemperature(5, temp);
  lpp.addRelativeHumidity(6,humidity);
  lpp.addTemperature(7, pm1);
  lpp.addTemperature(8, pm2_5);
  lpp.addTemperature(9, pm4);
  lpp.addTemperature(10, pm10);

  switch (LoRaBee.send(1, lpp.getBuffer(), lpp.getSize()))
  {
  case NoError:
    debugSerial.println("Successful transmission.");
    break;
  case NoResponse:
    debugSerial.println("There was no response from the device.");
    break;
  case Timeout:
    debugSerial.println("Connection timed-out. Check your serial connection to the device! Sleeping for 20sec.");
    delay(20000);
    break;
  case PayloadSizeError:
    debugSerial.println("The size of the payload is greater than allowed. Transmission failed!");
    break;
  case InternalError:
    debugSerial.println("Oh No! This shouldn't happen. Something is really wrong! The program will reset the RN module.");
    setupLoRa();
    break;
  case Busy:
    debugSerial.println("The device is busy. Sleeping for 10 extra seconds.");
    delay(10000);
    break;
  case NetworkFatalError:
    debugSerial.println("There is a non-recoverable error with the network connection. The program will reset the RN module.");
    setupLoRa();
    break;
  case NotConnected:
    debugSerial.println("The device is not connected to the network. The program will reset the RN module.");
    setupLoRa();
    break;
  case NoAcknowledgment:
    debugSerial.println("There was no acknowledgment sent back!");
    break;
  default:
  break;
  }
  // Sleep for 3 minutes (3 * 60 * 1000 milliseconds)
  LowPower.sleep(3 * 60 * 1000);
}

float co2(){
  float co2_ppm;
  // Take multiple CO2 readings for better accuracy
  for (int i = 0 ; i < 5 ; i++){
    co2_ppm = gasSensor.getPPM(); 
    delay(2000);
  }
  // Log the CO2 concentration
  debugSerial.print("CO2 concentration (ppm): ");
  return co2_ppm;
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

float ozone(){
  SerialUSB.println("Sampling...");
  MQ131.sample();
  float val = MQ131.getO3(PPB);
  SerialUSB.print(val);
  return val;
}
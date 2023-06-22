// connection code
#include <Sodaq_RN2483.h>
#include <TheThingsNetwork.h>
#include <CayenneLPP.h>

#define debugSerial SerialUSB
#define loraSerial Serial2
#define freqPlan TTN_FP_EU868

<<<<<<< HEAD
int led = 13;
int digitalPin = 2;
int digitalVal; // digital readings
=======
>>>>>>> ee9960d9c46011e73da01afdd3a6254ef6a151a7
// Initialize CayenneLPP library
CayenneLPP lpp(51);

TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);

static uint8_t DevEUI[8] { 0x00, 0x04, 0xA3, 0x0B, 0x00, 0xE8, 0xBC, 0xA6 };

const uint8_t AppEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };

const uint8_t AppKey[16] = { 0xC3, 0x73, 0x8B, 0x2C, 0x91, 0xBC, 0xC2, 0xB0, 0x78, 0x0A, 0x7F, 0x9C, 0xF7, 0x11, 0x25, 0xA0 };

void setup() {

    while ((!debugSerial) && (millis() < 10000)) {}

    debugSerial.begin(57600);
    debugSerial.println("Start");
    loraSerial.begin(LoRaBee.getDefaultBaudRate());

    LoRaBee.setDiag(debugSerial);
    LoRaBee.init(loraSerial, LORA_RESET);

    setupLoRa();
    //sensors
    pinMode(led, OUTPUT);
    pinMode(digitalPin, INPUT);
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

<<<<<<< HEAD
  digitalVal = digitalRead(digitalPin); 
  if(digitalVal == HIGH) // if flame is detected
  {
    digitalWrite(led, HIGH); // turn ON Arduino's LED
  }
  else
  {
    digitalWrite(led, LOW); // turn OFF Arduino's LED
  }
=======
    
>>>>>>> ee9960d9c46011e73da01afdd3a6254ef6a151a7
  // Prepare data for sending over LoRa
  //restting the payload
  lpp.reset();
  float reading = 23.5;
<<<<<<< HEAD
  debugSerial.println(digitalVal);

  lpp.addTemperature(1, reading);
  lpp.addDigitalInput(2, digitalVal);
=======

  lpp.addTemperature(1, reading);

>>>>>>> ee9960d9c46011e73da01afdd3a6254ef6a151a7
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
}


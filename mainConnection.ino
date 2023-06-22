// connection code
#include <Sodaq_RN2483.h>
#include <TheThingsNetwork.h>

#define debugSerial SerialUSB
#define loraSerial Serial2
#define freqPlan TTN_FP_EU868

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

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
    String reading = "23.5";
  switch (LoRaBee.send(1,(uint8_t*)reading.c_str(), reading.length()))
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

static void getHWEUI() {
    uint8_t len = LoRaBee.getHWEUI(DevEUI, sizeof(DevEUI));
}

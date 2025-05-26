/*
 * Authors: Kennan Bays
 * Hardware: QRET Comms Module Rev2 (WAGGLE)
 * Env: Arduino 1.8.10, STM32duino 2.7.1
 * Created: ~Jun.16.2024
 * Updated: May.26.2025
 * Purpose: SRAD firmware for communications module.
 */

#include "CANPackets.h"
#include "pinouts.h"
#include "STM32_CAN.h" //https://github.com/pazi88/STM32_CAN
#include <Wire.h>
#include <RadioLib.h>
#include <SoftwareSerial.h>
#include <SerialFlash.h>
#include <SPI.h>
//#include "flashTable.h"


// TODO: Implement proper dual-wave driving of the buzzer
#define BUZZER_PIN BUZZER_A_PIN


//TODO: Replace Defines
#define SERIAL_ENABLE true
#define SERIAL_BAUD 38400 // NOTE: While using software serial, dont push above 38400
#define CANBUS_BAUD 500000 //500kbps

//Buzzer Settings
const uint32_t BEEP_DELAY = 6000;
const uint32_t BEEP_LENGTH = 1000;
const uint32_t BEEP_FREQ = 1000;

const double FREQUENCY = 905.4;
const double BANDWIDTH = 31.25;
const int32_t SPREADING_RATE = 10;
const uint8_t CODING_RATE = 6;

#define NORMAL_TRANSMIT_GAP 100
#define BEACON_TRANSMIT_GAP 100 //15000 //15sec
uint32_t transmitGapTime = NORMAL_TRANSMIT_GAP;


#define POWER_DOWN_DELAY 6000 //600000
bool pendingPowerDown = false;
uint32_t powerDownInitTime = 0; //10min
bool inPowerDown = false;


// Hardware serial object for USB comms
// NOTE: This had to be changed to software serial as the TX/RX pins are flipped on hardware.
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN);

// CANBus objects
STM32_CAN can( CAN1, ALT ); //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_RX_msg;
static CAN_message_t CAN_TX_msg;

// Create FlashTable object
const uint8_t TABLE_NAME = 0;
const uint8_t TABLE_COLS = 3;
const uint32_t TABLE_SIZE = 204800; //4096000
//FlashTable table = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256);


// SX1262 (LoRa1262F30) objects
SPIClass newSPI(RF_MOSI_PIN, RF_MISO_PIN, RF_SCK_PIN);
Module* mod;
SX1262* radio;
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false; // flag to indicate that a packet was sent


void powerDown() {
  transmitGapTime = BEACON_TRANSMIT_GAP;
}//powerDown


void setFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
  digitalWrite(STATUS_LED_PIN, LOW);
}//setFlag()

void radioInit() {
// initialize SX1262 with default settings
  usb.print(F("[SX1262] Initializing ... "));
  radio->reset();
  delay(100);
  int state = radio->begin();
  if (state == RADIOLIB_ERR_NONE) {
    usb.println(F("success!"));
  } else {
    usb.print(F("failed, code "));
    usb.println(state);
    while (true) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(500);
    }
  }


  //radio->forceLDRO(true);

  if (radio->setFrequency(FREQUENCY) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    usb.println(F("Selected frequency is invalid for this module!"));
    while (true);
  }
  if (radio->setBandwidth(BANDWIDTH) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    usb.println(F("Selected bandwidth is invalid for this module!"));
    while (true);
  }

  // set spreading factor to 10
  if (radio->setSpreadingFactor(SPREADING_RATE) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    usb.println(F("Selected spreading factor is invalid for this module!"));
    while (true);
  }

  // set coding rate to 6
  if (radio->setCodingRate(CODING_RATE) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    usb.println(F("Selected coding rate is invalid for this module!"));
    while (true);
  }

  // set output power to 12 dBm (SX1262 max is +22, LoRa1262F30 has amp for up to +30)
  // TODO: Change this to a higher power. Check how the LoRa1262F30 is designed; does it use an external PA? Do we have to enable that?
  if (radio->setOutputPower(12) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    usb.println(F("Selected output power is invalid for this module!"));
    while (true);
  }
  //Enable LoRa CRC
  radio->setCRC(true);

}//radioInit()





// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (usb.available()) {usb.read();}
}//emptySerialBuffer()


void setup() {
  #if defined(SERIAL_ENABLE)
  usb.begin(SERIAL_BAUD);
  #endif
  
  // Set pinmodes
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_A_PIN, OUTPUT);
  pinMode(BUZZER_B_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Set up SPI
//  SPI.setSCLK(FLASH_SCK_PIN);
//  SPI.setMISO(FLASH_MISO_PIN);
//  SPI.setMOSI(FLASH_MOSI_PIN);

  // Init CANBUS
  can.begin(); //automatic retransmission can be done using arg "true"
  can.setBaudRate(CANBUS_BAUD);

  // Initialize Flash Chip
//  while (!SerialFlash.begin(FLASH_CS_PIN)) {
//    usb.println(F("Connecting to SPI Flash chip..."));
//    delay(250);
//    //toggleStatusLED();
//  }//while

  //digitalWrite(STATUS_LED_PIN, LOW);
  //delay(1000);

  // Initialize FlashTable object
  //table.init(&SerialFlash);

  //digitalWrite(STATUS_LED_PIN, HIGH);
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  newSPI.begin();
  mod = new Module(RF_CS_PIN, RF_DIO1_PIN, RF_RESET_PIN, RF_BUSY_PIN, newSPI);
  radio = new SX1262(mod);
  radioInit();


  // set the function that will be called
  // when packet transmission is finished
  //radio->setDio0Action(setFlag, RISING);

  // STARTUP BEEP
  delay(BEEP_DELAY);
  tone(BUZZER_PIN, BEEP_FREQ);
  delay(BEEP_LENGTH);
  tone(BUZZER_PIN, 0);

  // Startup delay - Check to enter debug mode
  uint32_t startTime = millis();
  while (!usb.available() and millis()-startTime < 5000) {}
  
  if (usb.available()) {
    byte d = usb.read();
    emptySerialBuffer();
    if (d == 'D') {
      usb.println(F("Entered Debug Mode"));
      //debugMode();
      while (true) {}
    }//if
  }//if
  usb.println(F("Running Normally"));
  
}//setup()

uint32_t recvGPSLat = 0;
uint32_t recvGPSLon = 0;
uint8_t recvGPSSats = 0;
uint32_t recvAltitude = 0;

bool seenAltimeter = false;
bool seenSensors = false;
bool seenGPS = false;

#define PACKET_SIZE 14
uint8_t packet[PACKET_SIZE] = {};

void loop() {
  // put your main code here, to run repeatedly:

//  if (!inPowerDown && pendingPowerDown && millis()-powerDownInitTime > POWER_DOWN_DELAY) {
//    inPowerDown = true
//    powerDown();
//  }//if

  usb.println("Ping");

  digitalWrite(STATUS_LED_PIN, HIGH);
  uint32_t strtTr = millis();
  //int state = radio->transmit("QRET RF TEST; THIS IS 30 BYTES");
  int state = radio->transmit(packet, PACKET_SIZE);
  uint32_t endTr = millis();
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(transmitGapTime);

  while (can.read(CAN_RX_msg)) {

    usb.print(F("CAN ID = "));
    usb.println(CAN_RX_msg.id, BIN);
    if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_LAT_CANID) {
      //CAN_RX_msg.buf[i]
      packet[0] = CAN_RX_msg.buf[0];
      packet[1] = CAN_RX_msg.buf[1];
      packet[2] = CAN_RX_msg.buf[2];
      packet[3] = CAN_RX_msg.buf[3];
      seenGPS = true;
      usb.println(F("Recv LAT"));
    } else if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_LON_CANID) {
      //CAN_RX_msg.buf[i]
      packet[4] = CAN_RX_msg.buf[0];
      packet[5] = CAN_RX_msg.buf[1];
      packet[6] = CAN_RX_msg.buf[2];
      packet[7] = CAN_RX_msg.buf[3];
      seenGPS = true;
      usb.println(F("Recv LON"));
    } else if (CAN_RX_msg.id == GPS_MOD_CANID+GPS_NUMSAT_CANID) {
      //CAN_RX_msg.buf[i]
      packet[8] = CAN_RX_msg.buf[0];
      usb.println(F("Recv NUM SAT"));
      seenGPS = true;
    } else if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+ALTITUDE_CANID) {
      //CAN_RX_msg.buf[i]
      packet[9] = CAN_RX_msg.buf[0];
      packet[10] = CAN_RX_msg.buf[1];
      packet[11] = CAN_RX_msg.buf[2];
      packet[12] = CAN_RX_msg.buf[3];
      usb.println(F("Recv ALTITUDE"));
    } else if (CAN_RX_msg.id == ALTIMETER_MOD_CANID+FLIGHT_STAGE_CANID) {
      //check if landed
      usb.println(F("Recv FLGHT STAGE"));
      seenAltimeter = true;
//      if (CAN_RX_msg.buf[0] >= 5) {
//        if (!pendingPowerDown) {
//          pendingPowerDown = true;
//          powerDownInitTime = millis();
//        }//if
//      }//if(landed)
    } else if (CAN_RX_msg.id == SENSOR_MOD_CANID+STATUS_CANID) {
      seenSensors = true;
    }

    packet[13] = seenGPS*1 + seenAltimeter*2 + seenSensors*4;

  }//while

  
}//loop()

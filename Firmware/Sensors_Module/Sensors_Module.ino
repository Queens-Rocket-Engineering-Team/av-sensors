/*COTIJA Firmware - AIM Sensors Module
 * Authors: Kennan Bays, Joachim Blohm, Brent Naumann, Ethan Toste
*/

/*
 * COTIJA Firmware - AIM Sensors Module
 * Authors: Kennan Bays, Joachim Blohm, Brent Naumann, Ethan Toste
 * Hardware: QRET Camera Module Rev2 (SENSILLA)
 * Env: Arduino 1.8.10, STM32duino 2.7.1
 * Created: ~Jun.16.2024
 * Updated: Aug.04.2025
 * Purpose: SRAD firmware for sensors module Rev2 (SENSILLA).
*/

#include <Wire.h>
#include <SPI.h>
#include <flashTable.h>
#include <SerialFlash.h>
#include <SoftwareSerial.h>
#include "pinouts.h"
#include <CANPackets.h>
#include "STM32_CAN.h"

//buzzer
const uint32_t BEEP_DELAY = 12000;
const uint32_t BEEP_LENGTH = 1000;
const uint32_t BEEP_FREQ = 1000;
#define BUZZER_PIN BUZZER_A_PIN

//canbus
const uint32_t STATUS_REPORT_DELAY = 1000; //How frequently the status report should be sent
uint32_t lastStatusReport = 0; //When the last status report sent


//--- FLASH SETTINGS
uint16_t flashDelay = 250; // How frequently the debug LED should be toggled
uint32_t lastFlash = 0; // Last millis() the debug LED was toggle at

const uint32_t LOG_INTERVAL = 50;
uint32_t lastLog = 0;

const uint8_t TABLE_NAME = 0;
const uint8_t TABLE_COLS = 5;
const uint32_t TABLE_SIZE = 16646144;
// IMPORTANT!!!; AT LEAST 2 BLOCK OF SPACE MUST BE RESERVED FOR FILE SYSTEM
// 16MiB = 16777216B, 2x 64KiB blocks = 131072B
// 16MiB - 128KiB = 16646144B

const uint32_t CAM_RECORD_DELAY = 360000; //5min after power on
const uint32_t CAM_POWER_TIME = 10800000; //3hrs after record start

//--- DATALOGGING SETTINGS

FlashTable flash = FlashTable(TABLE_COLS, 16384, TABLE_SIZE, TABLE_NAME, 256); 

//GLOBAL VARIABLES
STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_TX_msg ;

void setup() {
  // Configure pinmodes
  pinMode(BUZZER_A_PIN, OUTPUT);
  pinMode(BUZZER_B_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Start USB debugging
  // Configure I2C bus
  Serial.begin(115200);

  // Configure SPI
  SPI.setSCLK(FLASH_SCK_PIN);
  SPI.setMISO(FLASH_MISO_PIN);
  SPI.setMOSI(FLASH_MOSI_PIN);

  //LED FLASHES TO ALLOW FOR SERIAL OPEN
  for (int i=0 ; i <5 ; i++){
    digitalWrite(STATUS_LED_PIN,HIGH);
    delay(250);
    digitalWrite(STATUS_LED_PIN,LOW);
    delay(250);
  }//for

  // STARTUP BEEP
  delay(BEEP_DELAY);
  initBuzzer();
  setBuzzerFreq(BEEP_FREQ);
  startBuzzer();
  delay(BEEP_LENGTH);
  stopBuzzer();
  
  Serial.println("STARTED");

  Serial.println("[MDE] ENTER D FOR DEBUG");
  digitalWrite(STATUS_LED_PIN,HIGH);
  
  // Prompt for entering debug mode
  if (Serial.available()) {
    byte d = Serial.read();
    emptySerialBuffer();
    Serial.println("[MDE] Entered Debug Mode");
    debugMode();
    while (true) {}
  }//if

  digitalWrite(STATUS_LED_PIN,LOW);

  //Canbus Setup
  //Start CANBUS
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(500000); //500kbps
}//setup()


/*---\/---\/---\/---\/---*\
      |  DEBUG MENU |    
\*---/\---/\---/\---/\---*/

// Empties all bytes from incoming serial buffer.
// Used by Debug mode
void emptySerialBuffer() {
  while (Serial.available()) {Serial.read();}
}//emptySerialBuffer()

// Called when Debug mode is activated;
// all normal board functions will cease.
// Only purpose is to respond to serial
// commands.
void debugMode() {

  // Status LED static "ON"
  digitalWrite(STATUS_LED_PIN, HIGH);

  while (true) {

    // Empty buffer
    emptySerialBuffer();

    // Wait for command
    while (!Serial.available()) {}
    uint8_t cmd = Serial.read();

    if (cmd == 'I') {
      // "Identify" command; return board name
      Serial.println(F("[MDE] SENSORS"));
    }//if
  }//while
}//debugMode()

void loop() {  

  //Checking if it is time to report status to CANBUS
  if(millis() - lastStatusReport > STATUS_REPORT_DELAY){
    lastStatusReport = millis();
    sendCANStatus(); //Reporting status
  }//if


}//loop()


// ---CANBUS 
void sendCANStatus(){

    //Build the CANBUS message
    CAN_TX_msg.id = (SENSOR_MOD_CANID+STATUS_CANID);
    CAN_TX_msg.len = 1;

    CAN_TX_msg.buf[0] = 1;

    canb.write(CAN_TX_msg); //send
} //sendCANStatus()

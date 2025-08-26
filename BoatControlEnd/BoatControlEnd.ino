/* BoatControlEnd.ino 
 * Joystick Nano Tx / Rx from Handheld to Tiller
 * 
 * Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)
 * Copyright (c) 2016, olikraus@gmail.com
 * All rights reserved.

 * Original version by Bo
 * Servo tiller position indicator version by Mark (MC)
 * 
 * Last modified 03/06/2023 (MC)
 * Tuned potentiometer positioning to centre swing for left & right handed operation
 * Left hand range very poor
 * Modified DEBUG PRINT statements & serial so that serial is fully disabled if DEBUG=0
 * 
 * 22/07/2023 (MC)
 * Added separate constant control minimum detectable Pot position change, setting it to 2 (was 5)
 * Reduced MinTillerChange to 4
*/

// Who built the firmware last?
#define CompiledBy      "Mark"
//#define CompiledBy      "Bo"

/// Connect the nRF24L01 to most Arduino's like this (Caution, Arduino Mega has different pins for SPI, 
/// see below). Use these same connections for Teensy 3.1 (use 3.3V not 5V Vcc).
///
///                 Arduino      Sparkfun WRL-00691 or nRF24L01 / nRF24L01+
///                 5V-----------VCC   (3.3V to 7V in)
///             pin D8-----------CE    (chip enable in)
///          SS pin D10----------CSN   (chip select in)
///         SCK pin D13----------SCK   (SPI clock in)
///        MOSI pin D11----------SDI   (SPI Data in)
///        MISO pin D12----------SDO   (SPI data out)
///                              IRQ   (Interrupt output, not connected)
///                 GND----------GND   (ground in)

// *****************************************
// *****            I/O Pins           *****

// Which pin on the Arduino is connected to the NeoPixels?
#define ARC_LED_PIN     4   // was 6
#define ARC_POWER_PIN   A0  // Power for LED Arc

// JoyButton Remote Control of Hydaulic Ram
#define Button_Pin0     0   // Centre (Midship)
#define Button_Pin1     1   // GotoPot (Pot controlled tiller)
#define Button_Pin2     2   // Out / PCINT3
#define Button_Pin3     3   // In  / PCINT4
//#define Button_Pin4   4   // Clutch  ** for later

// DIP Switch
#define Switch_Pin0     5   // Dil Switch 0 on Dig IP 5 Channel Low  byte
#define Switch_Pin1     6   // Dil Switch 1 on Dig IP 6 Channel High byte
#define Switch_Pin2     7   // Dil Switch 2 on Dig IP 7 Supervisor Mode
#define Force_Channel   2   // Set to 0-3 to force radio channel, set to > 7 to work from Switch 1, 2 & 3
#define DBGLED_PIN      A3  // Debug Led for checking sleep function, battery voltage & radio status (Was 9)

// Tiller Control Pot
#define POT_PIN         A1  // 10k linear rotary pot

// Tiller Position Indicator Servo
#define SERVO_PIN       9   // Any low-power small R/C servo, eg GS-9018 micro servo

// Battery Voltage sense
#define BatV_PIN        A5  // Senses 5V voltage through 10k/10k divider (5V => 2.5V norm)

// *****************************************
// *****         Configuration         *****

// Define addresses for radio channels
#define CLIENT_ADDRESS  1       // Tiller is Client
#define SERVER_ADDRESS  2       // we are Server
#define SUPER_ADDRESS   3       // we are Superviser

#define Ref_5V          5.0     // ADC reference voltage
#define DRIVESERVO      1       // Enable servo driver
#define Bat_UV          5.9     // Battery under-voltage threshold (Warning) Assumes 4 x 1.5V cells
#define Bat_Critical    5.5     // Battery under-voltage threshold (Sleep)
#define LED_flash_time  5000    // LED flash cycle time (5s)
#define SERVO_LLIM      30      // Left-turn limit of servo
#define SERVO_RLIM      130     // Left-turn limit of servo
#define SERVO_CTR       80      // Centre position of servo

#define TBounce         50      // 50 mSec switch de-bounce time
#define TSleepNear      30000   // 30 seconds inactivity before "Nearly time to go to sleep!" warning is given
#define TSleep          40000   // 40 seconds inactivity before going to sleep
#define TTillerTimeout  15000   // 15 seconds after Tiller changes position, the insructions to move the tiller will stop

#define TillerPotMode   3       // Pot mode (0 = auto L/R, 1 = left-handed, 2 = right-handed, 3 = basic)
#define TillerPot_CCW   5       // Pot output when max CCW
#define TillerPot_CW    1020    // Pot output when max CW
#define TillerPot_RCCW  2       // Pot output when RH max CCW
#define TillerPot_RCW   390     // Pot output when RH max CW
#define TillerPot_LCCW  728     // Pot output when LH max CCW
#define TillerPot_LCW   1009    // Pot output when LH max CW
#define TillerPos_LLIM  0       // Left-end (CCW) Tiller position limit (not currently used)
#define TillerPos_RLIM  127     // Right-end (CW) Tiller position limit (not currently used)
#define PotGain         1.0     // TillerPot Gain (Default = 1.0)
#define PotOffset       -128    // TillerPot Offset (Default = 0) (16 for basic pot mode)
#define TillerPotMin    0       // ModifiedPotValue output when midway between left-handed range and right-handed range (pointing out of the end of the box)
#define TillerPotCentre 520     // ModifiedPotValue output when midway between left-handed range and right-handed range (pointing out of the end of the box)
#define TillerPotMax    1023    // ModifiedPotValue output when midway between left-handed range and right-handed range (pointing out of the end of the box)
#define TillerPosCentre 70      // Output of tiller position from tiller-end when tiller is mid-ship.
#define TillerPosMin    7       // Precautionary limit on Tiller position to prevent Ram being driven to limit
#define TillerPosMax    120     // Precautionary limit on Tiller position to prevent Ram being driven to limit
#define MinTillerChange 4       // Controls how sensitive the Tiller position sensor is - 5 is good, < 5 is too small (Tiller can oscillates), > 7 is too big (larger positional error)
#define MinPotChange    2       // Controls how sensitive the Pot is to change

// *****************************************
// ***** Debugging switches and macros *****

#define OLED 0                  // turn the OLED display on or off - also runs faster with no display page refresh
#define DEBUG 1                 // Switch debug output on and off by 1 or 0

#if DEBUG
#define PRINTS(s)       { Serial.print(F(s)); }
#define PRINTSLN(s)     { Serial.print(F(s)); Serial.print("\n");}
#define PRINTSS(s1,s2)  { Serial.print(F(s1)); Serial.print(F(s2));}
#define PRINTI(s,v)     { Serial.print(F(s)); Serial.print(v); }
#define PRINT(s,v,dec)  { Serial.print(F(s)); Serial.print(v,dec); }
#define PRINTX(s,v)     { Serial.print(F(s)); Serial.print(F("0xSerial.print(v, HEX); }
#else
#define PRINTS(s)
#define PRINTSLN(s)
#define PRINTSS(s1,s2)
#define PRINTI(s,v)
#define PRINT(s,v,dec)
#define PRINTX(s,v)
#endif

// *****************************************
// *****          Libraries...         *****

#include <Arduino.h>
#include <avr/sleep.h>
#include <PWMServo.h>               // Servo driver library (Note - this library works better than servo.h in this design)
#include <Adafruit_NeoPixel.h>

// Include RadioHead ReliableDatagram & NRF24 Libraries
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>

#if OLED  // Optional OLED library
#include <U8g2lib.h>
#endif

// #ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
// #endif

// #ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
// #endif


// *****************************************
// *****        Initialisation         *****

// Initialise Tiller Simulation Servo
PWMServo myservo;

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
  This is a page buffer  
*/

#if OLED //******************************************* OLED
// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2,/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
#endif //******************************************* OLED

// End of constructor list


uint8_t RadioChanNo = 0;
uint8_t SupervisorMode = 0;
uint8_t SuperAck = 0;

// Create an instance of the HW radio driver
RH_NRF24 RadioDriver;

// Sets the radio driver to NRF24 and the server address to 2
RHReliableDatagram RadioManager(RadioDriver, SERVER_ADDRESS);

// Define a message to return if values received
uint8_t ReturnMessage[] = "JoyStick Data Received"; 

// Define the Message Buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
// Declare unsigned 8-bit motorcontrol array
// 2 Bytes for motor Direction, plus 1 byte for Clutch control

uint8_t motorcontrol[4];            // in, out, clutch, linPot,...
uint8_t RX_state = 0;
uint8_t TillerPos;
uint8_t ServoPos = SERVO_CTR;       // Start the servo in the middle position
uint8_t MaxDisplayWidth = 128;
uint8_t MaxDisplayHeight = 96;
uint8_t wait_ok = 0;
uint8_t wait_last = 0;

uint8_t numPasses = 0;

int PotValue = 0;
int ModifiedPotValue = 0;
int PotOffsetL = 64-92;             // TillerPot Offset for Left-handed use (Default = 64-92 = -28)
int PotOffsetR = 64-36;             // TillerPot Offset for Right-handed use (Default = 64-36 = +28)
uint8_t PotTillerPos;
uint8_t OldPotTillerPos;

unsigned long TimeNow;
unsigned long TimeLast;
unsigned long TimeTillerInactive;
unsigned long Count;
unsigned long t_NextFlash = 0;      // LED flash timer
unsigned long t_LED_On = 50;        // LED flash timer
unsigned long t_LED_Off = 50;       // LED flash timer

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 12
uint8_t LedPos;
uint8_t LedPosMidShip;


// Declare our NeoPixel strip object:  - for Tiller Position feedback
Adafruit_NeoPixel strip(LED_COUNT, ARC_LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

uint8_t sim_Tiller = 0;
uint8_t abortClutch = 0;
uint8_t draw_dirn = 0;

float BatteryVoltage = 0.0;       // Battery Voltage
boolean BatteryCritical = false;
boolean BatteryWarning = false;
boolean NearToSleep = false;
boolean GoingToSleep = false;
uint8_t pulseLED = 0;             // Number of times to pulse the debug LED every 5s

// *****************************************
// *****          Functions            *****

#if OLED
void u8g2_prepare(void) {          //Prepare OLED Display
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_line(uint8_t TilPos) {
   u8g2.drawLine(MaxDisplayWidth/2,0,TilPos,MaxDisplayHeight );
   
}
#endif // OLED


// pin change interrupt service routine for D0 to D7
ISR (PCINT2_vect)
 {
   // handle pin change interrupt for D0 to D7 here
 }  // end of PCINT2_vect


// Update status LED
void flashLED() {
  // 5-second cycle
  // 1 pulse - all ok
  // 2 pulses - low battery warning
  // 5 pulses - Timing-out...

  // Read battery voltage and check it's not unver-voltage
  // Battery sensor is a 10k/10k potential divider so factor of 2 included below...
  BatteryVoltage = 2 * Ref_5V * (analogRead(BatV_PIN) / 1023.0);
  if (BatteryVoltage <= Bat_Critical) {
    BatteryCritical = true;
    BatteryWarning = true;
  }
  else if (BatteryVoltage <= Bat_UV) {
    BatteryCritical = false;
    BatteryWarning = true;
  }
  else {
    BatteryCritical = false;
    BatteryWarning = false;
  }
  
  PRINT("\n Battery Voltage:     ", BatteryVoltage, 3); 
  PRINTI("\n PotValue:            ", PotValue); 
  PRINTI("\n ModifiedPotValue:    ", ModifiedPotValue);
  PRINTI("\n PotTillerPos:        ", PotTillerPos);
  PRINTI("\n OldPotTillerPos:     ", OldPotTillerPos);
  PRINTI("\n TillerPos:           ", TillerPos); 
  PRINTI("\n buf[3]:              ", buf[3]); 
  PRINTI("\n Radio Status:        ", RX_state); 
  PRINTI("\n Time Now:            ", TimeNow);
  PRINTI("\n Time Last:           ", TimeLast);
  PRINTI("\n Time TillerInactive: ", TimeTillerInactive);
  if (TillerPotMode == 0) {
    if (ModifiedPotValue >= TillerPotCentre) {  // Handset is probably in left-handed configuration
      PRINTS("\n Left-handed!"); }
    else {                                      // Handset is probably in left-handed configuration
      PRINTS("\n Right-handed!"); }
  }
  else {                                        // Handset is in basic Pot mode - no L/R option
    PRINTS("\n Basic Pot Mode");
  }
  PRINTS("\n ********************************");
  

  
  // pulseLED = 1 flash by default to indicate controller is awake
  // pulseLED is increased by 1 for 'No RX' radio warning
  // pulseLED is increased by 2 for low-battery warning
  // pulseLED is increased by 8 for sleep time-out warning (TBD...)

  if (RX_state != 2 and RX_state != 3)  // Check radio
    pulseLED = pulseLED + 1;            // Add 1 flash if radio is not good
  if (BatteryWarning)                   // Check battery voltage
    pulseLED = pulseLED + 2;            // Add 2 flashes if battery is under-voltage
  if (BatteryCritical)                   // Check battery voltage
    pulseLED = pulseLED + 4;            // Add 4 flashes if battery is critically under-voltage
  if (NearToSleep)                      // Check time to sleep
    pulseLED = pulseLED + 8;            // Add 8 flashes if near to sleep-time
  if (GoingToSleep)                     // Check time to sleep
    pulseLED = pulseLED + 16;           // Add 16 flashes if going to sleep
    
  for (int i=0; i < pulseLED; i++){
    digitalWrite(DBGLED_PIN, HIGH);     // LED On
    delay(t_LED_On);
    digitalWrite(DBGLED_PIN, LOW);      // LED Off
    delay(t_LED_Off);
  }
}

// Process inputs from joystick, buttons or TillerPot into motor controls...
void readJoystick(uint8_t motor[3]){
  PotValue = analogRead(POT_PIN);
  ModifiedPotValue = (PotValue * PotGain) + PotOffset;
  if (ModifiedPotValue > 1023)
    ModifiedPotValue = 1023;
  
  if (ModifiedPotValue < 0)
    ModifiedPotValue = 0;

  // Check for left-handed or right-handed configuration based on positioning of the tiller pot...
  if (TillerPotMode == 0) {
    if (ModifiedPotValue >= TillerPotCentre) { // Handset is probably in left-handed configuration
      // Apply LH Limits
      if (ModifiedPotValue < TillerPot_LCCW)
        ModifiedPotValue = TillerPot_LCCW;
      if (ModifiedPotValue > TillerPot_LCW)
        ModifiedPotValue = TillerPot_LCW;
    
      // Apply LH Mapping
      PotTillerPos = MaxDisplayWidth - (map(ModifiedPotValue, TillerPot_LCCW, TillerPot_LCW, 0, MaxDisplayWidth-1));    // Works but Pot range is very limited and centre isn't centre. 
    }
    else                                     { // Handset is probably in right-handed configuration
      // Apply RH Limits
      if (ModifiedPotValue < TillerPot_RCCW)
        ModifiedPotValue = TillerPot_RCCW;
      if (ModifiedPotValue > TillerPot_RCW)
        ModifiedPotValue = TillerPot_RCW;
    
      // Apply RH Mapping
      PotTillerPos = MaxDisplayWidth - (map(ModifiedPotValue, TillerPot_RCCW, TillerPot_RCW, 0, MaxDisplayWidth-1));
    }
  }

  else { // Assume basic mode (3)
    // Apply basic Mapping
    PotTillerPos = MaxDisplayWidth - (map(ModifiedPotValue, TillerPot_CCW, TillerPot_CW, 0, MaxDisplayWidth-1));    // 
  }
  
  // Check for Tiller Pot change...
  if (abs(PotTillerPos - OldPotTillerPos) > MinPotChange) {
    OldPotTillerPos = PotTillerPos;
    TimeTillerInactive = TimeNow + TTillerTimeout; // Reset Tiller Timeout
  }
      
  if (!digitalRead(Button_Pin1)) { // Highest priority control - Goto Pot position
    //PRINTS("\n Button_Pin1!");
    if (RX_state == 3 and PotTillerPos > TillerPos + MinTillerChange and TimeNow < TimeTillerInactive) // Pot controls the Tiller
      motor[0] = 1;
    if (RX_state == 3 and PotTillerPos < TillerPos - MinTillerChange and TimeNow < TimeTillerInactive) // Pot controls the Tiller
      motor[1] = 1;

    // Reset standby timeout
    TimeLast = millis(); //set Timelast to current millis()
  }
  else if (!digitalRead(Button_Pin0)) { // Goto MidShip position
     //PRINTS("\n Button_Pin0!");
     if (TillerPos < (TillerPosCentre-3)) {
       motor[0] = 1;
       motor[1] = 0;
     }
     else if (TillerPos > (TillerPosCentre+3)) {
       motor[0] = 0;
       motor[1] = 1;
     }    

    // Reset standby timeout
    TimeLast = millis(); //set Timelast to current millis()
  }
  else if      (!digitalRead(Button_Pin2)) { // Button or Joystick controls the Tiller
    //PRINTS("\n Button_Pin2!");
    motor[0] = 1;
    motor[1] = 0;

    // Reset standby timeout
    TimeLast = millis(); //set Timelast to current millis()
  }
  else if (!digitalRead(Button_Pin3)) { // Button or Joystick controls the Tiller
    //PRINTS("\n Button_Pin3!");
    motor[0] = 0;
    motor[1] = 1;

    // Reset standby timeout
    TimeLast = millis(); //set Timelast to current millis()
  }
  
//  if (motor[0] || motor[1])
//  {
//    TimeLast = millis(); //set Timelast to current millis()
//  }

  // Check precautionary limits
  // Override motor controls, disabling motor in each direction if the tiller position sensor is nearing limits
  if (TillerPos <= TillerPosMin) {
    motor[1] = 0;
  }
  if (TillerPos >= TillerPosMax) {
    motor[0] = 0;
  }

  // Unlock the clutch if there is any activity - (locks when going to sleep)
  // (MC) Not sure this doesn anything, I think the clutch control is entirely handled in the Tiller-end controller
  motor[2] =  1; // set Clutch
}

// LED Arc update function
void ledArcWrite(uint32_t color, int posn) {
  uint32_t magenta = strip.Color(128, 0, 128);
  uint8_t col = 0;
  // for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.clear();                         // clear out previous LED posn
    /********** set several Leds on either side of LED
    for(int i=0; i<strip.numPixels(); i++) {  // 0 -> numPixels
      if ((posn < 6) && (i < posn)) 
       {
        col = col + 30;
        strip.setPixelColor(i, 0, col, 0);
       }
       
    if ((posn > 5) && (i > posn)) 
       {
        col = col + 30;
        strip.setPixelColor(i, 0, col, 0);
       }
    }
    ********************/
    if (LedPosMidShip == 1)
     {
      strip.setPixelColor(5, color);
      strip.setPixelColor(6, color);
     }
     else
      strip.setPixelColor(posn, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    //delay(wait);                           //  use main loop delay Pause for a moment
    
 
}// End ledArcWrite

void setup_radio(void){

  if (digitalRead(Switch_Pin2)== 0 and Force_Channel > 7) // supervisor Switch on
  {
    // Sets the radio driver to NRF24 and the server address to 3
    // RHReliableDatagram RadioManager(RadioDriver, SUPER_ADDRESS);   
    // SUPER_ADDRESS
    RadioManager.setThisAddress(SUPER_ADDRESS);
       
    SupervisorMode = 1;
    PRINTSLN("supervisor Mode"); 
  }
  else {
    // Sets the radio driver to NRF24 and the server address to 2
    // RHReliableDatagram RadioManager(RadioDriver, SERVER_ADDRESS); 
    RadioManager.setThisAddress(SERVER_ADDRESS); 
    SupervisorMode = 0;
    PRINTSLN("Normal Mode");
  }

  // Initialize RadioManager with defaults - 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!RadioManager.init()) {
    PRINTSLN("init failed");
    RX_state = 0;
  }
    
  else {
    PRINTSLN("Radio initialized OK");
    //if (SupervisorMode == 1 ) DoRadioTx(); // let Tiller know we are supervisor  ************<<<
    RX_state = 1;
  }

  if (digitalRead(Switch_Pin1)== 0 and Force_Channel > 7) { // Chan Hi Switch on
    RadioChanNo = 2;
  }
  else {
    RadioChanNo = 0;
  }

  if (digitalRead(Switch_Pin0)== 0 and Force_Channel > 7) // Chan Lo Switch on
    RadioChanNo = RadioChanNo + 1;  // results in Chan 0 - 3

  // Option to override Radio Channel using the Force_Channel constant
  if (Force_Channel <= 7)
    RadioChanNo = Force_Channel & 3;
    
  RadioDriver.setChannel(120 + RadioChanNo); // Channel 120 - 123
  PRINTI(" Radio Channel ", RadioChanNo);
  PRINTS("\n");
  //RadioDriver.setChannel(124);

  if (!RadioDriver.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPowerm6dBm)) { //12dBm
    PRINTS("\n setRF failed"); 
    RX_state = 0;
  }
  else {
    PRINTS("\n Radio initialized OK");
    RX_state = 1;
  }

  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  /**********************************  Radio options - eg: set different chan, etc **************
  if (!RadioDriver.setChannel(1))
    PRINTSLN("setChannel failed");
  **************************/

  /**********************************  set different Data Rate **************   
  if (!RadioDriver.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    PRINTSLN("setRF failed");  
  **************************/

  /*********************************
  DataRate { DataRate1Mbps = 0, DataRate2Mbps, DataRate250kbps }
  TransmitPower {
  TransmitPowerm18dBm = 0, TransmitPowerm12dBm, TransmitPowerm6dBm, TransmitPower0dBm,
  RFM73TransmitPowerm10dBm = 0, RFM73TransmitPowerm5dBm, RFM73TransmitPowerm0dBm, RFM73TransmitPower5dBm
  *******************/

  /**************** 
  if  (!RadioDriver.setTxPower(14))
    PRINTS("setRF failed \n");
  else  PRINTS("Radio setRF OK \n");
  *******************/  
}

// Arduino setup function - runs automatically at power-up
void setup(void) {

  // Only setup the serial port if in DEBUG mode...
  if (DEBUG)
    Serial.begin(9600);
    
  // Print the build info...
  PRINTSS("Compiled by: ", CompiledBy);
  PRINTS("\n");
  PRINTSS(__FILE__, "\n");
  PRINTS(__DATE__);
  PRINTS(" ");
  PRINTSS(__TIME__, "\n");
  PRINTS("********************************\n\n");

  // **** Setup I/O *****

  pinMode(ARC_POWER_PIN, OUTPUT);  // A0 Power for LED ARC
  digitalWrite(ARC_POWER_PIN, HIGH); // Turn ON 

  // Steering Control Buttons & Joystick
  pinMode(Button_Pin0,INPUT_PULLUP);//set the button as an input 0 High enable internal pullup 
  pinMode(Button_Pin1,INPUT_PULLUP);//set the button as an input 1 High enable internal pullup 
  pinMode(Button_Pin2,INPUT_PULLUP);//set the button as an input 2 High enable internal pullup 
  pinMode(Button_Pin3,INPUT_PULLUP);//set the button as an input 3 High enable internal pullup

  // DIL Switch 
  pinMode(Switch_Pin0,INPUT_PULLUP);//set the Switch 0 as an input 5 High enable internal pullup 
  pinMode(Switch_Pin1,INPUT_PULLUP);//set the Switch 1 as an input 6 High enable internal pullup 
  pinMode(Switch_Pin2,INPUT_PULLUP);//set the Switch 2 as an input 7 High enable internal pullup 

  // declare the Servo pin as an OUTPUT:
  pinMode(SERVO_PIN, OUTPUT);

  // ************************


  // Attach the servo:
  myservo.attach(SERVO_PIN);


  // NeoPixel Strip...
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(60); // Set BRIGHTNESS to about 1/5 (was=30) (max = 255)

  
  // Pin Change Interrupt (example for D3 / D4)  << ********************
  // want pin D3 / pin 2 | D4 / pin 3, pin D00, pin D01
  if (!DEBUG){
    PCMSK2 |= bit (PCINT16); // Pin D00 (Can't use as an interrupt until we kill the serial...)
    PCMSK2 |= bit (PCINT17); // Pin D01 (Can't use as an interrupt until we kill the serial...)
  }
  PCMSK2 |= bit (PCINT18); // Pin D02
  PCMSK2 |= bit (PCINT19); // Pin D03
  PCIFR   |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR   |= bit (PCIE2);    // enable pin change interrupts 
  
  
  #if OLED //******************************************* OLED **********************************************
    u8g2.begin();
    //u8g2_prepare();
  
    MaxDisplayWidth = u8g2.getDisplayWidth();
    MaxDisplayHeight = u8g2.getDisplayHeight();
  #endif //******************************************* OLED **********************************************


  // PRINTSLN(" BoatContRxNRF24_Sleep_Arc4RdGnMidShipSuperChanNoTX");
  // Set motor commands to off, Clutch off
  motorcontrol[0] = 0; // ram out go Left Tiller Right
  motorcontrol[1] = 0; // ram in  go Right Tiller Left
  motorcontrol[2] = 1; // Clutch set engage initially

  TillerPos = MaxDisplayWidth/2;

  // ******************** Radio Setup *********************
  setup_radio();
  
} // End setup

void DoRadioTx(void) {   // send to Tiller Supervisor Status  -- Not Used Now !!!

  if (RadioManager.sendtoWait(motorcontrol, sizeof(motorcontrol), CLIENT_ADDRESS)) 
                                               // sends 4 buf motorcontrol values to Tiller Nano
  {
    // Now wait for a reply from the Tiller
    uint8_t len = sizeof(buf);    
    uint8_t from;      
    if (RadioManager.recvfromAckTimeout(buf, &len, 100, &from))  // was 500 wait
    {   //  Got reply from Tiller
      PRINTSLN(" Registered as Supervisor SuperAck");
      // registered as supervisor
      SuperAck = 1;
      wait_last = 0; //clear wait_fail
      wait_ok = 0;
    }//Reply received 
    else
    {
     PRINTSLN(" Timeout on Tiller reply ");
     
    }  // No Reply 
   
  }
   else
     {
      PRINTSLN(" No reply, is nrf24_reliable_datagram_server running?");
     
    }  // No Reply 
} //end DoRadioTx

void DoRadioRx(void) {    
 if (RadioManager.available()) // Wait for a message addressed to us from the Tiller
  {
 
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (RadioManager.recvfromAck(buf, &len, &from))  //get message from Tiller - buf

    {

      if (SupervisorMode == 1) SuperAck = 1;
      

      //Serial Print the values of joystick
     /* PRINTX("got request from : 0x", from);
      * PRINTS("\n");
      */
      TillerPos = map(buf[3],0,255,0,MaxDisplayWidth-1); //Get Tiller Pos
      if (buf[2] == 0) { 
        abortClutch = 1;} // abort received from Tiller End
        else abortClutch = 0;
      // Return Motor Control Outputs
      buf[0] = motorcontrol[0];   //ram out go Left Tiller Right
      buf[1] = motorcontrol[1];   //ram in go Right Tiller Left
      buf[2] = motorcontrol[2];   // Clutch
      
      RX_state = 3; //u8g2.drawStr(10,40, "Data RX ");
      wait_ok++;
      wait_last = wait_ok;
     /*  Debug stuff
      PRINTI(" TillerPos = ", buf[1]);
      PRINTS("\n");
      PRINTI(" ramOut = ", motorcontrol[0]);
      PRINTS("\n");
      PRINTI(", RamIn = ", motorcontrol[1]);
      PRINTS("\n");
      PRINTI(", Clutch = ", motorcontrol[2]);
      PRINTS("\n");
      */
    } // RX OK
    else RX_state = 2; //u8g2.drawStr(10,40, "Wait RX ");
    
  

 // Send a reply back to the originator client, check for error
      // if (!RadioManager.sendtoWait(ReturnMessage, sizeof(ReturnMessage), from))  //was ReturnMessage
       if (!RadioManager.sendtoWait(buf, sizeof(buf), from)) 
        { /*PRINTSLN("sendtoWait failed"); */
        RX_state = 4; //u8g2.drawStr(10,40, "No Comms");
        }
  }   //RadioManager available
  
}  // End DoRadioRx


// Is it time to "go to sleep" ?   if no activity for TimeDelay millis
void DoSleep(void) { 

    PRINTSLN("\nGoing to sleep...");
    
    // Disable the ADC (Analog to digital converter, pins A0 [14] to A5 [19])
    static byte prevADCSRA = ADCSRA;
    ADCSRA = 0;

    strip.clear();         // clear out previous ARC LED posn
    strip.show();         //  show new status - cleared strip
    digitalWrite(ARC_POWER_PIN, LOW);  // turn off ARC LED strip
    digitalWrite(DBGLED_PIN, LOW);      // Turn off LED go to sleep
     
    RadioDriver.setModeIdle(); // Set Radio off before sleep

    pinMode(Switch_Pin0,LOW);//set the Switch 0 as an input 5 High enable internal pullup to save power
    pinMode(Switch_Pin1,LOW);//set the Switch 1 as an input 6 High enable internal pullup 
    pinMode(Switch_Pin2,LOW);//set the Switch 2 as an input 7 High enable internal pullup 
      
   

    /* Set the type of sleep mode we want. Can be one of (in order of power saving):

        SLEEP_MODE_IDLE (Timer 0 will wake up every millisecond to keep millis running)
        SLEEP_MODE_ADC
        SLEEP_MODE_PWR_SAVE (TIMER 2 keeps running)
        SLEEP_MODE_EXT_STANDBY
        SLEEP_MODE_STANDBY (Oscillator keeps running, makes for faster wake-up)
        SLEEP_MODE_PWR_DOWN (Deep sleep)
    */
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    

    // Turn of Brown Out Detection (low voltage)
    // Thanks to Nick Gammon for how to do this (temporarily) in software rather than
    // permanently using an avrdude command line.
    //
    // Note: Microchip state: BODS and BODSE only available for picoPower devices ATmega48PA/88PA/168PA/328P
    //
    // BODS must be set to one and BODSE must be set to zero within four clock cycles. This sets
    // the MCU Control Register (MCUCR)
    MCUCR = bit (BODS) | bit (BODSE);

    // The BODS bit is automatically cleared after three clock cycles so we better get on with it
    MCUCR = bit (BODS);

    // Ensure we can wake up again by first disabling interupts (temporarily) so
    // the wakeISR does not run before we are asleep and then prevent interrupts,
    // and then defining the ISR (Interrupt Service Routine) to run when poked awake
    noInterrupts();
    // was *** > attachInterrupt(digitalPinToInterrupt(Button_Pin2), wakeISR, LOW); // wakePin
    PCIFR  |= bit (PCIF2);   // clear any outstanding interrupts
    PCICR  |= bit (PCIE2);   // enable pin change interrupts for D00 to D07

    // Send a message just to show we are about to sleep
    PRINTSLN("Good night!");
    if (DEBUG)
      Serial.flush();

    // Allow interrupts now
    interrupts();

    // And enter sleep mode as set above
    sleep_cpu();

    // --------------------------------------------------------
    // Controller is now asleep until woken up by an interrupt
    // --------------------------------------------------------

    // Wakes up at this point when wakePin is brought LOW - interrupt routine is run first
    PRINTSLN("I'm awake!");

    // cancel sleep as a precaution
    // sleep_disable();
  
    // Re-enable ADC if it was previously running
    ADCSRA = prevADCSRA;
    pinMode(Switch_Pin0,INPUT_PULLUP);//set the Switch 0 as an input 5 High enable internal pullup 
    pinMode(Switch_Pin1,INPUT_PULLUP);//set the Switch 1 as an input 6 High enable internal pullup 
    pinMode(Switch_Pin2,INPUT_PULLUP);//set the Switch 2 as an input 7 High enable internal pullup 
    if (digitalRead(Switch_Pin2)==0 ) // supervisor Switch on
    {
      SupervisorMode = 1;
    }
    // if (SupervisorMode == 1 ) DoRadioTx(); // let Tiller know we are supervisor to reestablish Supervisor

    digitalWrite(ARC_POWER_PIN, HIGH);  // turn ARC LED strip back on
    TimeNow = millis();  // to cancel sleep till next delay
    TimeLast = TimeNow;

    // Radio was powered down so it needs to be setup again...
    // (Applies to MC customised Arduino board only)
    setup_radio();

  } // End DoSleep



// When wakePin is brought LOW this interrupt is triggered FIRST (even in PWR_DOWN sleep)
void wakeISR() {
  // Prevent sleep mode, so we don't enter it again, except deliberately, by code
  sleep_disable();
  noInterrupts();
  PCIFR  |= bit (PCIF2);   // clear any outstanding interrupts
  // Detach the interrupt that brought us out of sleep
  // was ******** > detachInterrupt(digitalPinToInterrupt(Button_Pin2));  // wakePin

  // Now we continue running the main Loop() just after we went to sleep
}  // End wakeISR


void driveServo() {
   //PRINTI("\n ServoPos = ", ServoPos); 
   myservo.write(ServoPos);
 }


void loop(void) {            // ********* Main Loop - The Receiver Code
  // picture loop 
  numPasses = 0;
//  /***      Not Used now ***********
   pinMode(DBGLED_PIN, OUTPUT);
   //digitalWrite(DBGLED_PIN, HIGH);  // Turn on LED
   pulseLED = 1; // Drives the LED with a single short 10ms pulse instead of leaving it on.

   if (DRIVESERVO) {
     ServoPos = map(buf[3], 1, 254, SERVO_LLIM, SERVO_RLIM);
     driveServo();
   } // DRIVESERVO
   
//   ***/
  #if OLED //******************************************* OLED **********************************************
  char m_str[3];
  strcpy(m_str, u8x8_u8toa(TillerPos, 3)); 
  #endif //******************************************* OLED **********************************************
   TimeNow = millis();
  
  // LED update timer...
  if (TimeNow > t_NextFlash) {
    t_NextFlash = TimeNow + LED_flash_time;
    flashLED();
  }

  // Joystick reader timer...
  if (TimeNow > (TimeLast + TBounce)) // greater than last time joystick activated + Bounce Time
  {
   motorcontrol[0] =0; //clear switches each pass
   motorcontrol[1] =0;
   readJoystick(motorcontrol); // get Joystick input values to motorcontrol 0, 1
  }


  #if OLED //******************************************* OLED **********************************************
  u8g2.firstPage();  
  do {
     numPasses++;
     u8g2_prepare();
     u8g2.drawFrame(1,1,MaxDisplayWidth-1, MaxDisplayHeight-1);
     u8g2.drawStr( 4, MaxDisplayHeight-10, "Port");
     u8g2.drawStr((MaxDisplayWidth/2)-20, 10, "Tiller");
     u8g2.drawStr( MaxDisplayWidth-55, MaxDisplayHeight-10, "Starboard");
     if (RX_state == 0) u8g2.drawStr(10,40, "No Init ");
     if (RX_state == 1) u8g2.drawStr(10,40, "Init OK ");
     if (RX_state == 2)  u8g2.drawStr(10,40, "Wait RX ");
     if (RX_state == 3) u8g2.drawStr(10,40, "Data RX ");
     if (RX_state == 4)  u8g2.drawStr(10,40, "No Comms");
     //u8g2.drawStr(70,40, "numP ");   // debug no passes
     
     
#endif //******************************************* OLED **********************************************

     LedPos = map(TillerPos,0,MaxDisplayWidth-1, 0,12); // TillerPos starts as 0 - 255
     LedPosMidShip = (TillerPos > ((MaxDisplayWidth/2)-6)&& (TillerPos < ((MaxDisplayWidth/2)+6)) );
  #if OLED  //******************************************* OLED **********************************************
     u8g2_line(TillerPos);  // Draw Line from Tiller Data from Lin Pot); 
     
     u8g2.drawStr(100,10,m_str); // TillerPos to 3 places
    // strcpy(m_str, u8x8_u8toa(numPasses, 3));  // debug no passes
    // u8g2.drawStr(100,40,m_str);
  #endif //******************************************* OLED **********************************************
    
     
     if (abortClutch > 0 ){
      if (wait_ok % 3 == 0) ledArcWrite(strip.Color( 30, 0 ,  30), LedPos);  // Yellow = 100 to Blink every 3rd
      else ledArcWrite(strip.Color( 128, 0 , 128), LedPos);  // Yellow = 250  Magenta strip.Color(128, 0, 128)
      }
      else {
              
        if (LedPos < 6 )   // Red Port
        {
          if (wait_ok % 3 == 0) ledArcWrite(strip.Color(  80, 0,   0), LedPos);  // Green = 100 to Blink every 3rd
         else  ledArcWrite(strip.Color(  250, 0,   0), LedPos);  // Green = 250
          
        }
        else 
        {
          if (wait_ok % 3 == 0) ledArcWrite(strip.Color(  0, 80,   0), LedPos);  // Green = 100 to Blink every 3rd
         else  ledArcWrite(strip.Color(  0, 250,   0), LedPos);  // Green = 250
        }
       
      }
      
    #if OLED //******************************************* OLED **********************************************
     if (motorcontrol[0] == 1) u8g2.drawTriangle(108,24,  118,32,  108,40); // ram out go Right Tiller Left
     if (motorcontrol[1] == 1) u8g2.drawTriangle(20,24,  10,32,  20,40); //ram in go Left Tiller Right
    #endif //******************************************* OLED **********************************************

   /*******
    if   (1 < 0)   // never! ((SupervisorMode == 1) && (SuperAck == 0))  // (1 < 0)    First pass not acked by tiller
    {
      DoRadioTx();
      delay(20);
      PRINTSLN(" DoRadioTx  Not Acked ");
    }
    else 
    {  
      *****/
     DoRadioRx();
     delay(20);
     // PRINTSLN(" DoRadioRx ");
     wait_last++ ;
     // PRINTI("wait_ok = ", wait_ok);
     // PRINTI("wait_last = ", wait_last);
     // PRINTI("wait_diff = ", wait_last-wait_ok);
      if (SupervisorMode == 1)
      {
        if ((wait_last - wait_ok) > 80)  
        {
          PRINTI("wait_diff = ", wait_last-wait_ok);
          PRINTSLN(" SuperAck Cancelled "); 
        SuperAck = 0; // Tiller does not know we are here 
        }
      }
   // }
   
    
    #if OLED //******************************************* OLED **********************************************
   delay(50);
  } while( u8g2.nextPage() );
   #endif //******************************************* OLED **********************************************
   

  delay(20);  // was 50
  // delay between each page
  #if OLED //******************************************* OLED **********************************************
    delay(100);
  #endif //******************************************* OLED **********************************************
  
  // Is it time to "go to sleep" ?
  TimeNow = millis();

  // Check if it's close to sleep-time
  if (TimeNow > (TimeLast + TSleepNear))  // greater than last time joystick activated + Delay TSleepNear
    NearToSleep = true;
  else
    NearToSleep = false;

  // Check if it's time to go to sleep
  if (TimeNow > (TimeLast + TSleep)) {    // greater than last time joystick activated + Delay TSleep
    GoingToSleep = true;
    DoSleep();
  }
  else
    GoingToSleep = false;

} // End Loop

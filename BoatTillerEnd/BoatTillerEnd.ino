/* BoatTillerEnd.ino
 * Tiller Ram end receives Rx TX for Handheld, controls Relays and sends back Tiller Position
 * Developed for the Arduino Uno, BTN8982TA based Infineon Motor Control Shield and nRF24L01(original or '+') radio module
 * Hardware modifications modify the connections to the motor shield to allow SPI interface pins to be used for the nRF24L01

 * Changes required to IfxMotorControlShieldUtil.cpp from Infineon:

 * // pre-instantiated Halfbridge instances
 * //Halfbridge out1 = { .active = FALSE, .pinIn = 3, .pinInh = 12, .pinIs = A0 };
 * //Halfbridge out2 = { .active = FALSE, .pinIn = 11, .pinInh = 13, .pinIs = A1 };
 * // Modified to be compatible with Bruce Boats Tiller Controller Driver
 * Halfbridge out1 = { .active = FALSE, .pinIn = 3, .pinInh = 6, .pinIs = A0 };
 * Halfbridge out2 = { .active = FALSE, .pinIn = 9, .pinInh = 5, .pinIs = A1 };
  
*/

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

// Tiller Control / IFX Motor Shield
// Use two Infineon BTN8923 half-bridge drivers to provide bi-directional control to the 'Ram' motor
// Default motor control library uses pins 3 & 11 to provide PWM control to the two half-bridge drivers
// Default motor control library uses pins 12, & 13 to provide on/off control to the two half-bridge drivers
// Default motor control library uses pins A0 & A1 to provide analogue current sense from the two half-bridge drivers
// Custom ATMEGA328 Hardware (similar to but NOT an Arduino Uno) configuration re-wires motor control to use pins 3 & 9 for PWM (was 3 & 11)
// Custom ATMEGA328 Hardware (similar to but NOT an Arduino Uno) configuration re-wires motor control to use pins 6 & 5 for ENH (was 12 & 13)

// Cluch control pin
#define ramClutch       7   // Drives something (?) to operate the clutch...

// DIP Switch
#define Switch_Pin0     A3  // Dil Switch 0 on An-Dig IP 5 Channel Low  byte
#define Switch_Pin1     A4  // Dil Switch 1 on An-Dig IP 6 Channel High byte
#define Switch_Pin2     A5  // Dil Switch 2 on An-Dig IP 7 Supervisor

// Tiller Position Sense Linear Pot
#define linPot          A1  // (Was A2) Note - A1 conflicts with Motor Driver Shield - needs to be returned to A2

// Tiller Simulator Servo
#define SERVO_PIN       9   // Incompatible with Motor Shield changes above but not used (SIM_Servo 0)

// Battery Voltage sense
#define BatV_PIN        A5  // Senses 5V voltage through 10k/10k divider (5V => 2.5V norm)

// *****************************************
// *****         Configuration         *****

// Define addresses for radio channels
#define CLIENT_ADDRESS  1   // We are Client
#define SERVER_ADDRESS  2   // Remote Handset 2 is Server
#define SUPER_ADDRESS   3   // Remote Handset 3 is Supervisor

// *****************************************
// ***** Debugging switches and macros *****

#define OLED 0              // not used now
#define DEBUG 1             // Switch debug output on and off by 1 or 0
#define SIM 0               // sim Tiller (outputs simulated tiller position to COM, drives tiller simulation servo)
#define SIM_Servo 0         // Use Tiller Simulation Servo

#if DEBUG
#define PRINTS(s)       { Serial.print(F(s)); }
#define PRINTI(s,v)     { Serial.print(F(s)); Serial.print(v); }
#define PRINT(s,v,dec)  { Serial.print(F(s)); Serial.print(v,dec); }
#define PRINTX(s,v)     { Serial.print(F(s)); Serial.print(F("0xSerial.print(v, HEX); }
#else
#define PRINTS(s)
#define PRINTI(s,v)
#define PRINT(s,v,dec)
#define PRINTX(s,v)
#endif

// *****************************************
// *****          Libraries...         *****

#include <Arduino.h>
#include <IfxMotorControlShield.h>  // Infineon official Driver
#include <PWMServo.h>               // Servo driver library (Note - this library works better than servo.h in this design)

// Include RadioHead ReliableDatagram & NRF24 Libraries
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>

// Include dependant SPI Library 
#include <SPI.h>
#include <Wire.h>


// *****************************************
// *****        Initialisation         *****

#if SIM_Servo
// Initialise Tiller Simulation Servo
PWMServo myservo;
#endif

int Counter = 0;

// Define Linear Pot Values - Start at 512 (middle position) - for simulation with joystick
int linPotpos = 512;  // 0 - 512 - 1024
float avgPotpos = 0;

#define MEASURE_PERIOD  5           // measurement time, * 2 ms eg 500 Periods of 2ms = 1 sec
#define ADC_RESOLUTION  4.8828125   // resolution of ADC, mV (5000 mV / 1024)
#define OFFSET          0.          // output voltage offset, 2494mV (for ACS712) ~ 0 - Mid scale
#define SCALE_FACTOR    400.0       //  scale factor, 66 mV/Amp (for ACS712)
unsigned long timenow;
int timeCount;                      // measurement time counter
long sumA3;                         // variable to add ADC codes
long averageCurrent;                // average current value (sum of ADC codes, average value * 500)

float current = 0. ;                // calculated current, Amps
float maxCurrent = 160.0;           // current above which Clutch disabled 

float Q0 = 0.0185;
float Q1 = 0.3214;
float Q2 = 0.2327;

unsigned long TimeNow = 0 ;
unsigned long TimeSupLast = 0 ;
unsigned long TimeServLast = 0;
unsigned long TimeAnalogLast = 0;
unsigned long TimeClutchLast = 0;
unsigned long TimeSupDelay = 200;     // 500 mSec
unsigned long TimeServDelay = 200;    // 100 msecs 
unsigned long TimeAnalogDelay = 100;  // 500 mSec
unsigned long TimeClutchDelay = 100;  // 5000 = 5 Secs 100 = 100mSec

int speedvalue= 100;                // 100
int initSpeed = 230;                // 230
int maxSpeed = 254;                 /* ( speed 0 - 255 ) */
int acceleration = 10;
int LastDir = 3;                    // 3 halt, 1 left, 0 right
int LastClutch = 0;                 // Zero at drive action
int ClutchDelay = 200;              // count up to Delay then disable Clutch ###################
float rawCurrent = 0. ;
int leftSpeed = 0;
int rightSpeed = 0;

// Create an instance of the radio driver
RH_NRF24 RadioDriver;

// Sets the radio driver to NRF24 and the client address to 1
RHReliableDatagram RadioManager(RadioDriver, CLIENT_ADDRESS);

uint8_t RadioChanNo = 0;
uint8_t Super_Online = 0;

// Declare unsigned 8-bit motorcontrol array
// 2 Bytes for motor speeds plus 1 byte for direction control

uint8_t motorcontrol[4];            // in, out, clutch, linPot,...
uint8_t supercontrol[4];            // in, out, clutch, linPot,...
uint8_t maxTiller = 25;             // tiller pos to Endstop 
uint8_t RX_state = 0;

uint8_t MaxDisplayWidth = 128 ;
uint8_t MaxDisplayHeight = 96 ;

uint8_t TillerPos = 0;
uint8_t lastTillerPos = 0;
uint8_t TillerStopCount = 0;
uint8_t sim_Tiller = 0;
uint8_t draw_dirn = 0;

// debugs
uint8_t ServoPos = 80;
uint8_t wait_fail = 0;
uint8_t just_failed = 0;
uint8_t wait_ok = 0;

uint8_t reTrys;

// *****************************************
// *****          Functions            *****

void CallSuper();
void CallHandset();

// Define the Message Buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

/****  DoRadioSuperTx  &  DoRadioTx  send calls to the Super and Normal Handsets
 * 
 *    RadioManager.sendtoWait sends the message (with retries and timeout) and waits for an ack.
 *    if ack received - 
 *    RadioManager.recvfromAckTimeout  waits for valid message available from this node or timeout (Starts the receiver automatically)
 * 
 *    the Handsets reply on ack - and respond with the Buffer Data - at present a 4 byte array [in, out, clutch, linPot]
 *    in, out, clutch, are incomming from Handsets and linPot is outgoing from Tiller
 *    the in, out, and clutch are commands to the Tiller based on Switch movement at the Handset
 *    the linPot is essentially a Tiller position reply from the Tiller, used to display position at the Handset
 *    The linPot is in the reply message Buffer Data Array
 * 
 * 
 * **********/


void DoRadioSuperTx(int16_t timeOut) {
  /***********************************************/
  //Send a message containing Motor Control data to Super

  if (RadioManager.sendtoWait(supercontrol, sizeof(supercontrol), SUPER_ADDRESS)) // reply to Supervisor TX
    /// Send the message (with retries) and waits for an ack. Returns true if an acknowledgement is received.
    /// Synchronous: any message other than the desired ACK received while waiting is discarded.
  {
    //Super_Online = 1; 
    //PRINTS("\n Super Online ");
    
     // Now wait for a reply from the server
    uint8_t len = sizeof(buf);    
    uint8_t from;      
    
    if (RadioManager.recvfromAckTimeout(buf, &len, timeOut, &from))  // was 500 
    /// this will block until either a valid message available for this node
    /// or the timeout expires. Starts the receiver automatically.
    {
      
      wait_ok++;
      supercontrol[2] = buf[2]; //Clutch
       
      supercontrol[0] = buf[0];  //ramOut
      
      supercontrol[1] = buf[1]; //ramIn
      Super_Online = 1; 
      PRINTS("\n SuperRX Online ");
    }//Reply received 
    else
    {
      PRINTS("\n No reply Sup, is nrf24_reliable_datagram_server running?");
     
    }  // No Reply 
  }  // Send Data 
  else
   {
    // timenow = millis();
    PRINTS("\n no Super online ");
    Super_Online = 0; 
    }
   
}  // End of DoRadioSuperTx

/*****
void DoSuperRadioRx(void) // Got Tx from Supervisor  -- not now used !!!!!!!!!!!
{
  //
 if (RadioManager.waitAvailableTimeout(10))  // Data Received  - should be from Supervisor - if no data timeout 80ms 
  {
 //
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (RadioManager.recvfromAck(buf, &len, &from))  //get message from Tiller - buf
    {
     if (from == SUPER_ADDRESS)  // message from Supervisor
     {
      Super_Online = 1 ;  
      PRINTI("\n Got Super TX from =", from);
     }
     else
     {
      PRINTI("\n wrong address came from =", from);
     }
    
    }// RX OK
   else
   {
     PRINTS("\n Timeout from Super");
   }
    
 // Send a reply back to the originator client, check for error
      if (!RadioManager.sendtoWait(buf, sizeof(buf), from)) 
        { PRINTS("\n  SupRX sendtoWait failed") ; //Serial.println("  SupRX sendtoWait failed"); 
       
        }
  //      
  }   //RadioManager available
  else PRINTS("\n no Super Data RX ");   // Serial.println("\n no Super Data RX ");
  //
} // End DoSuperRadioRx
***************/


void DoRadioTx(int16_t timeOut) {
  /***********************************************/
  //Send a message containing Motor Control data to manager_server
  if (RadioManager.sendtoWait(motorcontrol, sizeof(motorcontrol), SERVER_ADDRESS)) 
                                               // sends 4 buf motorcontrol values to Handset Nano
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);    
    uint8_t from;      
    if (RadioManager.recvfromAckTimeout(buf, &len, timeOut, &from))  // was 500 wait
    {
      //Serial.print("got reply from : 0x");
      RX_state = 3; //u8g2.drawStr(10,40, "Data RX ");
      wait_ok++;
      PRINTI("\n\n sim_Tiller = ", sim_Tiller);    
      PRINTI("\n TillerPos = ", TillerPos);    
      PRINTI("\n got reply from ", SERVER_ADDRESS);
      PRINTI("\n Wait fail =", wait_fail);
      PRINTI("\n Wait 0k =", wait_ok); 
      PRINTI("\n JustFailed =", just_failed); 

     
      if ((wait_ok > 5) && (wait_fail > 5) && (TimeNow < (TimeClutchLast + TimeClutchDelay)))  // (LastClutch < ClutchDelay)) // can have clutch on
         {
          wait_fail = 0; //reset fail if radio back 
          just_failed = 0;
          motorcontrol[2] = 1; //turn clutch back on
         }
       else  motorcontrol[2] = buf[2]; //Clutch
       
      motorcontrol[0] = buf[0];  //ramOut
      
      motorcontrol[1] = buf[1]; //ramIn
      
      /************/
    }//Reply received 
    else
    {
      PRINTS("\n No reply Handset, is nrf24_reliable_datagram_server running?");
      RX_state = 4; //u8g2.drawStr(10,40, "No Reply");
    }  // No Reply 
  } // Send Data 
  else
  {
    PRINTI("\n\n sim_Tiller = ", sim_Tiller);    
    PRINTI("\n TillerPos = ", TillerPos);    
    PRINTS("\n RTX sendtoWait failed");
    wait_fail++;
    RX_state = 2; //u8g2.drawStr(10,40, "Wait Fail");
    PRINTI("\n Wait fail = ", wait_fail);
    PRINTI("\n Wait Ok = ", wait_ok); 
    PRINTI("\n JustFailed = ", just_failed); 
  }
  //  RadioManager.recvfrom(); //turns on receiver
  } //  end DoRadioTx
  

  void initRadio() {
      // Initialize RadioManager with defaults - 2.402 GHz (channel 2), 2Mbps, 0dBm
        if (!RadioManager.init())
          {PRINTS("\n init failed");
            RX_state = 0;}
        else  {PRINTS("\n Radio initialized OK");
          RX_state = 1;
          }
          
      //if (digitalRead(Switch_Pin1)== 0 ) // Chan Hi Switch on
      if (true) // Chan Hi Switch on (FORCE ON WHILE NO SWITCH INSTALLED)
      {RadioChanNo = 2;
      }
      else
      {
        RadioChanNo = 0;
      }
      //if (digitalRead(Switch_Pin0)== 0 ) // Chan Lo Switch on
      if (false) // Chan Lo Switch on (FORCE OFF WHILE NO SWITCH INSTALLED)
        RadioChanNo = RadioChanNo + 1;  // results in Chan 0 - 3
        RadioDriver.setChannel(120 + RadioChanNo); // Channel 120 - 123
        Serial.print(" Radio Channel ");
        Serial.println(RadioChanNo);

          
          // RadioDriver.setChannel(124);
          RadioManager.setTimeout(100);
          RadioManager.setRetries(2);  // was  2  back to 2 from 3
          reTrys = RadioManager.retries();
          PRINTI("\n : reTrys: ", reTrys);
          
        if (!RadioDriver.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))  //12dBm
              {PRINTS("\n setRF failed"); 
            RX_state = 0;}
        else  {PRINTS("\n Radio initialized OK");
          RX_state = 1;
          }
        // radio.setChannel(124);
        // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
      /**********************************  set various radio params eg: different chan **************
        if (!nrf24.setChannel(1))
          Serial.println("setChannel failed");
      **************************/

      /**********************************  set different Data Rate **************   
        if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
          Serial.println("setRF failed");  
          **************************/
      /*********************************
         DataRate { DataRate1Mbps = 0, DataRate2Mbps, DataRate250kbps }
          TransmitPower {
        lowestTransmitPowerm18dBm = 0, TransmitPowerm12dBm, TransmitPowerm6dBm, TransmitPower0dBm, highest
        RFM73TransmitPowerm10dBm = 0, RFM73TransmitPowerm5dBm, RFM73TransmitPowerm0dBm, RFM73TransmitPower5dBm
      *******************/

        
      /**************** 
         if  (!RadioDriver.setTxPower(14))
          Serial.println("setRF failed");
        else  Serial.println("Radio setRF OK");
        *******************/

        

  }  // initRadio


void setup()
{
// Setup Serial Monitor
   
  Serial.begin(115200);  // Put Filename out so we know what it is !
  Serial.print("Compiled by Bo:  ");
  Serial.println(__FILE__);
  Serial.println("Delay Task Ver.");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  
  PRINTS("\n Program started");
  if(ifxMcsBiDirectionalMotor.begin())
    PRINTS("\n Init Fail!");
  PRINTS("\n Initialising speed");
  ifxMcsBiDirectionalMotor.setBiDirectionalSpeed(0); // Init Value zero
  PRINTS("\n Starting motor");
  ifxMcsBiDirectionalMotor.start(); 

  // **** Setup I/O     *****
  
  pinMode(ramClutch, OUTPUT); // D7
  /*
  digitalWrite(ramOut, HIGH);   //set outputs off
  digitalWrite(ramIn, HIGH);
  */
  digitalWrite(ramClutch, LOW);
   
  // DIL Switch
  pinMode(Switch_Pin0,INPUT_PULLUP);//set the Switch 0 as an input 5 High enable internal pullup 
  pinMode(Switch_Pin1,INPUT_PULLUP);//set the Switch 1 as an input 6 High enable internal pullup 
  pinMode(Switch_Pin2,INPUT_PULLUP);//set the Switch 2 as an input 7 High enable internal pullup 

  // ************************

  // ***** Option I/O config for when Tiller SImulation Servo is enabled...
  #if Sim_Servo
  // declare the Tiller Simulator Servo pin as an OUTPUT:
  pinMode(SERVO_PIN, OUTPUT);

  // Attach the Tiller Simulator Servo:
  myservo.attach(SERVO_PIN);

  // Move the Tiller Simulator Servo to the middle
  myservo.write(ServoPos);
  #endif

  // ************************

  
  TillerPos = (MaxDisplayWidth/2)-1;  // init TillerPos
  sim_Tiller = TillerPos;

  // Initialise the Radio service
  initRadio();

  // Set initial motor values to zero
  motorcontrol[0] = 0; // ramOut
  motorcontrol[1] = 0; // ramIn
  motorcontrol[2] = 0; // Clutch
  motorcontrol[3] = 0; // Tiller Pos Analog
  lastTillerPos = 0;
  buf[0] = 0; // ramOut
  buf[1] = 0; // ramIn
  buf[2] = 0; // Clutch
  buf[3] = 0; // Tiller Pos Analog

}  // setup

void halt() {
  ifxMcsBiDirectionalMotor.stop();
   }

void left(int motorSpeed) { // Drive Out
   ifxMcsBiDirectionalMotor.setBiDirectionalSpeed(motorSpeed);
    if(!ifxMcsBiDirectionalMotor.getRunning())
        ifxMcsBiDirectionalMotor.start();
   }

void right(int motorSpeed) { // Drive In 
  ifxMcsBiDirectionalMotor.setBiDirectionalSpeed(-motorSpeed);
   if(!ifxMcsBiDirectionalMotor.getRunning())
        ifxMcsBiDirectionalMotor.start();
  }

float runAvg(float factor, float lastAvg, float newVal){  //fact, lastAvg, New
  newVal = ((1.0 - factor)* newVal)+ (factor * lastAvg);
  return newVal;
}
  

float linVolts(float rawVolts){  // linearize with 2nd order Poly
 rawVolts = rawVolts /100.0;
 float lin = ((rawVolts * rawVolts * Q2) + (rawVolts *  Q1) + Q0)*100;
  return lin ;
}

void doCurrent() {
  rawCurrent = (float)ifxMcsBiDirectionalMotor.getCurrentSense(); //read Current fron Infineon
   current = runAvg(0.5, current, rawCurrent); //fact, Avg, New
  if (current > maxCurrent) // Disable Clutch if current exceeds maxCurrent
      {
      // motorcontrol[2] = 0;  // clutch off
      }
   
   //PRINT("\n  Avg Current: ", current,3);
  // doAvgCurrent();  not used now
}

void readPot(){
// Read the Joystick X and Y positions to simulate Linear Pot
  linPotpos = analogRead(linPot); 
  avgPotpos = runAvg(0.1, avgPotpos, linVolts(float(linPotpos)) ); //Returns NewAvg from (fact, Avg, New)
  // PRINTI("\n AvgPotpos = ", avgPotpos);

  if (SIM_Servo) {
    motorcontrol[3] = map(ServoPos, 30, 130, 1, 254);
  }
  else {
    motorcontrol[3] = map((int)avgPotpos, 685, 2, 1, 254); //Pot posn 0 -  255); Pot only Half Scale!!! 530 - 40
    //                                    475, 1.8
  }
  supercontrol[3] = motorcontrol[3];  // copy tiller for Super 
  if (abs(motorcontrol[3] - lastTillerPos) < 2) {
     TillerStopCount++ ;
  }
  else  TillerStopCount = 0;
    
  lastTillerPos = motorcontrol[3];  // save for test if tiller moves
  // PRINTI("\n mot3: ", motorcontrol[3]);
  TillerPos = map(motorcontrol[3], 0, 254, 0, MaxDisplayWidth-1); //TillerPos  0 -  128);
  //PRINTI("\n TillerPos: ", TillerPos);
 // motorcontrol[3] = linPotpos;
} // readPot


void simTiller() {          /*******  for sim tiller ********/
    //Display the Motor Control values in the serial monitor Com5.
      // set sim_Tiller
      if ( ((motorcontrol[0] > 0)||(supercontrol[0] > 0)) && (sim_Tiller < MaxDisplayWidth-5)) {
        sim_Tiller = sim_Tiller +5; //clamp to 0 - MaxDisplayWidth
      }
      if ( ((motorcontrol[1] > 0)||(supercontrol[1] > 0)) && (sim_Tiller > 5)) {
        sim_Tiller = sim_Tiller -5;
      }
     if (( TillerPos > 5))   // ((MaxDisplayWidth/2)-2)) || ( TillerPos > ((MaxDisplayWidth/2)+1)))
 
     {

      
   
     sim_Tiller = TillerPos; // for Bumpless
     }
     
     // ############# sim ############
     else // pot close to zero so can sim_Tiller);
         { 
          #if SIM
           motorcontrol[3] = map(sim_Tiller, 0, MaxDisplayWidth-1, 0, 255); //Pot posn 0 -  255); 
           supercontrol[3] = motorcontrol[3];
          #endif 
        
         }
}  // simTiller


void driveServo() {
   #if Sim_Servo
   myservo.write(ServoPos);
   #endif
 }


void driveMotors() {
  // Stop Clutch if too many RX Fails 
  if ((wait_fail > 5 ) && (just_failed == 0))   //oneshot
  { 
    just_failed = 1;
    motorcontrol[2] = 0;  // clutch off - too many RX fails
    PRINTS("\n Clutch off.. RX fail ");
    wait_ok = 0; //reset wait_ok
  }
         
   
   // TimeClutchLast = millis();         // LastClutch = LastClutch + 1; //increment to cut clutch after delay
  //PRINTI("\n LastClutch = ", LastClutch);

   
  if (Super_Online == 1) //do super Outputs       ########### Drive Motors #############
  {  
    if (supercontrol[0] == 1) // Left drive output
   { LastClutch = 0; //start clutch after delay
    if (LastDir != 1) // changed Dir
     {
      LastDir = 1;
      speedvalue = initSpeed;
      delay(200);  // should probably  wait longer than 20 mSec  before change dir was 100
     }
     else  // same dir
      {speedvalue = speedvalue + acceleration;
       if (speedvalue > maxSpeed) speedvalue = maxSpeed;
      }
       left(speedvalue);  /* left( speed 0 - 255 ) */
     }  // go left 
   
    else {
      if (supercontrol[1] == 1) // right
      {  LastClutch = 0; // start clutch after delay
        if (LastDir != 0) // changed Dir
         {
          LastDir = 0;
          speedvalue = initSpeed;
          delay(100);  // should probably  wait longer than 20 mSec  before change dir
         }
       else //same dir
        {  speedvalue = speedvalue + acceleration;
           if (speedvalue > maxSpeed) speedvalue = maxSpeed;
        }  // go right
        right(speedvalue);  /* right( speed 0 - 255 ) */
      }
        else {
                halt();
                speedvalue = initSpeed;
                LastDir = 3;
             }
    }
  }  //do super Outputs 
  else                   //Super Not Online       ########### Drive Motors #############
  {
    if ((motorcontrol[0] == 1) )   // left drive output            // && ((motorcontrol[3] < (254 - maxTiller))|| (TillerStopCount 10)  )) 
   { LastClutch = 0; //start clutch after delay
      TimeClutchLast = millis(); 
    if (LastDir != 1) // changed Dir
     {
      LastDir = 1;
      speedvalue = initSpeed;
      delay(200);  // should probably  wait longer than 20 mSec  before change dir
     }
     else  // same dir
      {speedvalue = speedvalue + acceleration;
       if (speedvalue > maxSpeed) speedvalue = maxSpeed;
      }
       left(speedvalue);  /* left( speed 0 - 255 ) */

     if (SIM_Servo) {
       if (ServoPos < 130)
         ServoPos = ServoPos + 5;
       else
         ServoPos = 130;
       driveServo();       
     }
   }  // go left 
   
    else {
      if ((motorcontrol[1] == 1) )   // right drive output             //  && ((motorcontrol[3] > (1 + maxTiller))|| (TillerStopCount <10)    ) ) 
      {  LastClutch = 0; // start clutch after delay
         TimeClutchLast = millis(); 
        if (LastDir != 0) // changed Dir
         {
          LastDir = 0;
          speedvalue = initSpeed;
          delay(200);  // should probably  wait longer than 20 mSec  before change dir
         }
       else //same dir
        {  speedvalue = speedvalue + acceleration;
           if (speedvalue > maxSpeed) speedvalue = maxSpeed;
        }  // go right
        right(speedvalue);  /* right( speed 0 - 255 ) */
      
       if (SIM_Servo) {
         if (ServoPos > 30)
           ServoPos = ServoPos - 5;
         else
           ServoPos = 30;
         driveServo();       
       }
      }
        else {
                halt();
                speedvalue = initSpeed;
                LastDir = 3;
             }
    }
  }  //Super Not Online
   
    

  if (Super_Online == 1)  // Super turns on Clutch if any Output 
  {
      digitalWrite(ramClutch, (supercontrol[0] || supercontrol[1])); //set Clutch if Super Output
     // digitalWrite(ramClutch, 0); //reset Clutch
  }
  else                    // Server turns on Clutch if not > ClutchDelay
  {
     // PRINTI("\n TimeNow: ", TimeNow);
     // PRINTI("\n TimeClutchLast: ", TimeClutchLast);
  if  (TimeNow < (TimeClutchLast + TimeClutchDelay))  // (LastClutch < ClutchDelay) // OK to output to Clutch
   {
    digitalWrite(ramClutch, motorcontrol[2]); //set Clutch (was !motorcontrol[2]) 
   }
   else 
   {
    digitalWrite(ramClutch, 0); // cut clutch if >= Clutch Delay
   }
  }
 } // driveMotors

 void clearMotors() {
   motorcontrol[0] = 0; //Clear it so next RX read needs to be valid to increase more
   motorcontrol[1] = 0;
   supercontrol[0] = 0; //Clear it so next RX read needs to be valid to increase more
   supercontrol[1] = 0;
 }

void loop()          /*************** Main Loop ****************************/
{
  // Print to Serial Monitor
  /* Serial.println("Reading motorcontrol values "); */

  TimeNow = millis();
  // doCurrent();
  if (TimeNow > (TimeAnalogLast + TimeAnalogDelay)) {//  100 mSec
    readPot();
    TimeAnalogLast = millis();  // dont do another before TimeAnalogDelay passed
  }

  #if SIM
   simTiller();
  #endif


  /***********
  
  rawCurrent = (float)ifxMcsBiDirectionalMotor.getCurrentSense(); //read Current fron Infineon
   current = runAvg(0.5, current, rawCurrent); //fact, Avg, New
  if (current > maxCurrent) // Disable Clutch if current exceeds maxCurrent
      {
      // motorcontrol[2] = 0;  // clutch off
      }
   
   //PRINT("\n  Avg Current: ", current,3);
  // doAvgCurrent();  not used now
  
  // Read the Joystick X and Y positions to simulate Linear Pot
  linPotpos = analogRead(linPot); 
  avgPotpos = runAvg(0.1, avgPotpos, linVolts(float(linPotpos)) ); //Returns NewAvg from (fact, Avg, New)
  PRINTI("\n AvgPotpos = ", avgPotpos);

  motorcontrol[3] = map((int)avgPotpos, 685, 2, 1, 254); //Pot posn 0 -  255); Pot only Half Scale!!! 530 - 40
  //                                    475, 1.8
  supercontrol[3] = motorcontrol[3];  // copy tiller for Super 
  if (abs(motorcontrol[3] - lastTillerPos) < 2) {
     TillerStopCount++ ;
  }
  else  TillerStopCount = 0;
    
  lastTillerPos = motorcontrol[3];  // save for test if tiller moves
  PRINTI("\n mot3: ", motorcontrol[3]);
  TillerPos = map(motorcontrol[3], 0, 254, 0, MaxDisplayWidth-1); //TillerPos  0 -  128);
  //PRINTI("\n TillerPos: ", TillerPos);
 // motorcontrol[3] = linPotpos;



  
  //


  //Display the Motor Control values in the serial monitor Com5.

  if ( ((motorcontrol[0] > 0)||(supercontrol[0] > 0)) && (sim_Tiller < MaxDisplayWidth-5)) sim_Tiller = sim_Tiller +5; //clamp to 0 - MaxDisplayWidth
  if ( ((motorcontrol[1] > 0)||(supercontrol[1] > 0)) && (sim_Tiller > 5)) sim_Tiller = sim_Tiller -5;
 
  
  //
  // picture loop 
  char m_str[3];
  

    //
     if (( TillerPos > 5))// ((MaxDisplayWidth/2)-2)) || ( TillerPos > ((MaxDisplayWidth/2)+1)))
 
     {
     
     sim_Tiller = TillerPos; // for Bumpless
     }
     
     // sim #######
     else // sim_Tiller);
         { 
          #if SIM
           motorcontrol[3] = map(sim_Tiller, 0, MaxDisplayWidth-1, 0, 255); //Pot posn 0 -  255); 
           supercontrol[3] = motorcontrol[3];
          #endif 
        
         }
         *********/

  driveMotors();
   
  /**********
   motorcontrol[0] = 0; //Clear it so next RX read needs to be valid to increase more
   motorcontrol[1] = 0;
   supercontrol[0] = 0; //Clear it so next RX read needs to be valid to increase more
   supercontrol[1] = 0;
   Counter++ ;  //increment Counter by 1
  ***********/
   /*************
   if (Super_Online == 1)  // have received Tx from Super
    {
        clearMotors();   //Clear it so next RX read needs to be valid to increase more
        
        Counter++ ;  //increment Counter by 1

      // if (Counter % 2 == 0) // even passes
     
         DoRadioTx(100); // Tx to Handset and get data from Handset 
         delay(30);  // #### was 10
      
         DoRadioSuperTx(100);  // Tx to Super and get data from Super
          delay(30);
         // PRINTI("\n Odd Counter = ", Counter);
            
    }
    else                  // No Super Detected yet
    {
      ***************/
      // TimeNow = millis();

      if (Super_Online == 1) {
        CallSuper();
        CallHandset();
      }
      else {
        CallHandset();
        CallSuper();
      }
    
  // delay(50);  // Wait a bit before next transmission
} // End Loop


void CallSuper() {
if (TimeNow > (TimeSupLast + TimeSupDelay)) //  500 mSec
      // if (Counter % 10 == 0) // even passes
        {
         //DoSuperRadioRx(); // check for data Rx from Super  was  tried before
         if (Super_Online == 1) { // have received Tx from Super
              clearMotors();   //Clear it so next RX read needs to be valid to increase more   
         }
         DoRadioSuperTx(100);  // this one checks if super online
         TimeSupLast = millis();  // dont do another before TxSuperDelay passed
          // delay(30);  // #### was 10
        }

}  // CallSuper


void CallHandset() {
 if (TimeNow > (TimeServLast + TimeServDelay)) {  //  100 mSec

              if (Super_Online == 0) { 
                clearMotors();   //Clear it so next RX read needs to be valid to increase more
              } // Server needs to clear them
              Counter++ ;  //increment Counter by 1
              DoRadioTx(100); // Tx to Handset and get data from Handset was 200
              TimeServLast = millis();  // dont do another before TimeServDelay passed
            } // delay(30);
          
        // PRINTI("\n NoS Counter = ", Counter);
        // PRINTI("\n Counter = ", Counter);          
    // }

} // CallHandset

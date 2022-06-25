// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       RoundhouseLocoNet.ino
    Created:    12/28/2018 3:46:36 PM for DCC, adapted to LocoNet 11/9/2021
    Latest revision: 1/21/2022
    Author:     Bob Gamble and others from posted code examples

This sketch uses:
1) the LocoNet, Wire, Adafruit_PWMServoDriver, I2CKeyPad and EEPROM libraries;
2) a LocoNet Shield (John Plocher's design or similar)
3) PWM / 16 Bit 16 Bit Servo Driver - I2C Interface - PCA9685
4) I2C PCF8574 IO Expansion Board Module and 4x4 Keypad

Pin usage:
The LocoNet Shield uses pins 7 & 8
The I2C interface uses D2 for interupt, A4 for SDA & A5 for SCL & Ground & +5V
The LEDs use pins as defined below

Note if a servo is in the "positive" state and the same state is reactivated,
 the servo will move rapidly in the opposite direction and then take the programmed position regularly
 in the same way if it is in the "negative" state. This happens automatically.
 In the sketch, therefore, I included the servo Status check so that what is described above does not happen.

IMPORTANT OBSERVATION:
Whenever the Arduino "goes out" the sketch will lose the memory of the servo
Status and therefore they will suffer the problem described above.

Improvements:
1) save servo states in EEPROM 
2) report feedback on LocoNet
3) keypad input for some functions 


*/
//#define DEBUG_PRINT true

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LocoNet.h>
#include<EEPROM.h>
#include<I2CKeyPad.h>

// Pin for interupt from I2C module
#define KeyPad_Int_Pin 2

// Define your LocoNet TX Pin below, RX is on 8
#define  TX_PIN   7

#define NumOfLights 2
#define Light_A 11
#define Light_B 12
  int LightPin[NumOfLights] {Light_A, Light_B};
  int LightAddr[NumOfLights] {700, 701};

int setupPin=A2;

static   lnMsg        *LnPacket;

const uint8_t KEYPAD_ADDRESS = 0x20;
I2CKeyPad keyPad(KEYPAD_ADDRESS);
char keymap[19] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail (e.g. >1 keys pressed)

// two different lay out styles of a nummeric keyPad
//char phone_layout[19]      = "123A456B789C*0#DNF";  // N = NoKey, F = Fail
//char calculator_layout[19] = "789A456B123C*0#DNF";  // N = NoKey, F = Fail

// volatile for IRQ var
volatile bool keyChange = false;

#define buffLen 10
  char buff[buffLen];
  uint8_t bufferIndex = 0;
  
uint8_t lastDirection = 0xFF;

#define i_max_servo 10   // modify as desired, you can have 16 for each PCA9685
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // to manage servos from 1  to 16 (addresses 100-115)

typedef struct
{
	int address;
	bool active;
	int Status;
	int ServoMin;
	int ServoMax;
	int Position;
}
DCCAccessoryAddress;
DCCAccessoryAddress sAddresses[i_max_servo];

void ConfigureServos()
// memory for the Status of servo:  
//			0 = deviate position
//      1 = correct position
//      2 = Status on start sketch

{	// these are Status arrays for each servo
	sAddresses[0].address = 600;	// DCC address for this servo
	sAddresses[0].active = false;	// servo in use flag
	sAddresses[0].Status = 2;		// flag for opening or closing of servo
	sAddresses[0].ServoMin = 250;	// position of servo at close
	sAddresses[0].ServoMax = 500;	// position of servo at open
	sAddresses[0].Position = 250;	// current position

	sAddresses[1].address = 601;
	sAddresses[1].active = false;
	sAddresses[1].Status = 2;
	sAddresses[1].ServoMin = 265;
	sAddresses[1].ServoMax = 500;
	sAddresses[1].Position = 265;

	sAddresses[2].address = 602;
	sAddresses[2].active = false;
	sAddresses[2].Status = 2;
	sAddresses[2].ServoMin = 235;
	sAddresses[2].ServoMax = 500;
	sAddresses[2].Position = 235;

	sAddresses[3].address = 603;
	sAddresses[3].active = false;
	sAddresses[3].Status = 2;
	sAddresses[3].ServoMin = 215;
	sAddresses[3].ServoMax = 490;
	sAddresses[3].Position = 215;

	sAddresses[4].address = 604;
	sAddresses[4].active = false;
	sAddresses[4].Status = 2;
	sAddresses[4].ServoMin = 240;
	sAddresses[4].ServoMax = 500;
	sAddresses[4].Position = 240;

	sAddresses[5].address = 605;
	sAddresses[5].active = false;
	sAddresses[5].Status = 2;
	sAddresses[5].ServoMin = 250;
	sAddresses[5].ServoMax = 500;
	sAddresses[5].Position = 250;

	sAddresses[6].address = 606;
	sAddresses[6].active = false;
	sAddresses[6].Status = 2;
	sAddresses[6].ServoMin = 255;
	sAddresses[6].ServoMax = 500;
	sAddresses[6].Position = 255;

	sAddresses[7].address = 607;
	sAddresses[7].active = false;
	sAddresses[7].Status = 2;
	sAddresses[7].ServoMin = 235;
	sAddresses[7].ServoMax = 500;
	sAddresses[7].Position = 235;

	sAddresses[8].address = 608;
	sAddresses[8].active = false;
	sAddresses[8].Status = 2;
	sAddresses[8].ServoMin = 245;
	sAddresses[8].ServoMax = 500;
	sAddresses[8].Position = 245;

	sAddresses[9].address = 609;
	sAddresses[9].active = false;
	sAddresses[9].Status = 2;
	sAddresses[9].ServoMin = 225;
	sAddresses[9].ServoMax = 500;
	sAddresses[9].Position = 225;

/*	repeat the above construct for any additional servos implemented 	*/
}

void setupLocoNet()
{
	Serial.println(F("Setting up LocoNet node..."));

    // First initialize the LocoNet interface, specifying the TX Pin
  LocoNet.init(TX_PIN);
}

void setup()
{	
	Serial.begin(115200);
	while (!Serial);   // Wait for the USB Device to Enumerate

	Serial.println(F("Start - Roundhouse Ready"));
  
  for (int Light=0;Light<NumOfLights;Light++){
    pinMode(LightPin[Light], OUTPUT);
	  digitalWrite(LightPin[Light], LOW); // turn off
  }
 
  // NOTE: PCF8574 will generate an interrupt on key press and release.
  pinMode(KeyPad_Int_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KeyPad_Int_Pin), keyChanged, FALLING);
  keyChange = false;
  
  Wire.begin();
  Wire.setClock(100000);
  if (keyPad.begin() == false)
  {
    Serial.println("\nERROR: cannot communicate to keypad.\nPlease reboot.\n");
    while (1);
  }
  keyPad.loadKeyMap(keymap);
  
	ConfigureServos();
	
	setupLocoNet();	
  
  if(analogRead(setupPin)<500){
    // populate default values in EEPROM
    Serial.print(F("Writing EEPROM with defaults "));
    for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
    {   
      EEPROM.put(4*i,(sAddresses[i].Status));
//      EEPROM.put((4*i)+2,(sAddresses[i].Position));
    }
//      EEPROM.put((4*(sizeof(sAddresses) / sizeof(DCCAccessoryAddress))),true);
  }
  else
  {
    // read values from EEPROM
    Serial.println(F("Reading EEPROM"));
    for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
    {   
      EEPROM.get((4*i),sAddresses[i].Status);
//      EEPROM.get((4*i)+2,sAddresses[i].Position);
      Serial.print(i); 
      Serial.println(sAddresses[i].Status);
//      Serial.println(sAddresses[i].Position);
      sAddresses[i].active = true;
    }
  }
  for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
    {   
      LN_STATUS lnStatus = LocoNet.reportSensor(sAddresses[i].address,sAddresses[i].Status);
    }  
  // Initialize PCA9685

	pwm1.begin(); /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
//  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
//	pwm1.setPWMFreq(60);   // frequency set to 60 Hz  - NOT modify!!!!
	delay(10);
}

void keyChanged()
{
  keyChange = true;
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages

void notifySwitchRequest( uint16_t Addr, uint8_t OutputPower, uint8_t Direction ) {

  int InDir = (Direction > 0);

// the following prints to the serial monitor for debugging purposes
#ifdef DEBUG_PRINT
  Serial.print("notifySwitchRequestOutput: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.print(Direction, DEC);
  Serial.print(',');
  Serial.print(OutputPower, DEC);
  Serial.print(',');
  Serial.print(" InDir = ");
  Serial.println(InDir, DEC);
#endif

  if ((Addr == 700)) // hard coded address for controlling LED lights from pin Light_A
  {
    LightSwitch(0,InDir);   
  }
  if ((Addr == 701)) // hard coded address for controlling LED lights from pin Light_B
  {
    LightSwitch(1,InDir); 
  }
  for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
  {   
    if ((Addr == sAddresses[i].address)  && (InDir != sAddresses[i].Status) && (OutputPower != 0))
    {
        MoveServo(i,InDir);
        break;
    }
  }
}
void MoveServo(int i, int dir)
{     if (i > (i_max_servo - 1)) return;
      Serial.print(F("Activating Servo : "));
      Serial.println(i, DEC);

      sAddresses[i].active = true;
      sAddresses[i].Status = dir;

      if (dir)
      {
        Serial.print(F("Opening : "));
        Serial.println(sAddresses[i].address, DEC);
      }
      else
      {
        Serial.print(F("Closing : "));
        Serial.println(sAddresses[i].address, DEC);
      }
}  
void LightSwitch(int Light, int dir)
{ if (Light > (NumOfLights - 1)) return;
  if (dir)
    {
      digitalWrite(LightPin[Light], HIGH); // turn on
    }
    else
    {
      digitalWrite(LightPin[Light], LOW); // turn off
    }
#ifdef DEBUG_PRINT
      LN_STATUS lnStatus = LocoNet.reportSensor(LightAddr[Light], dir);
      Serial.print(F("Tx: Sensor: "));
      Serial.print(LightAddr[Light]);
      Serial.print(" Status: ");
      Serial.println(LocoNet.getStatusStr(lnStatus));
#endif
}    
void keyPadCommand()
{
#ifdef DEBUG_PRINT
    Serial.print(F("BufferIndex: "));
    Serial.println(bufferIndex);
#endif    
  if (bufferIndex <= 1)
  {
    switch (buff[0]) {
  case '#':
    //
    
    break;
  case '*':
    // 
    
    break;
  case 'A':
    //
    Actions(0);
    break;
  case 'B':
    // 
    Actions(1);
    break;
  case 'C':
    //
    Actions(2);
    break;
  case 'D':
    // 
    Actions(3);
    break;
  default:
    // statements
    keyChange = false;
    return;
    break;
    }}
  if (bufferIndex == 2)
  {
  switch (buff[1]) {
  case '#':
    // open door
    MoveServo(buff[0] - 48,1);
    break;
  case '*':
    // close door
    MoveServo(buff[0] - 48,0);
    break;
  default:
    // statements
    break;
    }
  }   
  if (bufferIndex == 3)
  {
  switch (buff[2]) {  //ignore first character
    case '#':
      // turn on light #
      LightSwitch((buff[1] - 48),1); 
      break;
    case '*':
      // turn off light #
      LightSwitch((buff[1] - 48),0);
      break;
    default:
      // statements
      break;
    }    
  }  
  if (bufferIndex == 4)
  {
    Actions(buff[2] - 48);   //ignore first two characters 
  } 
#ifdef DEBUG_PRINT  
    Serial.print(F("Buffer: "));
    Serial.println(buff);
#endif    
      for (int i = 0; i < (sizeof(buff)); i++)
      {
        buff[i]   = 0;
      }
//        buff[bufferIndex]   = 0;
  bufferIndex = 0;
    keyChange = false;
}
void Actions(int i)
    {
      switch(i){
          case 0:
            // light toggle
            digitalWrite(Light_A, digitalRead(Light_A) ^ 1);
            break;
          case 1:
            // light toggle
            digitalWrite(Light_B, digitalRead(Light_B) ^ 1);
            break;
          case 2:
            // open all doors        
              Serial.println(F("Open all doors"));
            for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
            {                 
              MoveServo(i,1);
            }
            break;
          case 3:
            // close all doors            
              Serial.println(F("Close all doors"));
            for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
            {                 
              MoveServo(i,0);
            }
            break;
          default:
            //
            break;
          }
      }   
   
    
void loop()
/* this is the loop the code continuously executes after everything is initialized. 
When there is LocoNet traffic the code is interrupted to process the message and if there is a command to a servo that respective Status array is updated. 
The changes to the Status array will cause this code loop to move the servo as commanded */
{
	// Drive each servo one at a time
	for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
	{
		if (sAddresses[i].active ) 
		{	
			pwm1.setPWM(i, 0, sAddresses[i].Position);
			if (sAddresses[i].Status)
			{
				sAddresses[i].Position++;
				if (sAddresses[i].Position >= sAddresses[i].ServoMax)
				{
          sAddresses[i].Position = sAddresses[i].ServoMax;
					sAddresses[i].active = false;
   
#ifdef DEBUG_PRINT
					Serial.print(F("Servo Open: "));
					Serial.print(i, DEC);
					Serial.print(',');
					Serial.print(sAddresses[i].Status, DEC);
					Serial.print(',');
					Serial.println(sAddresses[i].Position, DEC);			
#endif          
          EEPROM.put(4*i,(sAddresses[i].Status));  
//          EEPROM.put((4*i)+2,(sAddresses[i].Position));
      
          LN_STATUS lnStatus = LocoNet.reportSensor(sAddresses[i].address,1);
 
#ifdef DEBUG_PRINT         
          Serial.print(F("Tx: Sensor: "));
          Serial.print(sAddresses[i].address);
          Serial.print(F(" Status: "));
          Serial.println(LocoNet.getStatusStr(lnStatus));
#endif          
				}				
			}
			else
			{
				sAddresses[i].Position--;
				if (sAddresses[i].Position <= sAddresses[i].ServoMin)
				{
          sAddresses[i].Position = sAddresses[i].ServoMin;
					sAddresses[i].active = false;
  
#ifdef DEBUG_PRINT
					Serial.print(F("Servo Closed: "));
					Serial.print(i, DEC);
					Serial.print(',');
					Serial.print(sAddresses[i].Status, DEC);
					Serial.print(',');
					Serial.println(sAddresses[i].Position, DEC);
#endif          
          EEPROM.put(4*i,(sAddresses[i].Status));
//          EEPROM.put((4*i)+2,(sAddresses[i].Position));
          
          LN_STATUS lnStatus = LocoNet.reportSensor(sAddresses[i].address,0);
   
#ifdef DEBUG_PRINT       
          Serial.print(F("Tx: Sensor: "));
          Serial.print(sAddresses[i].address);
          Serial.print(F(" Status: "));
          Serial.println(LocoNet.getStatusStr(lnStatus));
#endif          
				}
			}
      
		}
	}
	 delay(20);  // increase the delay to slow the rate of the servo movement
  
// Check LocoNet for pending switch commands:
  LnPacket=LocoNet.receive();
  if(LnPacket){
    LocoNet.processSwitchSensorMessage(LnPacket);
  }
  
if (keyChange)
  {
    uint8_t index = keyPad.getKey();
//    char lastChar = '\0';
//    char ch = keyPad.getChar();
//    keyChange = false;
    if (index < 16)
    {
#ifdef DEBUG_PRINT      
      Serial.print("press: ");
      Serial.println(keymap[index]);
#endif      
      buff[bufferIndex++] = keymap[index];
      if (keymap[index] == '#' || keymap[index] == '*' || keymap[index] == 'A' || keymap[index] == 'B' || keymap[index] == 'C' || keymap[index] == 'D')
      {
        keyPadCommand();
      }      
      else
      {
        delay(10);
        keyChange = false;  
      }    
    }  
    else
    {
      keyChange = false;  
    }    
  }
}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Sensor messages
//void notifySensor( uint16_t Address, uint8_t State ) {
//  Serial.print("Sensor: ");
//  Serial.print(Address, DEC);
//  Serial.print(" - ");
//  Serial.println( State ? "Active" : "Inactive" );
//}
//
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch Output Report messages
//void notifySwitchOutputsReport( uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput) {
//  Serial.print("Switch Outputs Report: ");
//  Serial.print(Address, DEC);
//  Serial.print(": Closed - ");
//  Serial.print(ClosedOutput ? "On" : "Off");
//  Serial.print(": Thrown - ");
//  Serial.println(ThrownOutput ? "On" : "Off");
//}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch Sensor Report messages
//void notifySwitchReport( uint16_t Address, uint8_t State, uint8_t Sensor ) {
//  Serial.print("Switch Sensor Report: ");
//  Serial.print(Address, DEC);
//  Serial.print(':');
//  Serial.print(Sensor ? "Switch" : "Aux");
//  Serial.print(" - ");
//  Serial.println( State ? "Active" : "Inactive" );
//}
//
//// This call-back function is called from LocoNet.processSwitchSensorMessage
//// for all Switch State messages
//void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction ) {
//  Serial.print("Switch State: ");
//  Serial.print(Address, DEC);
//  Serial.print(':');
//  Serial.print(Direction ? "Closed" : "Thrown");
//  Serial.print(" - ");
//  Serial.println(Output ? "On" : "Off");
//}

/*
 Name:		Final_ETEC252_RC_Send_Nunchuck.ino
 Created:	3/9/2016 11:05:14 PM
 Author:	Toby
*/

/*
Program to gather the nunchuk data and place in a small packet to send it wirelessly to the
receiver.

/-------------\   /---------\   /------\
|Wii Nunchuck |-->| Arduino |-->| xBee |
\-------------/   \---------/   \------/

This code deals with data from an input source (in this case a Wii Nunchuck), and uses it to
send left and right motor values to a slave device (via a xBee radio).

Normalized values are from 100 to -100 , representing 100% to -100%.

Initial Code based on:
The nunchuk code was borrowed from http://www.windmeadow.com/node/42

This program is like Nunchuk version 2 except the acceleration values are not adjusted to the left 2 places
after the reading. This keeps the reading in a smaller range.

*/

// Using the Wire library because of the two wire communication to the nunchuk.

// Override the wire library i2c speed (from 100Khz to 400 KHz)
#define TWI_FREQ 400000L

#include <Wire.h>;

//------------------- Constants / Defines -------------------

// ---- Wii Constants ----

// Wii Joystick allowable value range.
const int WII_JOYSTICK_MIN = 28;
const int WII_JOYSTICK_MAX = 225;

const int WII_BUTTON_PRESSED = 1;

// ---- Program Constants ----
const int NORMALIZED_DEAD_ZONE = 10;

const int GET_DATA_OK = 1;
const int GET_DATA_FAIL = 0;

//Button pins and value storage
const byte DUMP_BUTTON = 4;
const byte BOE_BUTTON = 5;
const byte CONTROLLER_BUTTON = 3;

int valbutton1;
int valbutton2;
int valbutton3;
int UDvalue;
int LRvalue;
// ---- Delay and watch times, in milliSeconds ----
const unsigned long TIME_BETWEEN_GET_DATA = 50;

/*
Commands / data rates / etc

This section MUST match between send and
receive code!

** Section Start **
*/
const long SERIAL_DATA_SPEED_38400_BPS = 38400;

const int MOTOR_VALUE_STOP = 0;

const int SERIAL_COMMAND_SET_RIGHT_MOTOR = 255;
const int SERIAL_COMMAND_SET_LEFT_MOTOR = 254;
const int SERIAL_COMMAND_SEND_TO_CONTROLLER = 253;
const int SERIAL_COMMAND_SEND_TO_BOE = 252;
const int SERIAL_COMMAND_ARM_LE = 251;
const int SERIAL_COMMAND_ARM_RL = 249;
const int SERIAL_COMMAND_DUMP = 250;

const int DEAD_ZONE = 15;
// Normalized value range.
const int NORMALIZED_RANGE_MIN = -100;
const int NORMALIZED_RANGE_MAX = 100;

// ** Section end **

//------------------- Global Variables -------------------

// Timer to control when we get new data from the Wii.
unsigned long lastGetDataTimer;

/*
Returns normalized data from the input device.

pNormalizedX = pointer to the int where we will put the returned X value.
pNormalizedY = pointer to the int where we will put the returned Y value.

return = 0 if there was an error, non-zero if there was no error. Note that if there was an
error, the pNormalizedX and pNormalizedY values cannot be trusted.
*/
unsigned char getNormalizedInput(int* pNormalizedX, int* pNormalizedY);
unsigned long previousTime;
//------------------- Functions -------------------

/*------
Function: setup
Description: Arduino setup hook, called once before the calls to loop() start.
*/
void setup()
{
	Serial.begin(SERIAL_DATA_SPEED_38400_BPS);
  pinMode(BOE_BUTTON,INPUT_PULLUP);
  pinMode(DUMP_BUTTON,INPUT_PULLUP);
	// Setup the Wii nunchuk power, and start communicating.
	nunchuk_init();

	lastGetDataTimer = millis();
}

/*------
Function: loop
Description: Arduino main loop, called over and over again... forever...
*/
void loop()
{
  if (millis() - previousTime >= TIME_BETWEEN_GET_DATA)
  {
      if (nunchuk_get_data())
      {
    	readJoysticks();
        readButtons();        
    	}
     else
     {
       // if we didn't get any data, zero the x and y values in case someone uses them without checking.
       UDvalue = 0;
       LRvalue = 0;
     }
 }
 
}
	

//######################\ Read Push Buttons /################################//
//================  =====================//
void readButtons()
{
  if (! digitalRead(CONTROLLER_BUTTON)) {              
    Serial.write(SERIAL_COMMAND_SEND_TO_CONTROLLER);
    }
    
  if (!digitalRead(BOE_BUTTON)) {              
    Serial.write(SERIAL_COMMAND_SEND_TO_BOE);
  }

  if (!digitalRead(DUMP_BUTTON)) {              
    Serial.write(SERIAL_COMMAND_DUMP);
  }

  if ( get_nunchuk_zbutton()) {              
    Serial.write(SERIAL_COMMAND_ARM_RL);
  }
  
   if (get_nunchuk_cbutton()) {              
     Serial.write(SERIAL_COMMAND_ARM_LE);
   }
}

//***********************************************************************  
void readJoysticks()
{
  static int joystick_x_offset = 120;
  static int joystick_y_offset = 120;
  LRvalue = constrain((nunchuk_joyx() - joystick_x_offset),NORMALIZED_RANGE_MIN,NORMALIZED_RANGE_MAX);
  UDvalue = constrain((nunchuk_joyy() - joystick_y_offset),NORMALIZED_RANGE_MIN,NORMALIZED_RANGE_MAX);
  //Serial.print ("LRvalue = "); Serial.println(LRvalue);
  DeadZone();
  }
//######################\ Dead Zone /################################//
void DeadZone()
{
  if ((UDvalue >= -DEAD_ZONE && UDvalue <= DEAD_ZONE) && (LRvalue <=  DEAD_ZONE && LRvalue >= -DEAD_ZONE)) // The deadspot
  {
    // Stop motors if in the dead zone.
    SendNewMotorValues(MOTOR_VALUE_STOP, MOTOR_VALUE_STOP);
    previousTime = millis();
  }
  else                                    // If the joystick is not in the deadspot range then:
  { // use the Y direction to set the forward and backward motion.
    moveRobot(LRvalue, UDvalue);
    previousTime = millis();
  }
}
//*************** MoveRobot ***********************************************//
void moveRobot (int valueX, int valueY)//subroutine to send pulses to the correct servo.
{
  //Use the X direction to set the forward and backward motion
  int leftMotorVal = valueY;
  int rightMotorVal = valueY;
  byte twist = 4; // Twist percentage

  // =============\ Creates a plus shape /============//
  // Gives control to just the X axis if Y Axis is in the dead zone
  if (valueY >= (-DEAD_ZONE + 2) && valueY <= (DEAD_ZONE - 2))
  {
    leftMotorVal = 0 + (valueX * 7.5) / 10; // Reduces twist effect to 75%
    rightMotorVal = 0 - (valueX * 7.5) / 10; //  of whole speed.
    //Serial.println("X only");
  }

  // Gives control to just the Y axis if X Axis is in the dead zone
  else if (valueX >= (-DEAD_ZONE + 2) && valueX <= (DEAD_ZONE - 2))
  {
    leftMotorVal = valueY;
    rightMotorVal = valueY;
    //Serial.println("Y only");
  }

  //Apply the X as a "twist" effect. (about 30% or what ever twist is set to of the calculated value)
  else if (valueY >= 0)
  {
    leftMotorVal = leftMotorVal + (valueX * twist) / 10;
    rightMotorVal = rightMotorVal - (valueX * twist) / 10;
  }
  else
  {
    leftMotorVal = leftMotorVal - (valueX * twist) / 10;
    rightMotorVal = rightMotorVal + (valueX * twist) / 10;
  }

  // Normalizes motorvals
  leftMotorVal = constrain(leftMotorVal, -100, 100);
  rightMotorVal = constrain(rightMotorVal, -100, 100);

  SendNewMotorValues(leftMotorVal, rightMotorVal);
}
//*************** Send Motor Values ***********************************************//
void SendNewMotorValues(char left, char right)
{

  Serial.write (SERIAL_COMMAND_SET_LEFT_MOTOR);
  Serial.write (left);

  Serial.write (SERIAL_COMMAND_SET_RIGHT_MOTOR);
  Serial.write (right);
  
//    Serial.print("left ");  Serial.print(left, DEC);
//    Serial.print("\t");
//    Serial.print("right ");  Serial.println(right, DEC);
//  
}

//------------------------------------------------------------------------------------------
/*
Nunchuck functions

The static keyword is used to create variables that are visible to only one function.
However unlike local variables that get created and destroyed every time a function is called,
static variables persist beyond the function call, preserving their data between function calls.

From : http://wiibrew.org/wiki/Wiimote/Extension_Controllers
Bit
Byte    |7      6       5       4       3       2       1       0
0       |SX<7:0>
1       |SY<7:0>
2       |AX<9:2>
3       |AY<9:2>
4       |AZ<9:2>
5       |AZ<1:0>        AY<1:0>         AX<1:0>         BC      BZ

SX,SY are the Analog Stick X and Y positions, while AX, AY, and AZ are the 10-bit accelerometer data
*/
const unsigned char WII_NUNCHUCK_nX_MASK = 0x03;
const unsigned char WII_NUNCHUCK_nX_NUM_BITS = 2;
const unsigned char WII_NUNCHUCK_AX_SHIFT = 2;
const unsigned char WII_NUNCHUCK_AY_SHIFT = 4;
const unsigned char WII_NUNCHUCK_AZ_SHIFT = 6;

//------------------- Constants / Defines -------------------

// 3 + 14 == Analog Pin 3, Analog Pin 3 is the same as pin 17 on the arduino
const unsigned char PWRPIN = 17;

// 2 + 14 == Analog Pin 2 , Analog Pin 2 is the same as pin 16 on the arduino
const unsigned char GNDPIN = 16;

const unsigned char WII_NUNCHUK_I2C_ADDRESS = 0x52;

const unsigned char WII_NUMBER_OF_BYTES_TO_READ = 6;

//------------------- Global Variables -------------------

// array to store nunchuck data
byte nunchuk_buf[WII_NUMBER_OF_BYTES_TO_READ];

//------------------- Functions -------------------

/*------
Function: nunchuk_setpowerpins
Description: Subroutine to set the Analog pins on the board to power and ground, clock and data.
*/
 void nunchuk_setpowerpins()
{
	pinMode(PWRPIN, OUTPUT);
	pinMode(GNDPIN, OUTPUT);

	/*
	Analog pin 3 is set high to power the nunchuk
	Analog pin 2 is set to ground to provide ground to the nunchuk
	*/
	digitalWrite(PWRPIN, HIGH);
	digitalWrite(GNDPIN, LOW);

	// 100 ms delay to allow settling of the lines.
	delay(100);
}

/*------
Function: nunchuk_init
Description: initialize the I2C system, join the I2C bus, and tell the nunchuck we're talking to it
*/
void nunchuk_init()
{
	// Make sure the hardware is properly configured.
	nunchuk_setpowerpins();

	// join i2c bus as master
	Wire.begin();

	// transmit to device
	Wire.beginTransmission(WII_NUNCHUK_I2C_ADDRESS);

	// sends value of 0x00 to memory address 0x40
	Wire.write(0x40);
	Wire.write(0);

	// stop transmitting via i2c stop.
	Wire.endTransmission();
}

/*------
Function: nunchuk_send_request
Description: Send a request for data to the nunchuck was "send_zero()"
*/
void nunchuk_send_request()
{
	// transmit to device
	Wire.beginTransmission(WII_NUNCHUK_I2C_ADDRESS);

	// sending 0x00 sets the pointer back to the lowest address in order to read multiple bytes
	Wire.write(0);

	// stop transmitting via i2c stop.
	Wire.endTransmission();
}

/*------
Function: nunchuk_get_data
Description: Receive data back from the nunchuck, returns 1 on successful read. returns 0 on failure.
Subroutine to read the data sent back from the nunchuk. It comes back in in 6 byte chunks.
*/
int nunchuk_get_data()
{
	int bytesReadBackCount = 0;

	// request data from nunchuck
	Wire.requestFrom(
		WII_NUNCHUK_I2C_ADDRESS,
		WII_NUMBER_OF_BYTES_TO_READ
		);

	// If there is data in the buffer then send to the arduino. 
	while (Wire.available())
	{
		/*
		receive byte as an integer.

		Wire.receive() - Retrieve a byte that was transmitted from a slave device to
		a master after a call to requestFrom or was transmitted from a master to a slave
		*/
		nunchuk_buf[bytesReadBackCount] = nunchuk_decode_byte(Wire.read());
		bytesReadBackCount++;
	}

	// send request for next data payload.
	nunchuk_send_request();

	// If we got enough bytes, let the caller know.
	return (bytesReadBackCount >= (WII_NUMBER_OF_BYTES_TO_READ - 1)) ? GET_DATA_OK : GET_DATA_FAIL;
}

/*------
Function: nunchuk_decode_byte
Description: helper function to decode the data coming from the nunchuk.
*/
char nunchuk_decode_byte(char x)
{
	x = (x ^ 0x17) + 0x17;
	return x;
}

/*------
Function:  process_nunchuck_accel
Description: helper function for processing accelromtere data for the wii nunchuck.
*/
int process_nunchuck_accel(unsigned char data_byte, unsigned char shift_size)
{
	int correctValue = nunchuk_buf[data_byte];
	correctValue <<= WII_NUNCHUCK_nX_NUM_BITS;
	correctValue |= ((nunchuk_buf[5] >> shift_size) & WII_NUNCHUCK_nX_MASK);
	return correctValue;
}

/*------
Function: get_nunchuk_zbutton
Description: returns zbutton state: 1=pressed, 0=notpressed
*/
int get_nunchuk_zbutton()
{
	// Shift byte by 0, AND with BIT0. If the bit is set, return 0, else 1.
	return (nunchuk_buf[5] & 0x01) ? 0 : 1;
}

/*------
Function: nunchuk_cbutton
Description: returns zbutton state: 1=pressed, 0=notpressed
*/
int get_nunchuk_cbutton()
{
	// Shift byte by 1, AND with BIT0. If the bit is set, return 0, else 1.
	return (nunchuk_buf[5] & 0x02) ? 0 : 1;
}

/*------
Function: nunchuk_joyx
Description: returns value of x-axis joystick
*/
int nunchuk_joyx() { return nunchuk_buf[0]; }

/*------
Function: nunchuk_joyy
Description: returns value of y-axis joystick
*/
int nunchuk_joyy() { return nunchuk_buf[1]; }

/*------
Function: nunchuk_accelx
Description: returns value of x-axis accelerometer
*/
int nunchuk_accelx() { return process_nunchuck_accel(2, WII_NUNCHUCK_AX_SHIFT); }

/*------
Function:  nunchuk_accely
Description: returns value of y-axis accelerometer
*/
int nunchuk_accely() { return process_nunchuck_accel(3, WII_NUNCHUCK_AY_SHIFT); }

/*------
Function:  nunchuk_accelz
Description: returns value of z-axis accelerometer
*/
int nunchuk_accelz() { return process_nunchuck_accel(4, WII_NUNCHUCK_AZ_SHIFT); }





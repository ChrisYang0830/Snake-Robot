

// Example code modified by Alexandr Bajenov
// to test sinusoidal servo motion
// Can receive float values now, but a 256-float array is too big.
// Using a 10-float array to store general parameters regarding motion model.
// Motion model is not complete yet, but can be written via serial link.

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  NOTE: Arduino Micro is based on the Leonardo; SCL -> 3, SDA -> 2
  
  Arduino Pro Mini has its I2C line on A5 and A4, which are outside
  of the breadboard header.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 6);// RX, TX

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// IMU Reset Pin, active-low:
#define IMU_RESETN 5 // Arduino pin connected to IMU reset pin
#define IMU_RESET_CODE 3 // Code used to verify IMU reset command
#define IMU_RESET_DURATION 0.1 // Duration of IMU reset pulse, in seconds


// Minimum that does not stall the servo seems to be 173(dec)
#define SERVOMIN  173 // this is the 'minimum' pulse length count (out of 4096)
// Maximum meaningful value seems to be 542(dec)
#define SERVOMAX  542 // this is the 'maximum' pulse length count (out of 4096)

#define SERVOMID 357
#define SERVORANGE 185
// Digital value of 31 corresponds to 20 degrees with current gearwork
#define SERVOLIFT 120

// In radians per second:
#define ANGLE_INCREMENT 0.04

#define PI_TIMES_1_OVER_135 2.3259
#define PI_TIMES_2_OVER_135 4.652
#define PI_TIMES_3_OVER_135 6.9778
#define PI_TIMES_4_OVER_135 9.3037
#define PI_TIMES_5_OVER_135 11.6296
#define PI_TIMES_6_OVER_135 13.9556
#define PI_TIMES_7_OVER_135 16.2815
#define PI_TIMES_8_OVER_135 18.607
#define PI_TIMES_9_OVER_135 20.933

#define ParametricArraySize 4
// Including start, steering, speed, locomotion mode

// Define addresses:
#define aXSteering 0
#define aYSteering 1
#define aPropagation 2
#define aLocomotion_Mode 3

// Define default values:
#define DEFAULT_XSteering 0
#define DEFAULT_start 0
#define DEFAULT_speed 15
#define DEFAULT_locomotion_mode 0

//float angle_increment = ANGLE_INCREMENT;

// Times are in microseconds since start of program.
unsigned long time_previous = 0;
unsigned long time_current;

float angle = 0;

char packet_ASCII_hex[12]; // Used to store packets before moving them
byte decoded_ASCII_hex[12];
byte packet_bytes[6]; // Used to store packets before moving them

//unsigned long parametric_data[256]; // Actually mixed 32 bit words, not just unsigned long
float parametric_data[ParametricArraySize]; // Actually mixed 32 bit words, not just unsigned long

void defaultParametricValues()
{
  parametric_data[aXSteering]       = DEFAULT_XSteering;
  parametric_data[aYSteering]       = DEFAULT_start;
  parametric_data[aPropagation]     = DEFAULT_speed;
  parametric_data[aLocomotion_Mode] = DEFAULT_locomotion_mode;
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  defaultParametricValues();

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  int n_servo;
  for (n_servo = 0; n_servo < 16; n_servo++)
  {
    pwm.setPWM(n_servo, 0, SERVOMID); // Set all unused servos to middle position
    delay(100);
  }

}

float longToFloat(long in) // This is a raw bitwise conversion.
{
  union {long uLong;  float uFloat;} uData;
  uData.uLong = in;
  return uData.uFloat;
}

float floatToLong(float in) // This is a raw bitwise conversion.
{
  union {long uLong;  float uFloat;} uData;
  uData.uFloat = in;
  return uData.uLong;
}

byte decodeASCIIHex(byte letter) // Takes ASCII hexadecimal digit (8 bit) returns its value (4 bit)
{
  if (('0' <= letter) && (letter <= '9')) { return letter - '0'; }
  if (('A' <= letter) && (letter <= 'F')) { return letter - 'A' + 10; }
  if (('a' <= letter) && (letter <= 'f')) { return letter - 'a' + 10; }
  return 0;
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
//  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
//  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


void incrementPacketQueue()
{
  byte i;
  for (i=0; i<11; i++)
  {
    packet_ASCII_hex[i] = packet_ASCII_hex[i+1];
  }
  packet_ASCII_hex[i] = mySerial.read();
  serialDataHandler();
}


// Accepts serial data as: AADDDDDDDDCC
// Where:
// AA is address of parameter,
// DDDDDDDD is parameter itself, represented as 32-bit IEEE 754 floating-point number.
// CC is checksum of packet
// Floating-point number calculator may be found here: http://www.h-schmidt.net/FloatConverter/IEEE754.html
void serialDataHandler()
{
  mySerial.readBytes(packet_ASCII_hex,12);
  byte i;

  // Need to make sure packet is all ASCII hex
  for (i = 0; i < 12; i++)
  {
    if (!isHexadecimalDigit(packet_ASCII_hex[i]))
    {
      return; // If we find a non-hex byte, this function cannot continue.
    }
  }
   
    
  // Need to decode packet from ASCII hex to bytes.
  for (i = 0; i < 12; i++)
  {
    decoded_ASCII_hex[i] = decodeASCIIHex(packet_ASCII_hex[i]);
  }
  for (i = 0; i < 6; i++)
  {
    packet_bytes[i] = (decoded_ASCII_hex[i<<1] << 4) + decoded_ASCII_hex[(i<<1)+1];
  }

  
  
  // Need to make sure packet was not damaged.
  unsigned int write_address = packet_bytes[0];
  
  byte checksum =
    packet_bytes[0] + 
    packet_bytes[1] + 
    packet_bytes[2] + 
    packet_bytes[3] + 
    packet_bytes[4] + 
    packet_bytes[5];
  
  checksum = checksum % 256;

  if (checksum == 0) // Packet is good?
  {
    long new_long_value;
    float new_float_value;
    new_long_value = 
      (long(packet_bytes[1])<< 24) + 
      (long(packet_bytes[2])<< 16) +
      (long(packet_bytes[3])<< 8)  +
      (long(packet_bytes[4]));
    new_float_value = longToFloat(new_long_value);
    if (0 <= write_address)
    {
        parametric_data[write_address] = new_float_value;
    }
  }
}


void loop() 
{

  while (mySerial.available())
  {
    serialDataHandler();
  }
  switch ((int)parametric_data[3])
  {
      case 0: stop_Locomotion();            break;
      case 1: Serpentine_Locomotion();      break;
      case 2: Serpentine_Lift_Locomotion(); break;
      //case 3: SideWinding_Locomotion();     break;
      //case 4: Rolling_Locomotion();         break;
    }
  
  delay(1);
}

void stop_Locomotion()
{
  int n_servo;
  for (n_servo = 0; n_servo < 16; n_servo++)
  {
    pwm.setPWM(n_servo, 0, SERVOMID); // Set all unused servos to middle position
    delay(100);
  }
  }

void Serpentine_Locomotion()
{
  short i,j=0;
  for(i=0;i<16;i+=2)
  {
    pwm.setPWM(i, 0, SERVOMID + ((-1)^j)*SERVORANGE*sin(angle + PI_TIMES_1_OVER_135));
    j++;
    }
  angle += parametric_data[1]*ANGLE_INCREMENT;
  delay((int)parametric_data[2]);
  }

void Serpentine_Lift_Locomotion()
{
  short i,j=0;
  for(i=0;i<16;i+=2)
  {
      pwm.setPWM(0, 0, SERVOMID + ((-1)^j)*SERVORANGE*sin(angle + PI_TIMES_1_OVER_135));
      j++;
    }
  for(i=1;i<16;i+=2)
  {
    pwm.setPWM(1, 0, SERVOMID + SERVOLIFT*sin(angle + PI_TIMES_1_OVER_135));
    }
  angle += parametric_data[1]*ANGLE_INCREMENT;
  delay((int)parametric_data[2]);
  }

 /* 1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 10
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
   - 

 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN   4
#define CSN_PIN 10
#define yaw A0
#define pitch A1

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int yaw_pitch[4];  // 4 element array holding Joystick readings
int swit1 = 5;
int swit2 = 6;


void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(pipe);
  pinMode(swit1,INPUT);
  pinMode(swit2,INPUT);

}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  radio.write( yaw_pitch, sizeof(yaw_pitch) );
  yaw_pitch[0] = analogRead(yaw);
  yaw_pitch[1] = analogRead(pitch);
  yaw_pitch[2] = 000;
  yaw_pitch[3] = 000;
  if (digitalRead(swit1) == HIGH)
  {
    yaw_pitch[2] = 111;
  }
  else if (digitalRead(swit2) == HIGH)
  {
    yaw_pitch[3] = 123;
  }
}

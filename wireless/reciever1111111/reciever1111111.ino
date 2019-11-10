#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#define CE_PIN   4
#define CSN_PIN 10

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int yaw_pitch[4];  // 4 element array holding Joystick readings
int pitch, yaw;
int pos = 0;
int on=0;
int counter=0;
bool done = false ;

Servo myservo;
void setup()   /****** SETUP: RUNS ONCE ******/
{
  myservo.attach(9);
  Serial.begin(9600);
  // delay(1000);
  Serial.println("Nrf24L01 Receiver Starting");
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();;
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  if ( radio.available() )
  {
    // Read the data payload until we've received everything

    while (!done)
    {
      // Fetch the data payload
      done = radio.read( yaw_pitch, sizeof(yaw_pitch) );
      Serial.print("yaw_pitch[2] ");
      yaw = map(yaw_pitch[0], 0, 1023, 0, 180);
      Serial.print(yaw_pitch[2]);
      Serial.print(" yaw_pitch[3] ");
      pitch = map(yaw_pitch[1], 0, 1023, 0, 180);
      Serial.println(yaw_pitch[3]);

  }
        Serial.println("out of while not done");

  }
//  else
//  {
////    counter=counter+1;
////    if (counter >= 2000)
////    {
////      if (on == 1)
////      {
////        fall();
////      }
////    }
//  }
      Serial.print(" in void loop ");
      Serial.print(on);

      if (on ==0){
            if (yaw_pitch[2] == 111)
                  {
                rise();
                   on=1;
                       }
                 }
    else {
       Serial.print(" else loop ");
      if (yaw_pitch[2] == 000){
        fall();
        on=0;
      }
    }
          Serial.println("my speed ");
          Serial.println(myservo.read());

}




void rise()
{
  // inintially put pin 13 to GND
  // to switch off motor ,make it float or to VCC

  for (pos = 0; pos <= 180; pos += 10) // goes from 0 degrees to 180 degrees
  { // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15ms for the servo to reach the position


  }
  for (pos = 180; pos >= 0; pos -= 10) // goes from 180 degrees to 0 degrees
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(500);

  }

  for (pos = 0; pos <= 140; pos += 10) // goes from 0 degrees to 180 degrees
  { // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15ms for the servo to reach the position

  }
}

void fall ()
{
     

      for (pos = 150; pos >= 20; pos -= 10)  // goes from 180 degrees to 0 degrees
      {
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(500);

      }

    }


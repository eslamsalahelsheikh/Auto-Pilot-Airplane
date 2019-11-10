//this code gets ypr from sensor ; setpoint from controller _put zero temporarely ;  
//and has outputs: output_yaw,output_pitch that actuate three servos
//still missing thurust

// ================================================================
// ===               1.MPU initialization                ===
// ================================================================
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL



#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// ================================================================
// ===               2.PID initialization                ===
// ================================================================

#define epsilon 0.01
//double dt;             //100ms loop time  have to deal with that!!!
#define MAX  90                   //For Current Saturation
unsigned long lastTime;
#define MIN -90
#define Kp  0.06
#define Kd  0.00
#define Ki  0.00
#define Kp_roll 0.06
#define Kd_roll  0.00
#define Ki_roll  0.00
float setpoint_yaw = 0;                   //should be input from controller;
float setpoint_pitch = 0;                 //should be input from controller;
int output_yaw=0;
int output_pitch=0;
float error_yaw;
float error_pitch;
float derivative;
static float integral_yaw = 0;  // should i put it in setup?and cancel static?? 
static float integral_pitch = 0;  // should i put it in setup?and cancel static?? 
static float pre_error_yaw = 0;// should i put it in setup?and cancel static?? 
static float pre_error_pitch = 0;// should i put it in setup?and cancel static?? 
// ================================================================
// ===               3.SERVO initialization                ===
// ================================================================

#include <Servo.h> 
 
Servo yaw_servo_1;  // create servo object to control a servo 
Servo yaw_servo_2;  // create servo object to control a servo 
Servo pitch_servo;  // create servo object to control a servo 
Servo thr; 
 
int yaw_servos_position=90; // to initialize it at 90
int yaw_servos_position2=90;
int pitch_servos_position=90;   // to initialize it at 90

//----------------------------------------------------------------------------------------- 
//----------------------------------------------------------------------------------------- 
//----------------------------------------------------------------------------------------- 
//----------------------------------------------------------------------------------------- 
// ================================================================
// ===               3.5.wireless definitions                ===
// ================================================================
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN   9
#define CSN_PIN 10

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int yaw_pitch_motor[4];  // 2 element array holding Joystick readings
int pos = 0;
bool done = false ;

void setup() {
// ================================================================
// ===               4.sensor setup                ===
// ================================================================
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
// ================================================================
// ===               5.wireless setup                ===
// ================================================================


//  Serial.begin(9600);

  
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
//      delay(1000);
  Serial.println("Nrf24L01 Receiver Starting");
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();;
  
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(86);
    mpu.setYGyroOffset(7);
    mpu.setZGyroOffset(40);
    mpu.setZAccelOffset(-1541); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
// ================================================================
// ===               5.other setup                ===
// ================================================================

  // put your setup code here, to run once:
  //lazem tkon pin pwm
  yaw_servo_1.attach(3);  // attaches the servo on pin 9 to the servo object 
  yaw_servo_2.attach(6);  // attaches the servo on pin 0 to the servo object 
  pitch_servo.attach(9);  // attaches the servo on pin 0 to the servo object 
  thr.attach(5);

}
  //----------------------------------------------------------------------------------------- 
    //----------------------------------------------------------------------------------------- 
void loop() {
// ================================================================
// ===               5.5 wireless dterminent               ===
// ================================================================
            // ================================================================
// ===               6.sensor loop               ===
// ================================================================
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
           ypr[0]=ypr[0]*(180/M_PI);
           ypr[1]=ypr[1]*(180/M_PI);
           ypr[2]=ypr[2]*(180/M_PI);
            Serial.print("ypr\t");
            Serial.print(ypr[0] );
            Serial.print("\t");
            Serial.print(ypr[1] );
            Serial.print("\t");
            Serial.println(ypr[2] ); //nw

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

// ================================================================
// ===               7.others loop               ===
// ================================================================
 unsigned long now = millis();
 double dt = (double)(now-lastTime);
 
// if (now >= 10000 && now <= 10500) 
// {
//   yaw_servos_position=90;
// }

//error_pitch = pitch_servo.read()+ ypr[1];
error_yaw = setpoint_yaw - ypr[2];
integral_yaw = integral_yaw + error_yaw*dt;

error_pitch = setpoint_pitch - ypr[1];
integral_pitch = integral_pitch + error_pitch*dt;
 


output_yaw = PIDcal2(error_yaw,pre_error_yaw,integral_yaw,dt);
output_pitch = PIDcal(error_pitch,pre_error_pitch,integral_pitch,dt);
        
pre_error_yaw = error_yaw;    //Update error
pre_error_pitch = error_pitch;    //Update error


//yaw_servos_position = yaw_servos_position + output_yaw;     //to use the output as a drift , not to return to zero angle every time
//pitch_servos_position = pitch_servos_position + output_pitch; //to use the output as a drift , not to return to zero angle every time
move_it(output_yaw,output_pitch);

//    Serial.print("output yaw \t");
//            Serial.print(output_yaw);
//                 Serial.print("\t");
//                Serial.print("error_yaw \t");
//            Serial.print(error_yaw);
//            Serial.print("\t");
       //      Serial.print("dt \t");
         //   Serial.print(dt);
             Serial.print("\t");
                             Serial.print("output_yaw \t");
            Serial.print(output_yaw);
            Serial.print("\t");
            
       //      Serial.print("dt \t");
         //   Serial.print(dt);
             Serial.print("\t");
             
//                Serial.print("output pitch \t");
//            Serial.print(output_pitch);
//                 Serial.print("\t");
//                Serial.print("error pitch \t");
//            Serial.print(error_pitch);
//            Serial.print("\t");
          //   Serial.print("dt \t");
         //   Serial.print(dt);
            lastTime = now; 
}
  //----------------------------------------------------------------------------------------- 
    //----------------------------------------------------------------------------------------- 
// ================================================================
// ===               8.functions               ===
// ================================================================

int PIDcal(float error,float pre_error,float integral,double dt)
{

  int output;
  float derivative;
    //In case of error too small then stop integration
    if(abs(error) > epsilon)
    {
        integral = integral + error*dt;
    }
    derivative = (error - pre_error)/dt;
    output = Kp*error + Ki*integral + Kd*derivative;

    //Saturation Filter
    if(output > MAX)
    {
        output = MAX;
    }
    else if(output < MIN)
    {
        output = MIN;
    }
 return output;
}

int PIDcal2(float error,float pre_error,float integral,double dt)
{

  int output;
  float derivative;
    //In case of error too small then stop integration
    if(abs(error) > epsilon)
    {
        integral = integral + error*dt;
    }
    derivative = (error - pre_error)/dt;
    output = Kp_roll*error + Ki_roll*integral + Kd_roll*derivative;

    //Saturation Filter
    if(output > MAX)
    {
        output = MAX;
    }
    else if(output < MIN)
    {
        output = MIN;
    }
 return output;
}
  //----------------------------------------------------------------------------------------- 
  void move_it(float output_yaw,float output_pitch )    // moving yaw and pitch servos
{ 
  
  yaw_servos_position = yaw_servo_1.read()- 90 + output_yaw;
  yaw_servos_position2 = yaw_servo_2.read()- 90 - output_yaw;
pitch_servos_position = pitch_servo.read()- 90 - output_pitch;
// yaw_servos_position = 90+ yaw_servos_position;
//pitch_servos_position = 90+ pitch_servos_position;
     yaw_servo_1.write(yaw_servos_position + 90); 
     yaw_servo_2.write(yaw_servos_position2 + 90);  
     pitch_servo.write(pitch_servos_position + 90);
     Serial.print("yaw_servos_position \t");
            Serial.print(yaw_servo_1.read());
                 Serial.print("\t");
 Serial.print("yaw_servos_position2 \t");
            Serial.print(yaw_servo_2.read());
                 Serial.print("\t");
        //delay(15);    
 }    
 

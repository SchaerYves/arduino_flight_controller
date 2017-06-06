

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
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

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

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

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/******************** PID Values ******************************************************/
float Kpx=1, Kix=0.1, Kdx=0.1; 
float Kpy=Kpx, Kiy=Kix, Kdy=Kdx;
float Kpz=20, Kiz=0.01, Kdz=0.001;

float pid_x, pid_y, pid_z;
int16_t ax, ay, az,  gx, gy, gz, dx, dy, dz, err_ax, err_ay, err_az, err_gx, err_gy, err_gz, err_dx, err_dy, err_dz;

int16_t in_ax=0, in_ay=0, in_az=0;
int16_t in_gx=0, in_gy=0, in_gz=0;
int16_t in_dx=0, in_dy=0, in_dz=0;

const int R1=8, R2=9, R3=10, R4=11;       // Input 8, 9, 10, 11 --> receiver channel 1, 2, 3, 4
const int M1=3, M2=5, M3=6, M4=7;         // Output 3, 5, 6, 7 --> Esc 1, 2, 3, 4 OUTPUT 4 is broken


int receiver_input[4];
byte highByte, lowByte;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;


long t_initial, t;
long t_loop;
int16_t timer_1, timer_2, timer_3, timer_4;
int16_t pulse_1=1000, pulse_2=1000, pulse_3=1000, pulse_4=1000;
int8_t flag=0, flag_1=0, flag_2=0, flag_3=0, flag_4=0;




// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {


    Serial.println("HELOOOO");
    DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
 
    Wire.begin();
    TWBR = 12;                                                                //set I2C to 400kHz

    t= millis();

  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    mpu.initialize();

     PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);  // set PCINT3 (digital input 11)to trigger an interrupt on state change

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
/*
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
*/
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


  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.


    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    Serial.print("setup complete");
}





void GetPosition(){

    mpuIntStatus = mpu.getIntStatus();
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

            ypr[0]=ypr[0]*180/M_PI/20*500+1500;
            ypr[1]=ypr[1]*180/M_PI/20*500+1500;
            ypr[2]=ypr[2]*180/M_PI/20*500+1500;
            
           /* Serial.print("ypr\t");
            Serial.print(ypr[0]);
            Serial.print("\t");
            Serial.print(ypr[1]);
            Serial.print("\t");
            Serial.println(ypr[2] );*/
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

       mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //mpu.getAcceleration(&ax, &ay, &az);
    //mpu.getRotation(&gx, &gy, &gz);

  
}




/********************************* error ***************************/

void CalcError(){

  err_ax=receiver_input_channel_1-ypr[2];
  err_gx=0-gx;
  /*Serial.print(err_ax);
  
  Serial.print(" ");
  Serial.print(err_gx);
  
  Serial.print(" ");*/

  err_ay=in_ay-ay;
  err_gy=in_gy-gy;

  err_az=in_az-az;
  err_gz=in_gz-gz;
  
  

  
}

/********************************** PID ******************************************/
void PID() {
  pid_x=(Kpx*err_ax+Kix*err_gx)/100;
  pid_y=(Kpy*err_ay+Kix*err_gy)/100;
 // Serial.println(pid_x);
  
}


void SetPulse() {

  pulse_1=1700+pid_x;
  
}
/*********************************** MOVE MOTOR ************************************/
void MoveMotor(){

    pulse_1=(receiver_input_channel_2/20*5)+1500;
    Serial.println(pulse_1);
    
  
   t_initial=micros();
   PORTD |= B11110000;  
   //PORTD |= B00001111;//Set digital port 3, 5, 6 and 7 high.
   
  
      
      
//      delayMicroseconds(pulse);                                                //Wait 1000us.
//      PORTD &= Bflag_40010111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    //delayMicroseconds(2000);
      while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    t = micros()-t_initial; 
    
    //Serial.println(t-pulse_1);//Read the current time.
    if(pulse_1 <= t)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(pulse_1 <= t)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(pulse_1 <= t)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(pulse_1 <= t)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
   
 

}




// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
//Serial.println("hallo");
t_loop=micros();
//GetPosition();

//CalcError();
//PID();
//SetPulse();
MoveMotor();
/*Serial.print(timer_1);
Serial.print(" ");
Serial.print(timer_2);
Serial.print(" ");
Serial.print(timer_3);
Serial.print(" ");
Serial.println(timer_4);
delay(500); */

  //delay(250);
 
 //print_signals();
 while(micros()-t_loop<4000){}
 //Serial.println(micros()-t_loop);
 
   
}



/********************** INTERRUPT **************************/

ISR(PCINT0_vect){
  //Channel 1=========================================
  if(last_channel_1 == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }
  //Channel 2=========================================
  if(last_channel_2 == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }
  //Channel 3=========================================
  if(last_channel_3 == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }
  //Channel 4=========================================
  if(last_channel_4 == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }
}

void print_signals(){
  Serial.print("Roll:");
  
  Serial.print((float)(receiver_input_channel_1-1500)/500*20);
  
  Serial.print("  Nick:");
  
  Serial.print((float)(receiver_input_channel_2-1500)/500*20);
  
  Serial.print("  Gas:");
  
  Serial.print((float)(receiver_input_channel_3-1000)/10);
  
  Serial.print("  Yaw:");
  
  Serial.println((float)(receiver_input_channel_4-1500)/500*20);
}

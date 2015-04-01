// Receiver
#include <MyPinChangeInt.h>

// Assign your channel in pins
#define CH1_IN_PIN 13
#define CH2_IN_PIN 12
#define CH3_IN_PIN 11
#define CH4_IN_PIN 10

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define CH1_FLAG 1
#define CH2_FLAG 2
#define CH3_FLAG 4
#define CH4_FLAG 8

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile float unCh1InShared;
volatile float unCh2InShared;
volatile float unCh3InShared;
volatile float unCh4InShared;

float unCh1In, unCh2In, unCh3In, unCh4In;
float unCh1InLast, unCh2InLast, unCh3InLast, unCh4InLast;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
float ulCh1Start;
float ulCh2Start;
float ulCh3Start;
float ulCh4Start;

// ESC
#define ESC1_OUT_PIN 7
#define ESC2_OUT_PIN 6
#define ESC3_OUT_PIN 5
#define ESC4_OUT_PIN 4

#define ESC_MIN 1049
#define ESC_MAX 1849
#define ESC_ARM 1154
#define ESC_TAKEOFF_OFFSET 1338

// MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
  /* =========================================================================
     NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
     depends on the MPU-6050's INT pin being connected to the Arduino's
     external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
     digital I/O pin 2.
   * ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprLast[3];

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_x_angle() {return last_x_angle;}
inline float get_last_y_angle() {return last_y_angle;}
inline float get_last_z_angle() {return last_z_angle;}
inline float get_last_gyro_x_angle() {return last_gyro_x_angle;}
inline float get_last_gyro_y_angle() {return last_gyro_y_angle;}
inline float get_last_gyro_z_angle() {return last_gyro_z_angle;}

//  Use the following global variables
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
unsigned long t_now;
float gyro_x, gyro_y, gyro_z;
float accel_x, accel_y, accel_z;
float accel_angle_x, accel_angle_y, accel_angle_z;
float dt;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z;
const float alpha = 0.96;
float angle_x, angle_y, angle_z;

// Buffer for data output
char dataOut[256];

  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() {
  int       num_readings = 10;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }

  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}

// Motors
#include <Servo.h>

Servo servoMotorA, servoMotorC; // Motors on the Y axis (pitch)
Servo servoMotorB, servoMotorD; // Motors on the X axis (roll)

float va, vb, vc, vd;                    //velocities

// PID Algorithm
#include <PID_v1.h>

#define PITCH_P_VAL 0.6f
#define PITCH_I_VAL 0.0f
#define PITCH_D_VAL 0.0f

#define ROLL_P_VAL 0.0f
#define ROLL_I_VAL 0.0f
#define ROLL_D_VAL 0.0f

#define YAW_P_VAL 0.0f
#define YAW_I_VAL 0.0f
#define YAW_D_VAL 0.0f

#define PITCH_MIN -100
#define PITCH_MAX 100
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180

#define PID_PITCH_INFLUENCE 100
#define PID_ROLL_INFLUENCE 100
#define PID_YAW_INFLUENCE 100

float bal_ac = 0, bal_bd = 0, bal_axes = 0;

//PID yawReg(&last_z_angle, &bal_axes, &unCh4In, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);
PID pitchReg(&last_y_angle, &bal_ac, &unCh2In, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, DIRECT);
PID rollReg(&last_x_angle, &bal_bd, &unCh1In, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, DIRECT);

// DEBUG
// #define DEBUG
//#define OUTPUT_RECEIVER_VALUES
//#define OUTPUT_READABLE_COMPLIMENTARY
//#define OUTPUT_BAL_AC_BD
//#define OUTPUT_MOTOR_VARIABLES

void setup()
{
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  // Pin configuration
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  digitalWrite(A0, HIGH);   // Cam PWR
  digitalWrite(A1, HIGH);   // SD PWR
  digitalWrite(A2, HIGH);   // RC PWR
  digitalWrite(A3, HIGH);   // MPU PWR
  digitalWrite(A4, LOW);    // RC GND
  digitalWrite(A5, LOW);    // ESC GND
  digitalWrite(A6, LOW);    // ESC GND
  digitalWrite(A7, LOW);    // ESC GND

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers
  servoMotorA.attach(ESC1_OUT_PIN, ESC_MIN, ESC_MAX);
  servoMotorB.attach(ESC2_OUT_PIN, ESC_MIN, ESC_MAX);
  servoMotorC.attach(ESC3_OUT_PIN, ESC_MIN, ESC_MAX);
  servoMotorD.attach(ESC4_OUT_PIN, ESC_MIN, ESC_MAX);

  delay(100);

  // arm ESCs
  servoMotorA.write(ESC_ARM);
  servoMotorB.write(ESC_ARM);
  servoMotorC.write(ESC_ARM);
  servoMotorD.write(ESC_ARM);

  delay(5000);

  initMPU();
  // get calibration values for sensors
  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
  initRegulators();

  // using the MyPinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(CH1_IN_PIN, calcCh1, CHANGE);
  PCintPort::attachInterrupt(CH2_IN_PIN, calcCh2, CHANGE);
  PCintPort::attachInterrupt(CH3_IN_PIN, calcCh3, CHANGE);
  PCintPort::attachInterrupt(CH4_IN_PIN, calcCh4, CHANGE);
}

void initMPU() {

  Wire.begin();
  TWBR = 24; // set 400kHz mode @ 16MHz CPU or 200kHz mode @ 8MHz CPU

  // initialize device
  #ifdef DEBUG
    Serial.println(F("Initializing I2C devices..."));
  #endif
  mpu.initialize();

  #ifdef DEBUG
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  #endif

  // load and configure the DMP
  #ifdef DEBUG
    Serial.println(F("Initializing DMP..."));
  #endif
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //386	86	1864	-31	-63	-7
  mpu.setXGyroOffset(-31);
  mpu.setYGyroOffset(-63);
  mpu.setZGyroOffset(-7);
  mpu.setXAccelOffset(386);
  mpu.setYAccelOffset(86);
  mpu.setZAccelOffset(1864);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    #ifdef DEBUG
      Serial.println(F("Enabling DMP..."));
    #endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    #ifdef DEBUG
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    #endif
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    #ifdef DEBUG
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
    #endif
    dmpReady = true;

    // Set the full scale range of the gyro
    uint8_t FS_SEL = 0;
    //mpu.setFullScaleGyroRange(FS_SEL);

    // get default full scale value of gyro - may have changed from default
    // function call returns values between 0 and 3
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
    Serial.print("FS_SEL = ");
    Serial.println(READ_FS_SEL);
    GYRO_FACTOR = 131.0/(FS_SEL + 1);

    // get default full scale value of accelerometer - may not be default value.
    // Accelerometer scale factor doesn't reall matter as it divides out
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
    Serial.print("AFS_SEL = ");
    Serial.println(READ_AFS_SEL);
    //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);

    // Set the full scale range of the accelerometer
    //uint8_t AFS_SEL = 0;
    //mpu.setFullScaleAccelRange(AFS_SEL);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    #ifdef DEBUG
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    #endif
  }

  #ifdef DEBUG
    Serial.println("-----------------");
  #endif
}

void initRegulators(){
  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  //yawReg.SetMode(AUTOMATIC);
  //yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);
}

void loop()
{
  // local copy of variables on first function call to hold local copies
  static uint8_t bUpdateFlags;

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared) {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & CH1_FLAG)
    {
      unCh1In = unCh1InShared; // roll
    }
    if(bUpdateFlags & CH2_FLAG)
    {
      //unCh2In = unCh2InShared; // pitch
      unCh2In = map(unCh2InShared, ESC_MIN, ESC_MAX, PITCH_MIN, PITCH_MAX);
    }
    if(bUpdateFlags & CH3_FLAG)
    {
      unCh3In = unCh3InShared; // velocity
    }
    // if(bUpdateFlags & CH4_FLAG)
    // {
      // unCh4In = unCh4InShared; // yaw
    // }

    // clear shared copy of updated flags as we have already taken the updates
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with
  }

  // do any processing from here onwards
  // only use the local values unCh1In, unCh2In, unCh3In, unCh4In
  // the shared variables unCh1InShared, unCh2InShared, unCh3InShared, unCh4InShared are
  // always owned by the interrupt routines and should not be used in loop

  if((unCh1In < ESC_MIN) || (unCh1In > ESC_MAX)) unCh1In = unCh1InLast;
  if((unCh2In < ESC_MIN) || (unCh2In > ESC_MAX)) unCh2In = unCh2InLast;
  if((unCh3In < ESC_MIN) || (unCh3In > ESC_MAX)) unCh3In = unCh3InLast;
  // if((unCh4In < ESC_MIN) || (unCh4In > ESC_MAX)) unCh4In = unCh4InLast;

  unCh1InLast = unCh1In;
  unCh2InLast = unCh2In;
  unCh3InLast = unCh3In;
  // unCh4InLast = unCh4In;

  #ifdef OUTPUT_RECEIVER_VALUES
    Serial.print("ch1, ch2, ch3, ch4");
    Serial.print("\t\t");
    Serial.print(unCh1In);
    Serial.print("\t\t");
    Serial.print(unCh2In);
    Serial.print("\t\t");
    Serial.println(unCh3In);
    // Serial.print("\t\t");
    // Serial.println(unCh4In);
  #endif

  // Keep calculating the values of the complementary filter angles for comparison with DMP here
  // Read the raw accel/gyro values from the MPU-6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Get time of last raw data read
  t_now = millis();

  // Remove offsets and scale gyro data
  gyro_x = (gx - base_x_gyro)/GYRO_FACTOR;
  gyro_y = (gy - base_y_gyro)/GYRO_FACTOR;
  // gyro_z = (gz - base_z_gyro)/GYRO_FACTOR;
  accel_x = ax; // - base_x_accel;
  accel_y = ay; // - base_y_accel;
  // accel_z = az; // - base_z_accel;

  accel_angle_x = atan(-accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  accel_angle_y = atan(accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  // accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  dt = (t_now - get_last_time())/1000.0;
  gyro_angle_x = gyro_x*dt + get_last_x_angle();
  gyro_angle_y = gyro_y*dt + get_last_y_angle();
  // gyro_angle_z = gyro_z*dt + get_last_z_angle();

  // Compute the drifting gyro angles
  unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  // unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  // const float alpha = 0.96;
  angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  // angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

  #ifdef OUTPUT_READABLE_COMPLIMENTARY
    Serial.print("CMP:\t");
    Serial.print(get_last_z_angle(), 2);
    Serial.print("\t:\t");
    Serial.print(get_last_y_angle(), 2);
    Serial.print("\t:\t");
    Serial.println(get_last_x_angle(), 2);
  #endif

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    #ifdef DEBUG
      Serial.println(F("MPU FIFO overflow!"));
    #endif
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
  }

  rollReg.Compute();
  pitchReg.Compute();
  // yawReg.Compute();

  #ifdef OUTPUT_BAL_AC_BD
    Serial.print("ac, bd");
    Serial.print("\t\t");
    Serial.print(bal_ac);
    Serial.print("\t\t");
    Serial.println(bal_bd);
  #endif

  va = abs(((-100+bal_ac)/100)*unCh3In);
  vb = abs(((-100+bal_bd)/100)*unCh3In);
  vc = ((100+bal_ac)/100)*unCh3In;
  vd = ((100+bal_bd)/100)*unCh3In;

  if((va < ESC_MIN) || (va > ESC_MAX)) va = ESC_MIN;
  if((vb < ESC_MIN) || (vb > ESC_MAX)) vb = ESC_MIN;
  if((vc < ESC_MIN) || (vc > ESC_MAX)) vc = ESC_MIN;
  // if((vd < ESC_MIN) || (vd > ESC_MAX)) vd = ESC_MIN;

  if(unCh3In < ESC_TAKEOFF_OFFSET){
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  }

  //servoMotorA.write(va);
  //servoMotorB.write(vb);
  //servoMotorC.write(vc);
  //servoMotorD.write(vd);

  #ifdef OUTPUT_MOTOR_VARIABLES
    Serial.print("v, va, vb, vc, vd");
    Serial.print("\t");
    Serial.print(unCh3In);
    Serial.print("\t");
    Serial.print(va);
    Serial.print("\t");
    Serial.print(vb);
    Serial.print("\t");
    Serial.print(vc);
    Serial.print("\t");
    Serial.println(vd);
  #endif

  bUpdateFlags = 0;
}

// simple interrupt service routine
void calcCh1(){
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(CH1_IN_PIN) == HIGH)
  {
    ulCh1Start = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unCh1InShared = (uint16_t)(micros() - ulCh1Start);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= CH1_FLAG;
  }
}
void calcCh2(){
  if(digitalRead(CH2_IN_PIN) == HIGH)
  {
    ulCh2Start = micros();
  }
  else
  {
    unCh2InShared = (uint16_t)(micros() - ulCh2Start);
    bUpdateFlagsShared |= CH2_FLAG;
  }
}
void calcCh3(){
  if(digitalRead(CH3_IN_PIN) == HIGH)
  {
    ulCh3Start = micros();
  }
  else
  {
    unCh3InShared = (uint16_t)(micros() - ulCh3Start);
    bUpdateFlagsShared |= CH3_FLAG;
  }
}
void calcCh4(){
  if(digitalRead(CH4_IN_PIN) == HIGH)
  {
    ulCh4Start = micros();
  }
  else
  {
    unCh4InShared = (uint16_t)(micros() - ulCh4Start);
    bUpdateFlagsShared |= CH4_FLAG;
  }
}

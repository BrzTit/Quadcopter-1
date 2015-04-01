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

#define ESC_MIN 1000
#define ESC_MAX 1900
#define ESC_ARM 1150

// Motors
#include <Servo.h>

Servo servoMotorA, servoMotorC; // Motors on the Y axis (pitch)
Servo servoMotorB, servoMotorD; // Motors on the X axis (roll)

void setup() {
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

  Serial.begin(115200);

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

  // using the MyPinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(CH1_IN_PIN, calcCh1, CHANGE);
  PCintPort::attachInterrupt(CH2_IN_PIN, calcCh2, CHANGE);
  PCintPort::attachInterrupt(CH3_IN_PIN, calcCh3, CHANGE);
  PCintPort::attachInterrupt(CH4_IN_PIN, calcCh4, CHANGE);
}

void loop() {
  // local copy of variables on first function call to hold local copies
  static uint8_t bUpdateFlags;

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
      unCh2In = unCh2InShared; // pitch
    }
    if(bUpdateFlags & CH3_FLAG)
    {
      unCh3In = unCh3InShared; // velocity
    }
    if(bUpdateFlags & CH4_FLAG)
    {
      unCh4In = unCh4InShared; // yaw
    }

    // clear shared copy of updated flags as we have already taken the updates
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with
  }

  servoMotorA.write(unCh3In);
  servoMotorB.write(unCh3In);
  servoMotorC.write(unCh3In);
  servoMotorD.write(unCh3In);

  Serial.print("ch3");
  Serial.print("\t:\t");
  Serial.println(unCh3In);

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

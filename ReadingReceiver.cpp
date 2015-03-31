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
volatile uint16_t unCh1InShared;
volatile uint16_t unCh2InShared;
volatile uint16_t unCh3InShared;
volatile uint16_t unCh4InShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulCh1Start;
uint32_t ulCh2Start;
uint32_t ulCh3Start;
uint32_t ulCh4Start;

void setup()
{
  Serial.begin(115200);

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

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(CH1_IN_PIN, calcCh1, CHANGE);
  PCintPort::attachInterrupt(CH2_IN_PIN, calcCh2, CHANGE);
  PCintPort::attachInterrupt(CH3_IN_PIN, calcCh3, CHANGE);
  PCintPort::attachInterrupt(CH4_IN_PIN, calcCh4, CHANGE);
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unCh1In;
  static uint16_t unCh2In;
  static uint16_t unCh3In;
  static uint16_t unCh4In;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & CH1_FLAG)
    {
      unCh1In = unCh1InShared;
    }
    if(bUpdateFlags & CH2_FLAG)
    {
      unCh2In = unCh2InShared;
    }
    if(bUpdateFlags & CH3_FLAG)
    {
      unCh3In = unCh3InShared;
    }
    if(bUpdateFlags & CH4_FLAG)
    {
      unCh4In = unCh4InShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }

  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop

  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.

  // we are checking to see if the channel value has changed, this is indicated
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  Serial.print("ch1, ch2, ch3, ch4");
  Serial.print("\t\t");
  Serial.print(unCh1In);
  Serial.print("\t\t");
  Serial.print(unCh2In);
  Serial.print("\t\t");
  Serial.print(unCh3In);
  Serial.print("\t\t");
  Serial.println(unCh4In);

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

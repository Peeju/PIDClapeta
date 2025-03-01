#include <Arduino.h>
#include <PID/PID_v1.h>
#include "error/error.hpp"

#define faultyPin 13
#define LPWM 11
#define ELPWM 12
#define throttlePositionSensor1 A5
#define throttlePositionSensor2 A4 //!A5 is Just for testing, need to be 2 different inputs
#define pedalPositionSensor1 A2
#define pedalPositionSensor2 A2 //!Just for testing, need to be 2 different inputs
#define maxPWMValue 150
#define minPWMValue 60
#define Kp 0.35
#define Ki 2.13 
#define Kd 0.00164870
#define upShiftTime 300
#define downShiftTime 300 //!Max 500 ms
#define downShiftBlipPower 180
#define SCPin 4
#define sensorToleranceLow 10
#define sensorToleranceHigh 10


/*
* Throttle position 1 311-725
* Throttle postion 2 729-231
* Pedal position 1 0-1024
* Pedal position 2 0-1024 

Done:
Plausability senzori clapeta
Plausability senzori pedala
Enable for SCS - no difference
Plausability individual senzori pedala
Plausability individual senzori clapeta
If the throttle position differs by more that 10% from excepted target for more that 500 ms

Software:
Todo: Check at start for limits - idle check
Todo: Individual error check - LED for each type of error
Todo: Manual mode - potentiometer on the case for easy dyno tuning - Automatic, Potentiometer control, Full Throttle
Todo: Debounce butoane shifter

Hardware:
Todo: Resistor pull-up/pull-down pentru pinii A
Todo: Potentiometre control shifter
*/
int tpsLowerBoundry1 = 311;
int tpsUpperBoundry1 = 789;

int tpsLowerBoundry2 = 729;
int tpsUpperBoundry2 = 231;

unsigned long plausabilityTime = 0;
unsigned long errorTime = 0;
volatile bool upshiftButtonPressed = false;
volatile bool downShiftButtonPressed = false;

enum gearShift {UPSHIFT, DOWNSHIFT, NOGEARSHIFT};
gearShift gearShiftState = NOGEARSHIFT;


errorCode error;

//0-NOERROR, 1-PEDAL1, 2-PEDAL2, 3-PEDALGENERAL, 4-THROTTLE1, 5-THROTTLE2, 6-THROTTLEGENGERAL
 



volatile long gearShiftTime = 0;

double input, output, setpoint;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


void upShift();
void downShift();

void setup() {
  noInterrupts();
  pinMode(SCPin, OUTPUT);
  
  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minPWMValue, maxPWMValue);
  pinMode(LPWM,OUTPUT);
  pinMode(faultyPin, OUTPUT);
  digitalWrite(faultyPin, LOW);
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  myPID.SetSampleTime(10);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), upShift, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), downShift, FALLING);
  
  interrupts();
}

void loop() {
  //Serial.println("***NewLOOP***");
  unsigned long time = millis();
  int throttlePosition1 = analogRead(throttlePositionSensor1);
  int throttlePosition2 = analogRead(throttlePositionSensor2);
    // Serial.print("Throttle position raw 1:");
    // Serial.println(throttlePosition1);
    // Serial.print("Throttle position raw 2:");
    // Serial.println(throttlePosition2);

  if (throttlePosition1 < tpsLowerBoundry1 - sensorToleranceLow && throttlePosition1 > tpsUpperBoundry1 + sensorToleranceHigh) 
    error.setError(errorCode::ERROR_THROTTLE1);
  else if (error.hasError(errorCode::ERROR_THROTTLE1) == true)
    error.clearError(errorCode::ERROR_THROTTLE1);
  
  throttlePosition1 = map(throttlePosition1, tpsLowerBoundry1, tpsUpperBoundry1, 0, 255);
  input = throttlePosition1;

  int pedalPosition1 = analogRead(pedalPositionSensor1);
  if (pedalPosition1 < 0 && pedalPosition1 > 1024) error.setError(errorCode::ERROR_PEDAL1);
  pedalPosition1 = map(constrain(pedalPosition1, 0, 1024), 0, 1024, 0, 255);
  
  setpoint = pedalPosition1;
  // Serial.print("Pedal position:");
  // Serial.println(setpoint);

  myPID.Compute();

  if (throttlePosition2 > tpsLowerBoundry2 + sensorToleranceHigh && throttlePosition2 < tpsUpperBoundry2 - sensorToleranceLow) error.setError(errorCode::ERROR_THROTTLE2);
  else if (error.hasError(errorCode::ERROR_THROTTLE2) == true)
    error.clearError(errorCode::ERROR_THROTTLE2);
  
  throttlePosition2 = map(throttlePosition2, tpsLowerBoundry2, tpsUpperBoundry2, 0, 255);
  // Serial.print("Throttle position 1:");
  //   Serial.println(throttlePosition1);
  //   Serial.print("Throttle position raw 2:");
  //   Serial.println(throttlePosition2);
  if (abs(throttlePosition1 - throttlePosition2) > 30) {
    error.setError(errorCode::ERROR_THROTTLEGENERAL);
    digitalWrite(faultyPin, HIGH);}
  else {
    error.clearError(errorCode::ERROR_THROTTLEGENERAL);
    digitalWrite(faultyPin, LOW);}
  
  


  double pedalPosition2 = analogRead(pedalPositionSensor2);
  if (pedalPosition2 < 0 && pedalPosition2 > 1024) error.setError(errorCode::ERROR_PEDAL2);
  else if (error.hasError(errorCode::ERROR_PEDAL2) == true)
    error.clearError(errorCode::ERROR_PEDAL2);
  pedalPosition2 = map(constrain(pedalPosition2, 0, 1024), 0, 1024, 0, 255);


  if (abs(pedalPosition1 - pedalPosition2) > 30) {
    error.setError(errorCode::ERROR_PEDALGENERAL); 
    digitalWrite(faultyPin, HIGH);}
  else {
    error.clearError(errorCode::ERROR_PEDALGENERAL);
    digitalWrite(faultyPin, LOW);}
  
  unsigned long currentMillis = millis();  


// if (abs(setpoint - input) > 25) {
//     if (plausabilityTime == 0) {
//         plausabilityTime = currentMillis;  
//     } else {
//         unsigned long timeElapsed = currentMillis - plausabilityTime;
//         Serial.println(timeElapsed);
//         if (timeElapsed >= 500) {
//             Serial.println("IN IF");
//             error.setError(errorCode::ERROR_TARGET);
//         }
//     }
// } else {  
//     plausabilityTime = 0;  
//     error.clearError(errorCode::ERROR_TARGET);
// }
  
  //  Serial.print("Output:");
  //  Serial.println(output);
   if(output < minPWMValue - 5) output=0;
   if(setpoint > 20 && error.hasAnyErrors() == false && gearShiftState == NOGEARSHIFT)
      {analogWrite(LPWM, output);
      }
   else if(gearShiftState == NOGEARSHIFT) {
   analogWrite(LPWM, 0);
   output = 0;
   //Serial.println("Main PWM");
   }
  
  // error.printError();
  // Serial.print("Output:");
  // Serial.println(output);
  // error.printError();
  //Serial.println(throttlePosition1);
  //Serial.println(throttlePosition2);
  switch(gearShiftState){
    case UPSHIFT:
      if(millis()-gearShiftTime > upShiftTime ) {gearShiftState = NOGEARSHIFT;
      upshiftButtonPressed = false;
      // //Serial.println(millis()-gearShiftTime);
      }
      break;

    case DOWNSHIFT:
      if(millis()-gearShiftTime > downShiftTime) {gearShiftState = NOGEARSHIFT;
        downShiftButtonPressed = false;
      }
      break;

        case NOGEARSHIFT:
      if(upshiftButtonPressed == true && error.hasAnyErrors() == 0) {
        gearShiftTime = millis();
        gearShiftState = UPSHIFT;
        analogWrite(LPWM, 0);
        // //Serial.println("UPSHIFT PRESSED");
      }
      if(downShiftButtonPressed == true && error.hasAnyErrors() == 0){
        gearShiftTime = millis();
        gearShiftState = DOWNSHIFT;
        analogWrite(LPWM, downShiftBlipPower);
      }
      break;
    

  }
  
  currentMillis = millis();

  if (error.hasAnyErrors()){
    

    if(error.hasError(errorCode::ERROR_PEDAL1)) digitalWrite(faultyPin, HIGH);
    if(error.hasError(errorCode::ERROR_PEDAL2)) digitalWrite(faultyPin, HIGH);
    if(error.hasError(errorCode::ERROR_PEDALGENERAL)) digitalWrite(faultyPin, HIGH);
    if(error.hasError(errorCode::ERROR_THROTTLE1)) digitalWrite(faultyPin, HIGH);
    if(error.hasError(errorCode::ERROR_THROTTLE2)) digitalWrite(faultyPin, HIGH);
    if(error.hasError(errorCode::ERROR_THROTTLEGENERAL)) digitalWrite(faultyPin, HIGH);
    if(error.hasError(errorCode::ERROR_TARGET)) digitalWrite(faultyPin, HIGH);

    
    if(errorTime!=0) errorTime = currentMillis;
    else if (currentMillis-errorTime>=1000 && throttlePosition1>=10) {digitalWrite(SCPin, HIGH);}  
    
  }
  else { 
    digitalWrite(faultyPin, LOW);
    // if(throttlePosition1<=10) {
    //   if(throttlePosition1<=10)
  }
  currentMillis = millis();
  Serial.println(currentMillis-time);
  // Serial.println(microsecondsToClockCycles(micros()-time));
  
}

  void upShift(){
    upshiftButtonPressed = true;
  }

  void downShift(){
    downShiftButtonPressed = true;
  }
  


#include <Arduino.h>
#include <PID/PID_v1.h>

#define faultyPin 13
#define LPWM 11
#define ELPWM 12
#define throttlePositionSensor1 A3
#define throttlePositionSensor2 A4
#define pedalPositionSensor1 A2
#define pedalPositionSensor2 A2 //!Just for testing, need to be 2 different inputs
#define maxPWMValue 130
#define minPWMValue 60
#define Kp 0.25
#define Ki 4.77 
#define Kd 0.00173150
#define upShiftTime 300
#define downShiftTime 300 //!Max 500 ms
#define downShiftBlipPower 180


/*
* Throttle position 1 312-789
* Throttle postion 2 731-239
* Pedal position 1 0-1024
* Pedal position 2 0-1024 

Done:
Plausability senzori clapeta
Plausability senzori pedala
Enable for SCS - no difference
Plausability individual senzori pedala
Plausability individual senzori clapeta

Todo: If the throttle position differs by more that 10% from excepted target for more that 500 ms
Todo: Check at start for limits - idle check
Todo: Individual error check

Todo: Resistor pull-up/pull-down pentru pinii A
TPS1 312-789
TPS2 731-312
*/
int tpsUpperBoundry1 = 312;
int tpsLowerBoundry1 = 789;
int tpsUpperBoundry2 = 789;
int tpsLowerBoundry2 = 312;

volatile bool upshiftButtonPressed = false;
volatile bool downShiftButtonPressed = false;

enum gearShift {UPSHIFT, DOWNSHIFT, NOGEARSHIFT};
gearShift gearShiftState = NOGEARSHIFT;


volatile long gearShiftTime = 0;

double input, output, setpoint;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int error = 0;

void upShift();
void downShift();

void setup() {
  noInterrupts();

  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, maxPWMValue);
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



  int throttlePosition1 = analogRead(throttlePositionSensor1);
  if (throttlePosition1 < 300 && throttlePosition1 > 800) error = 3;
  throttlePosition1 = map(constrain(throttlePosition1, 312, 789), 312, 789, 0, 255);
  Serial.print("Throttle position:");
  Serial.println(input);
  // Serial.println(throttlePosition1 - throttlePosition2);
  // Serial.print("ERROR:");
  // Serial.println(error);
  input = throttlePosition1;

  int pedalPosition1 = analogRead(pedalPositionSensor1);
  if (pedalPosition1 < 0 && pedalPosition1 > 1024) error = 4;
  pedalPosition1 = map(constrain(pedalPosition1, 0, 1024), 0, 1024, 0, 255);
  setpoint = pedalPosition1;
  Serial.print("Pedal position:");
  Serial.println(setpoint);

  myPID.Compute();

  double throttlePosition2 = analogRead(throttlePositionSensor2);
  if (throttlePosition2 < 300 && throttlePosition2 > 800) error = 1;
  throttlePosition2 = map(constrain(throttlePosition2, 230, 731), 731, 230, 0, 255);
  // Serial.print("Throttle position 2: ");
  // Serial.println(throttlePosition2);

  if (abs(throttlePosition1 - throttlePosition2) > 30) {
    error = 3; 
    digitalWrite(faultyPin, HIGH);}
  else {error = 0;digitalWrite(faultyPin, LOW);}
  Serial.print("ERROR:");
  Serial.println(error);
  


  double pedalPosition2 = analogRead(pedalPositionSensor2);
  if (pedalPosition2 < 0 && pedalPosition2 > 1024) error = 4;
  pedalPosition2 = map(constrain(pedalPosition2, 0, 1024), 0, 1024, 0, 255);
  Serial.print("Pedal position 2:");
  Serial.println(pedalPosition2);

  Serial.print("ERROR:");
  Serial.println(error);

  if (abs(pedalPosition1 - pedalPosition2) > 30) {
    Serial.print("abs(pedalPosition1 - pedalPosition2): ");
    Serial.println(abs(pedalPosition1 - pedalPosition2));
    error = 2; 
    digitalWrite(faultyPin, HIGH);}
  else {error = 0;
  digitalWrite(faultyPin, LOW);}
  // error = 0; //!Bug la pedal postion
  

  Serial.print("ERROR:");
  Serial.println(error);

  
   Serial.print("Output:");
   Serial.println(output);
   if(output < minPWMValue - 5) output=0;
   if(setpoint > 20 && error == 0 && gearShiftState == NOGEARSHIFT)
      analogWrite(LPWM, output);
   else if(gearShiftState == NOGEARSHIFT) analogWrite(LPWM, 0);
  Serial.print("GearShiftState: ");
  Serial.println(gearShiftState);
  
  switch(gearShiftState){
    case UPSHIFT:
      if(millis()-gearShiftTime > upShiftTime ) {gearShiftState = NOGEARSHIFT;
      upshiftButtonPressed = false;
      Serial.println(millis()-gearShiftTime);
      }
      break;

    case DOWNSHIFT:
      if(millis()-gearShiftTime > downShiftTime) {gearShiftState = NOGEARSHIFT;
        downShiftButtonPressed = false;
      }
      break;

        case NOGEARSHIFT:
      if(upshiftButtonPressed == true && error == 0) {
        gearShiftTime = millis();
        gearShiftState = UPSHIFT;
        analogWrite(LPWM, 0);
        Serial.println("UPSHIFT PRESSED");
      }
      if(downShiftButtonPressed == true && error == 0){
        gearShiftTime = millis();
        gearShiftState = DOWNSHIFT;
        analogWrite(LPWM, downShiftBlipPower);
      }
      break;
    

  }
  
}

  void upShift(){
    upshiftButtonPressed = true;
  }

  void downShift(){
    downShiftButtonPressed = true;
  }
  


#include <Arduino.h>
#include <PID/PID_v1.h>
#include "error/error.hpp"
#include <EEPROM.h>


#define LPWM 11
#define RPWM 10
#define throttlePositionSensor1 A4
#define throttlePositionSensor2 A5 //!A5 is Just for testing, need to be 2 different inputs
#define pedalPositionSensor1 A2
#define pedalPositionSensor2 A3 //!Just for testing, need to be 2 different inputs
#define maxPWMValue 200
#define minPWMValue 0
#define Kp 2.8 
#define Ki 0.3
#define Kd 0//.00009971632
#define downShiftBlipPower 180
#define sensorToleranceLow 15
#define sensorToleranceHigh 15
#define pedalLED 12
#define throttleLED 9
#define targetLED 8
#define SDCPin 7
#define faultyPin 13
#define targetTime 3000 //!!Just for testing, change to 1000
#define errorTimeConstant 3000
#define upshiftPIN 14
#define downshiftPIN 6
#define upshiftTimePotentiometer A6
#define downshiftTimePotentiometer A1
#define calibrationButton 13




/*
* Throttle position 1 329-768
* Throttle postion 2 735-265
* Pedal position 1 200-900
* Pedal position 2 200-900 


Done:
Plausability senzori clapeta
Plausability senzori pedala
Enable for SCS - no difference
Plausability individual senzori pedala
Plausability individual senzori clapeta
If the throttle position differs by more that 10% from excepted target for more that 500 ms
Debounce butoane shifter

Software:
Todo: Check at start for limits - idle check
Todo: Manual mode - potentiometer on the case for easy dyno tuning - Automatic, Potentiometer control, Full Throttle
Todo: P1 error
Todo: check throttle plausability between sensors

Hardware:
Todo: Resistor pull-up/pull-down pentru pinii A
Potentiometre control shifter
Tranzistori
*/
uint16_t tpsLowerBoundry1 = 329;
uint16_t tpsUpperBoundry1 = 779;
uint16_t tpsLowerBoundry2 = 737;
uint16_t tpsUpperBoundry2 = 265;
uint16_t ppsLowerBoundry1 = 200;
uint16_t ppsUpperBoundry1 = 900;
uint16_t ppsLowerBoundry2 = 200;
uint16_t ppsUpperBoundry2 = 900;

uint16_t idleOffset = 30;

long lastPIDCompute = millis();


unsigned long plausabilityTime = 0;
unsigned long errorTime = 0;
volatile bool upshiftButtonPressed = false;
volatile bool downShiftButtonPressed = false;
volatile long gearShiftTime = 0;
unsigned long time = millis();
unsigned long SDCOpenTime = 0;

 enum CalibButtonEvent {
  NO_EVENT,
  CLICK,
  HOLD
};
void calibrateThrottlePedal();
void calibrateIdle();
CalibButtonEvent checkCalibButton();
CalibButtonEvent calibEvent = NO_EVENT;


uint16_t upShiftTime = 300;
uint16_t downShiftTime = 300; //!Max 500 ms

void upShift();
void downShift();
enum gearShift {UPSHIFT, DOWNSHIFT, NOGEARSHIFT};
gearShift gearShiftState = NOGEARSHIFT;

errorCode error;
//1-PEDAL1, 2-PEDAL2, 3-PEDALGENERAL, 4-THROTTLE1, 5-THROTTLE2, 6-THROTTLEGENGERAL, 7-Target

double input, output, setpoint, lastOutput;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
void setup() {
  
    noInterrupts();
    Serial.begin(9600);
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, maxPWMValue);
    myPID.SetSampleTime(1);
    pinMode(LPWM,OUTPUT);
    pinMode(RPWM, OUTPUT);

    pinMode(SDCPin, OUTPUT);
    pinMode(pedalLED, OUTPUT);
    pinMode(throttleLED, OUTPUT);
    pinMode(targetLED, OUTPUT);

    pinMode(SDCPin, OUTPUT);
    digitalWrite(SDCPin, LOW);
    // pinMode(faultyPin, OUTPUT);
    // digitalWrite(faultyPin, LOW);
    
    TCCR2B = (TCCR2B & 0b11111000) | 0x01;

    pinMode(upshiftPIN, OUTPUT);
    pinMode(downshiftPIN, OUTPUT);
    analogWrite(upshiftPIN, 0);
    analogWrite(downshiftPIN, 0);
    pinMode(upshiftPIN, INPUT_PULLUP);
    pinMode(downshiftPIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(2), upShift, FALLING);
    // attachInterrupt(digitalPinToInterrupt(3), downShift, FALLING);
    
    pinMode(calibrationButton, INPUT);
    
    
    EEPROM.get(0, tpsLowerBoundry1);
    EEPROM.get(2, tpsUpperBoundry1);
    EEPROM.get(4, tpsLowerBoundry2);
    EEPROM.get(6, tpsUpperBoundry2);

   

    EEPROM.get(8, ppsLowerBoundry1);
    EEPROM.get(10, ppsUpperBoundry1);
    EEPROM.get(12, ppsLowerBoundry2);
    EEPROM.get(14, ppsUpperBoundry2);

    EEPROM.get(16, idleOffset);
  Serial.println(tpsLowerBoundry1);
  Serial.println(tpsUpperBoundry1);
  Serial.println(tpsLowerBoundry2);
  Serial.println(tpsUpperBoundry2);
  

    interrupts();

    // unsigned long totalTime = 0;
    // const int iterations = 1000;
    // for (int i = 0; i < iterations; i++) {
    // unsigned long startTime = micros();
    // loop();
    // unsigned long endTime = micros();
    // totalTime += (endTime - startTime);
  // }

  // float meanTime = totalTime / (float)iterations;
  // Serial.print("Average execution time: ");
  // Serial.print(meanTime);
  // Serial.println(" microseconds");
}

uint16_t throttlePosition1, throttlePosition2, pedalPosition1, pedalPosition2;
unsigned long currentMillis;



void loop() {
    // Serial.println(millis());
    calibEvent = checkCalibButton();
    
    
    if(calibEvent){
      Serial.println(calibEvent);
      if(calibEvent == CLICK) calibrateIdle();
      else calibrateThrottlePedal();
    }
    // Serial.println(calibEvent);

    //*Sensor readings
    throttlePosition1 = analogRead(throttlePositionSensor1);
    throttlePosition2 = analogRead(throttlePositionSensor2);
    pedalPosition1 = analogRead(pedalPositionSensor1);
    pedalPosition2 = analogRead(pedalPositionSensor2);
    // Serial.println("New Loop");

  
    // Serial.println(throttlePosition1);
    // Serial.println(throttlePosition2);
    
    // Serial.println(analogRead(pedalPositionSensor1));
    // Serial.println(analogRead(pedalPositionSensor2));
    

    //*Plausability checks
    if (throttlePosition1 < tpsLowerBoundry1 - sensorToleranceLow || throttlePosition1 > tpsUpperBoundry1 + sensorToleranceHigh) 
        {error.setError(errorCode::ERROR_THROTTLE1);}
    else if (error.hasError(errorCode::ERROR_THROTTLE1) == true)
        error.clearError(errorCode::ERROR_THROTTLE1);

    if (throttlePosition2 > tpsLowerBoundry2 + sensorToleranceHigh || throttlePosition2 < tpsUpperBoundry2 - sensorToleranceLow)
        error.setError(errorCode::ERROR_THROTTLE2);
    else if (error.hasError(errorCode::ERROR_THROTTLE2) == true)
        error.clearError(errorCode::ERROR_THROTTLE2);
  
    if (pedalPosition1 < ppsUpperBoundry1 - sensorToleranceLow || pedalPosition1 > ppsLowerBoundry1 + sensorToleranceHigh) 
      error.setError(errorCode::ERROR_PEDAL1);
    else if (error.hasError(errorCode::ERROR_PEDAL1)==true)
        error.clearError(errorCode::ERROR_PEDAL1);
    
    if (pedalPosition2 < ppsUpperBoundry2 - sensorToleranceLow || pedalPosition2 > ppsLowerBoundry2 + sensorToleranceHigh) 
      error.setError(errorCode::ERROR_PEDAL2);
    else if (error.hasError(errorCode::ERROR_PEDAL2)==true)
        error.clearError(errorCode::ERROR_PEDAL2);
    
        //!!!
    // if (int16_t((pedalPosition1 - pedalPosition2)) > 120){
    //     error.setError(errorCode::ERROR_PEDALGENERAL); }
    // else if (error.hasError(errorCode::ERROR_PEDALGENERAL) == true) 
    //     error.clearError(errorCode::ERROR_PEDALGENERAL);

    input = throttlePosition1;
    setpoint = pedalPosition1;

    currentMillis = millis();
    input = map(input, tpsLowerBoundry1 + idleOffset, tpsUpperBoundry1, 0, 255);
    setpoint = map(setpoint, ppsLowerBoundry1, ppsUpperBoundry1, 0, 255);

    if (abs(setpoint - input) > 25) {
        if (plausabilityTime == 0) {
            plausabilityTime = currentMillis;} 
        else {
            unsigned long timeElapsed = currentMillis - plausabilityTime;
            if (timeElapsed >= targetTime && error.hasAnyErrors()==0)
                error.setError(errorCode::ERROR_TARGET);
            }
    } 
    else { 
        error.clearError(errorCode::ERROR_TARGET);
        plausabilityTime = 0;
    }
   
   

    //*Error handling
    if (error.hasAnyErrors()){
        setpoint = 0;
        if(error.hasError(errorCode::ERROR_PEDAL1))  digitalWrite(pedalLED, HIGH);
        if(error.hasError(errorCode::ERROR_PEDAL2)) digitalWrite(pedalLED, HIGH);
        if(error.hasError(errorCode::ERROR_PEDALGENERAL)) digitalWrite(pedalLED, HIGH);
        if(error.hasError(errorCode::ERROR_THROTTLE1)) {
        
          if(error.hasError(errorCode::ERROR_THROTTLE2) == 0) 
            input = map(throttlePosition2, tpsLowerBoundry2, tpsUpperBoundry2, 0, 255);
          else digitalWrite(SDCPin, HIGH); 
          digitalWrite(throttleLED, HIGH);
        }
        if(error.hasError(errorCode::ERROR_THROTTLE2)) digitalWrite(throttleLED, HIGH);
        if(error.hasError(errorCode::ERROR_THROTTLEGENERAL)) digitalWrite(throttleLED, HIGH);
        if(error.hasError(errorCode::ERROR_TARGET)) digitalWrite(targetLED, HIGH);
        
        
    
        if(errorTime==0) errorTime = currentMillis;
        if (currentMillis-errorTime>=errorTimeConstant)
          if(input>=10) {
          digitalWrite(SDCPin, HIGH); 
          SDCOpenTime = millis();
          }
      }
    else { 
        digitalWrite(pedalLED, LOW);
        digitalWrite(throttleLED, LOW);
        digitalWrite(targetLED, LOW);
        if (currentMillis-SDCOpenTime>=errorTimeConstant){
            digitalWrite(SDCPin, LOW);    
        } 
        
        errorTime = 0;
       
    }
    

 
     //*PID Computing
    
    switch(gearShiftState){
      case UPSHIFT:
        
        if(millis()-gearShiftTime > upShiftTime) {
          gearShiftState = NOGEARSHIFT;
          upshiftButtonPressed = false;
          analogWrite(upshiftPIN, 0);
          analogWrite(downshiftPIN, 0);
        }
        else if (error.hasAnyErrors() == 0){
          setpoint = 0;
          analogWrite(upshiftPIN, 255);
          analogWrite(downshiftPIN, 0);} 
        break;
  
      case DOWNSHIFT:
        if(millis()-gearShiftTime > downShiftTime) {
          gearShiftState = NOGEARSHIFT;
          downShiftButtonPressed = false;
          analogWrite(downshiftPIN, 0);
          analogWrite(upshiftPIN, 0);
        }
        else if (error.hasAnyErrors() == 0) {
          setpoint = 255;
          analogWrite(downshiftPIN, 255);
          analogWrite(upshiftPIN, 0);
        }
        break;
  
      case NOGEARSHIFT:
        if(upshiftButtonPressed == true) {
          gearShiftTime = millis();
          gearShiftState = UPSHIFT;
          upshiftButtonPressed = false;
        }

        if(downShiftButtonPressed == true){
          gearShiftTime = millis();
          gearShiftState = DOWNSHIFT;
          downShiftButtonPressed = false;          
        }

        break;
    }
  
    if (millis() - lastPIDCompute > 1) {
      lastOutput = output;
      myPID.Compute();
      lastPIDCompute = millis();
      if(output - lastOutput < 1 && output - lastOutput > -1)
        output = lastOutput;
    }

    // myPID.Compute();
    // Serial.println(output);
    if(output <= minPWMValue || error.hasAnyErrors() == 1) {
      output = 0;
      analogWrite(LPWM, 0);
      if(input>10 && error.hasAnyErrors() == 0 && gearShiftState == NOGEARSHIFT) analogWrite(RPWM, 60);
      else analogWrite(RPWM, 0);
    }
    else {
     analogWrite(RPWM, 0);
     analogWrite(LPWM, output);
    }
    
    
    
  
    upShiftTime = map(analogRead(upshiftTimePotentiometer), 0, 1024, 50, 500);
    downShiftTime = map(analogRead(downshiftTimePotentiometer), 0, 1024, 50, 500);
  
  }

  void upShift(){
    Serial.println("UPSHIFT UPSHIFT UPSHIFT");
    upshiftButtonPressed = true; 
  }

  void downShift(){
    Serial.println("DOWNSHIFT DOWNSHIFT DOWNUPSHIFT");
    downShiftButtonPressed = true;
  }

  
  void pedalCalibration(){
    for(int i=0;i<8;i++){
      digitalWrite(pedalLED, i%2);
      delay(500);
    }
    Serial.println("Lower side calibration");
    

    while(digitalRead(calibrationButton) == LOW){
      ppsLowerBoundry1 = analogRead(pedalPositionSensor1);
      ppsLowerBoundry2 = analogRead(pedalPositionSensor2);
      Serial.print(ppsLowerBoundry1);
      Serial.print("   ");
      Serial.println(ppsLowerBoundry2);
    }

    EEPROM.put(8, ppsLowerBoundry1);
    EEPROM.put(12, ppsLowerBoundry2);

    

    for(int i=0;i<8;i++){
      digitalWrite(pedalLED, i%2);
      delay(500);
    }
    digitalWrite(pedalLED, HIGH);
    Serial.println("Upper side calibration");

    while(digitalRead(calibrationButton) == LOW){  
      ppsUpperBoundry1 = analogRead(pedalPositionSensor1);
      ppsUpperBoundry2 = analogRead(pedalPositionSensor2);
      Serial.print(ppsUpperBoundry1);
      Serial.print("   ");
      Serial.println(ppsUpperBoundry2);
    } 

    EEPROM.put(10, ppsUpperBoundry1);
    EEPROM.put(14, ppsUpperBoundry2);

    Serial.println(ppsLowerBoundry1);
    Serial.println(ppsLowerBoundry2);
    Serial.println(ppsUpperBoundry1);
    Serial.println(ppsUpperBoundry2);
    
    for(int i=0;i<8;i++){
      digitalWrite(pedalLED, i%2);
      delay(500);
    }

    digitalWrite(pedalLED, LOW);
  }

  void throttleCalibration() {
    digitalWrite(throttleLED, HIGH);
    bool upper = true;
    analogWrite(LPWM, 0);
    delay(1000);
    uint16_t currentValue = analogRead(throttlePositionSensor1);

    while(upper) {
      analogWrite(LPWM, 120);
      Serial.println("Inainte de delay");
      delay(2000);
      Serial.println("Dupa delay");
      uint16_t lastValue = currentValue;
      currentValue = analogRead(throttlePositionSensor1);
      uint16_t upperBound = lastValue * (1+0.05);

      if(abs(currentValue - lastValue) <= 10){
        Serial.print("in if");
        Serial.println(abs(currentValue - lastValue));
        upper=0;
        tpsUpperBoundry1 = analogRead(throttlePositionSensor1);
        tpsUpperBoundry2 = analogRead(throttlePositionSensor2);
        analogWrite(LPWM, 0);
      }

    }
    Serial.println("test");
    delay(2000);

    tpsLowerBoundry1 = analogRead(throttlePositionSensor1);
    tpsLowerBoundry2 = analogRead(throttlePositionSensor2);

    EEPROM.put(0, tpsLowerBoundry1);
    EEPROM.put(2, tpsUpperBoundry1);
    EEPROM.put(4, tpsLowerBoundry2);
    EEPROM.put(6, tpsUpperBoundry2);

    Serial.println(tpsLowerBoundry1);
    Serial.println(tpsLowerBoundry2);
    Serial.println(tpsUpperBoundry1);
    Serial.println(tpsUpperBoundry2);


    digitalWrite(LPWM, 0);
    digitalWrite(throttleLED, LOW);
  }


  bool buttonVal = HIGH; 
  bool buttonLast = HIGH;
  long downTime = -1;
  long upTime = -1;         
  boolean holdEventPast = false;  
  boolean longHoldEventPast = false;

  int debounce = 20; 
  int holdTime = 1000; 

  void calibrateIdle() {
    Serial.println("Idle throttle");
    delay(100);
    uint16_t temp = 0;
    analogWrite(LPWM, 0);
    while(checkCalibButton()==0){
      setpoint = map(analogRead(downshiftTimePotentiometer), 650, 700, 0, 10);
      if(setpoint > 10)
        digitalWrite(pedalLED, HIGH);
      if(setpoint <=0) digitalWrite(throttleLED, HIGH);
      else 
        digitalWrite(throttleLED, LOW); digitalWrite(pedalLED, LOW);
      input =  map(analogRead(throttlePositionSensor1), tpsLowerBoundry1, tpsUpperBoundry1, 0, 255);
      myPID.Compute();
      analogWrite(LPWM, output);
      Serial.println(setpoint);
    }
    digitalWrite(throttleLED, LOW); digitalWrite(pedalLED, LOW);
    idleOffset = map(analogRead(throttlePositionSensor1), tpsLowerBoundry1, tpsUpperBoundry1, 0, 255);
    EEPROM.put(16, idleOffset);
    analogWrite(LPWM, 0);
    delay(100);
    Serial.println("Out of idle calibration");
  }

  void calibrateThrottlePedal(){
    // Serial.println(digitalRead(calibrationButton));
    pedalCalibration();
    // throttleCalibration();
  }

 


  CalibButtonEvent checkCalibButton() {
  const uint16_t debounceTime = 20;
  const uint16_t holdThreshold = 1000;

  static bool lastState = false;
  static bool holdDetected = false;
  static uint32_t lastChangeTime = 0;
  static uint32_t pressStartTime = 0;

  bool currentState = digitalRead(calibrationButton);
  uint32_t now = millis();

  if (currentState != lastState) {
    lastChangeTime = now;
    lastState = currentState;
    delay(1); // minimal debounce stabilization
  }

  if (currentState && (now - pressStartTime >= holdThreshold) && !holdDetected) {
    holdDetected = true;
    return HOLD;
  }

  if (currentState && pressStartTime == 0) {
    pressStartTime = now;
    holdDetected = false;
  }

  if (!currentState && pressStartTime > 0) {
    uint32_t pressDuration = now - pressStartTime;
    pressStartTime = 0;
    if (!holdDetected && pressDuration >= debounceTime && pressDuration < holdThreshold) {
      return CLICK;
    }
  }

  return NO_EVENT;
}
  

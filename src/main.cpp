#include <Arduino.h>
#include <PID/PID_v1.h>
#include "error/error.hpp"


#define LPWM 11
#define ELPWM 12
#define throttlePositionSensor1 A5
#define throttlePositionSensor2 A4 //!A5 is Just for testing, need to be 2 different inputs
#define pedalPositionSensor1 A0
#define pedalPositionSensor2 A1 //!Just for testing, need to be 2 different inputs
#define maxPWMValue 150
#define minPWMValue 60
#define Kp 0.35
#define Ki 2.13 
#define Kd 0.00164870
#define downShiftBlipPower 180
#define sensorToleranceLow 10
#define sensorToleranceHigh 10
#define pedalLED 7
#define throttleLED 6
#define targetLED 13
#define SDCPin 4
#define faultyPin 13
#define targetTime 3000
#define errorTimeConstant 3000
#define upshiftPIN 8
#define downshiftPIN 9
#define upshiftTimePotentiometer A6
#define downshiftTimePotentiometer A6
#define calibrationButton 12


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
Debounce butoane shifter

Software:
Todo: Check at start for limits - idle check
Todo: Manual mode - potentiometer on the case for easy dyno tuning - Automatic, Potentiometer control, Full Throttle
Todo: P1 error

Hardware:
Todo: Resistor pull-up/pull-down pentru pinii A
Potentiometre control shifter
Tranzistori
*/
uint16_t tpsLowerBoundry1 = 311;
uint16_t tpsUpperBoundry1 = 789;
uint16_t tpsLowerBoundry2 = 729;
uint16_t tpsUpperBoundry2 = 231;
uint16_t ppsLowerBoundry1 = 200;
uint16_t ppsUpperBoundry1 = 900;
uint16_t ppsLowerBoundry2 = 200;
uint16_t ppsUpperBoundry2 = 900;

unsigned long plausabilityTime = 0;
unsigned long errorTime = 0;
volatile bool upshiftButtonPressed = false;
volatile bool downShiftButtonPressed = false;
volatile long gearShiftTime = 0;
unsigned long time = millis();
unsigned long SDCOpenTime = 0;
void calibrate();

uint16_t upShiftTime = 300;
uint16_t downShiftTime = 300; //!Max 500 ms
void upShift();
void downShift();
enum gearShift {UPSHIFT, DOWNSHIFT, NOGEARSHIFT};
gearShift gearShiftState = NOGEARSHIFT;

errorCode error;
//1-PEDAL1, 2-PEDAL2, 3-PEDALGENERAL, 4-THROTTLE1, 5-THROTTLE2, 6-THROTTLEGENGERAL, 7-Target

double input, output, setpoint;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
void setup() {
    noInterrupts();
    Serial.begin(9600);
  

    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, maxPWMValue);
    myPID.SetSampleTime(10);
    pinMode(LPWM,OUTPUT);


    pinMode(SDCPin, OUTPUT);
    digitalWrite(SDCPin, LOW);
    pinMode(faultyPin, OUTPUT);
    digitalWrite(faultyPin, LOW);
    
    TCCR2B = (TCCR2B & 0b11111000) | 0x01;

    pinMode(upshiftPIN, OUTPUT);
    pinMode(downshiftPIN, OUTPUT);
    analogWrite(upshiftPIN, 0);
    analogWrite(downshiftPIN, 0);
    pinMode(upshiftPIN, INPUT_PULLUP);
    pinMode(downshiftPIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), upShift, FALLING);
    attachInterrupt(digitalPinToInterrupt(3), downShift, FALLING);

    pinMode(calibrationButton, INPUT_PULLUP);
    interrupts();
}

int throttlePosition1, throttlePosition2, pedalPosition1, pedalPosition2;
unsigned long currentMillis;



void loop() {
    if(digitalRead(calibrationButton) == 0) calibrate();
    //*Sensor readings
    throttlePosition1 = analogRead(throttlePositionSensor1);
    throttlePosition2 = analogRead(throttlePositionSensor2);
    pedalPosition1 = analogRead(pedalPositionSensor1);
    pedalPosition2 = analogRead(pedalPositionSensor2);

    //*Plausability checks
    if (throttlePosition1 < tpsLowerBoundry1 - sensorToleranceLow || throttlePosition1 > tpsUpperBoundry1 + sensorToleranceHigh) 
        {error.setError(errorCode::ERROR_THROTTLE1);}
    else if (error.hasError(errorCode::ERROR_THROTTLE1) == true)
        error.clearError(errorCode::ERROR_THROTTLE1);

    if (throttlePosition2 > tpsLowerBoundry2 + sensorToleranceHigh || throttlePosition2 < tpsUpperBoundry2 - sensorToleranceLow)
        error.setError(errorCode::ERROR_THROTTLE2);
    else if (error.hasError(errorCode::ERROR_THROTTLE2) == true)
        error.clearError(errorCode::ERROR_THROTTLE2);
  
    if (pedalPosition1 < ppsLowerBoundry1 - sensorToleranceLow || pedalPosition1 > ppsUpperBoundry1 + sensorToleranceHigh) 
      error.setError(errorCode::ERROR_PEDAL1);
    else if (error.hasError(errorCode::ERROR_PEDAL1)==true)
        error.clearError(errorCode::ERROR_PEDAL1);
    
    if (pedalPosition2 < ppsLowerBoundry2 - sensorToleranceLow || pedalPosition2 > ppsUpperBoundry2 + sensorToleranceHigh) 
      error.setError(errorCode::ERROR_PEDAL2);
    else if (error.hasError(errorCode::ERROR_PEDAL2)==true)
        error.clearError(errorCode::ERROR_PEDAL2);

    if (abs(pedalPosition1 - pedalPosition2) > 120)
        error.setError(errorCode::ERROR_PEDALGENERAL); 
    else if (error.hasError(errorCode::ERROR_PEDALGENERAL) == true) 
        error.clearError(errorCode::ERROR_PEDALGENERAL);

    input = throttlePosition1;
    setpoint = pedalPosition1;


 
    

    currentMillis = millis();
    input = map(input, tpsLowerBoundry1, tpsUpperBoundry1, 0, 255);
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
    myPID.Compute();
    
    if(output <= minPWMValue + 10 || error.hasAnyErrors() == 1) output = 0;

      analogWrite(LPWM, output);
    
  
    upShiftTime = map(analogRead(upshiftTimePotentiometer), 0, 1024, 50, 500);
    downShiftTime = map(analogRead(downshiftTimePotentiometer), 0, 1024, 50, 500);
  
  }

  void upShift(){
    upshiftButtonPressed = true; 
  }

  void downShift(){
    downShiftButtonPressed = true;
  }

  
  void pedalCalibration(){
    for(int i=0;i<8;i++){
      digitalWrite(pedalLED, i%2);
      delay(500);
    }
    Serial.println("Lower side calibration");

    while(digitalRead(calibrationButton) == 1){
      ppsLowerBoundry1 = analogRead(pedalPositionSensor1);
      ppsLowerBoundry2 = analogRead(pedalPositionSensor2);
      Serial.print(ppsLowerBoundry1);
      Serial.print("   ");
      Serial.println(ppsLowerBoundry2);
    }

    for(int i=0;i<8;i++){
      digitalWrite(pedalLED, i%2);
      delay(500);
    }
    digitalWrite(pedalLED, HIGH);
    Serial.println("Upper side calibration");

    while(digitalRead(calibrationButton) == 1){  
      ppsUpperBoundry1 = analogRead(pedalPositionSensor1);
      ppsUpperBoundry2 = analogRead(pedalPositionSensor2);
      Serial.print(ppsUpperBoundry1);
      Serial.print("   ");
      Serial.println(ppsUpperBoundry2);
    } 

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
    Serial.println(tpsLowerBoundry1);
    Serial.println(tpsLowerBoundry2);
    Serial.println(tpsUpperBoundry1);
    Serial.println(tpsUpperBoundry2);


    digitalWrite(LPWM, 0);
  }
  
  void calibrate(){
    pedalCalibration();
    throttleCalibration();
    

  }

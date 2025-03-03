#include <Arduino.h>
#include <PID/PID_v1.h>
#include "error/error.hpp"


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
#define sensorToleranceLow 10
#define sensorToleranceHigh 10
#define tpsLowerBoundry1  311
#define tpsUpperBoundry1  789
#define tpsLowerBoundry2 729
#define tpsUpperBoundry2 231
#define pedalLED 13
#define throttleLED 13
#define targetLED 13
#define SDCPin 4
#define faultyPin 13
#define targetTime 3000
#define errorTimeConstant 3000


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

unsigned long plausabilityTime = 0;
unsigned long errorTime = 0;
volatile bool upshiftButtonPressed = false;
volatile bool downShiftButtonPressed = false;
volatile long gearShiftTime = 0;
unsigned long time = millis();
unsigned long SDCOpenTime = 0;

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


    // pinMode(2, INPUT_PULLUP);
    // pinMode(3, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(2), upShift, FALLING);
    // attachInterrupt(digitalPinToInterrupt(3), downShift, FALLING);

    interrupts();
}

int throttlePosition1, throttlePosition2, pedalPosition1, pedalPosition2;
unsigned long currentMillis;

void loop() {
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
    
    //Todo: lower-upper boundries 
    if (pedalPosition1 < 100 || pedalPosition1 > 1024) error.setError(errorCode::ERROR_PEDAL1);
    else if (error.hasError(errorCode::ERROR_PEDAL1)==true)
        error.clearError(errorCode::ERROR_PEDAL1);
    
    if (pedalPosition2 < 0 || pedalPosition2 > 1024) error.setError(errorCode::ERROR_PEDAL2);
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
    setpoint = map(setpoint, 100, 1024, 0, 255);




    if (abs(setpoint - input) > 25) {
        if (plausabilityTime == 0) {
            plausabilityTime = currentMillis;} 
        else {
            unsigned long timeElapsed = currentMillis - plausabilityTime;
            if (timeElapsed >= targetTime && error.hasAnyErrors()!=0)
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
        if(error.hasError(errorCode::ERROR_THROTTLE1)) digitalWrite(throttleLED, HIGH);
        if(error.hasError(errorCode::ERROR_THROTTLE2)) digitalWrite(throttleLED, HIGH);
        if(error.hasError(errorCode::ERROR_THROTTLEGENERAL)) digitalWrite(throttleLED, HIGH);
        if(error.hasError(errorCode::ERROR_TARGET)) digitalWrite(targetLED, HIGH);
    
        if(errorTime==0) errorTime = currentMillis;
        else if (currentMillis-errorTime>=errorTimeConstant && input>=10) {
            digitalWrite(SDCPin, HIGH); 
            SDCOpenTime = millis();
            }
        
    
    }
    else { 
        digitalWrite(pedalLED, LOW);
        digitalWrite(throttleLED, LOW);
        digitalWrite(targetLED, LOW);
        if (currentMillis-SDCOpenTime<=1000){
            digitalWrite(SDCPin, LOW);
            
            
        } 
        else digitalWrite(SDCPin, LOW);
        errorTime = 0;
       
    }
    //!!Delete serials
    // Serial.println("NL");
    Serial.println(setpoint);

 
     //*PID Computing
    myPID.Compute();

    if(output <= minPWMValue + 10 || error.hasAnyErrors() == 1) output = 0;
    analogWrite(LPWM, output);
   
    
  }
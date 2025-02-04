#include "error.hpp"



void errorCode::setError(errorCodes temp) {
    errorFlag |= temp;
}
void errorCode::clearError(errorCodes temp){
    errorFlag &= ~temp;
}
bool errorCode::hasError(errorCodes temp){
    return errorFlag & temp;
}
bool errorCode::hasAnyErrors() {
    return errorFlag != 0;
}
void errorCode::clearAllErrors(){
    errorFlag = 0;
}
void errorCode::printError(){
    Serial.print("Pedal1: ");
    Serial.println(errorFlag & ERROR_PEDAL1);
    Serial.print("Pedal2: ");
    Serial.println(errorFlag & ERROR_PEDAL2);
    Serial.print("PedalGeneral: ");
    Serial.println(errorFlag & ERROR_PEDALGENERAL);
    Serial.print("Throttle1: ");
    Serial.println(errorFlag & ERROR_THROTTLE1);
    Serial.print("Throttle2: ");
    Serial.println(errorFlag & ERROR_THROTTLE2);
    Serial.print("ThrottleGeneral: ");
    Serial.println(errorFlag & ERROR_THROTTLEGENERAL);
}
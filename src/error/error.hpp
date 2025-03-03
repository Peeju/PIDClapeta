#include <Arduino.h>

class errorCode{
    // enum errorCode{
    //     GENERALERROR,
    //     PEDAL1ERROR,
    //     PEDAL2ERROR,
    //     PEDALGENERALERROR,
    //     THROTTLE1ERROR,
    //     THROTTLE2ERROR,
    //     THROTTLEGENERALERROR
        
    // }
public:
    enum errorCodes {
        ERROR_PEDAL1 = 1 << 0,  
        ERROR_PEDAL2 = 1 << 1, 
        ERROR_PEDALGENERAL = 1 << 2,   
        ERROR_THROTTLE1 = 1 << 3,
        ERROR_THROTTLE2 = 1 << 4, 
        ERROR_THROTTLEGENERAL = 1 << 5,
        ERROR_TARGET = 1 << 6 
    };

    void setError(errorCodes temp);
    void clearError(errorCodes temp);
    bool hasError(errorCodes temp);
    bool hasAnyErrors();
    void clearAllErrors();
    void printError();

private:
    uint16_t errorFlag;
    

};
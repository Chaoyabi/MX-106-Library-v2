#include "mbed.h"
#include "Mx106v2.h"
#include "CRC_cal"

//------------------------------------------------------------------------------
// Public Methods 
//------------------------------------------------------------------------------

DynamixelClass_v2::DynamixelClass_v2(int baud, PinName D_Pin, PinName tx, PinName rx){ 
    servoSerial = new Serial(tx, rx);
    servoSerial -> baud(baud);
    servoSerialDir = new DigitalOut(D_Pin);     
    servoSerialDir -> write(0);
}   


DynamixelClass_v2::~DynamixelClass_v2(){
    if(servoSerial != NULL) delete servoSerial;
    if(servoSerialDir != NULL) delete servoSerialDir;
}

int DynamixelClass_v2::test(){
    int b;
    b = test_get_a();
    return b;
}

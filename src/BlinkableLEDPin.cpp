/*********************************************************************************
 *  BlinkableLEDPin - Enhanced LED control with PWM and configurable logic levels
 *  
 *  Implementation file for the BlinkableLEDPin class
 ********************************************************************************/

#include "BlinkableLEDPin.h"

////////////////////////////////
//    BlinkableLEDPin Class   //
////////////////////////////////

BlinkableLEDPin::BlinkableLEDPin(int pin, float onLevel, uint16_t freq, boolean invert) 
    : LedPin(pin, 0, freq, invert), onLevel(onLevel), pin(pin) {
}

void BlinkableLEDPin::on() {
    set(onLevel);
}

void BlinkableLEDPin::off() {
    set(0);
}

void BlinkableLEDPin::setOnLevel(float level) {
    onLevel = constrain(level, 0.0, 100.0);
}

int BlinkableLEDPin::getPin() {
    return pin;
}

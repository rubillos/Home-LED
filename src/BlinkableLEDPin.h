/*********************************************************************************
 *  BlinkableLEDPin - Enhanced LED control with PWM and configurable logic levels
 *  
 *  Extends the HomeSpan LedPin class to provide PWM LED control with configurable
 *  on/off levels and integrated blinking functionality.
 ********************************************************************************/

#pragma once

#include <Arduino.h>
#include "HomeSpan.h"

////////////////////////////////
//    BlinkableLEDPin Class   //
////////////////////////////////

class BlinkableLEDPin : public LedPin, public Blinkable {
private:
    float onLevel;     // PWM level for "on" state (0-100%)
    int pin;

public:
    // Constructor
    BlinkableLEDPin(int pin, float onLevel = 100.0, uint16_t freq = DEFAULT_PWM_FREQ, boolean invert = false);
    
    // Blinkable interface implementation
    void on() override;
    void off() override;
    int getPin() override;
    
    // Enhanced functionality
    void setOnLevel(float level);
};

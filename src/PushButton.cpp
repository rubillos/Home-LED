#include <Arduino.h>

#include "PushButton.h"

Button::Button(int16_t pinNumber, bool doublePress, uint32_t longTimeSeconds) {
  _pinNumber = pinNumber;
  _longMillis = longTimeSeconds * 1000;
  _doublePress = doublePress;
  _lastShortPressTime = 10000;
  if (_pinNumber != -1) {
    pinMode(_pinNumber, INPUT_PULLUP);
  }
}

int16_t Button::pinNumber() {
  return _pinNumber;
}

void Button::swapPinsWith(Button &otherButton) {
  uint8_t otherPin = otherButton._pinNumber;
  otherButton._pinNumber = _pinNumber;
  _pinNumber = otherPin;
  _lastShortPressTime = 10000;
  _buttonIsPressed = false;
  _returnedLongPress = false;
}

buttonPressType Button::checkPress() {
  buttonPressType pressed = notPressed;
  bool currentButtonState = isCurrentlyPressed();

  if (currentButtonState != _lastButtonState) {
    _stateChangeTime = 0;
    _lastButtonState = currentButtonState;
    _repeatDelay = 0;
    _repeatRate = 0;
  }

  if (_stateChangeTime >= DEBOUNCE_TIME) {
    if (currentButtonState) {       // currently pressed
      if ((_longMillis > 0) && (_stateChangeTime >= _longMillis)) {
        if (!_returnedLongPress) {
          pressed = longPress;
          _returnedLongPress = true;
        }
      }
      if ((_longMillis == 0) && !_buttonIsPressed) {
        pressed = shortPress;
      }
      if ((_longMillis == 0) && (_repeatDelay > REPEAT_DELAY)) {
        if (_repeatRate > REPEAT_RATE) {
          pressed = shortPress;
          _repeatRate = 0;
        }
      }
    }
    else {
      if (_buttonIsPressed) {
        if (_returnedLongPress) {
          // this gets returned while still down
        }
        else if (_doublePress && _lastShortPressTime < DOUBLE_PRESS_TIME) {
          pressed = doublePress;
        }
        else if (_longMillis > 0) {
          _lastShortPressTime = 0;
          pressed = shortPress;
        }
      }
      _returnedLongPress = false;
    }

    _buttonIsPressed = currentButtonState;
  }

  return(pressed);
}

bool Button::isCurrentlyPressed() {
  return (_pinNumber == -1) ? false : (digitalRead(_pinNumber) == LOW);
}

#include "Arduino.h"

#include "HomeSpan.h" 
#include <Preferences.h>
#include "Pixel2.h"

//--------------------------------------------------------------------

#if defined(DEBUG)
#define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//--------------------------------------------------------------------

constexpr uint8_t statusStripPin = 33;
constexpr uint8_t statusStripLength = 4;

constexpr uint8_t indicatorDataPin = RGB_DATA;	// 40
constexpr uint8_t indicatorPowerPin = RGB_PWR;	// 39

constexpr uint8_t ambientLightPin = 4;

//--------------------------------------------------------------------
// Pins

constexpr uint8_t pinServoWasher = MOSI; // 35
constexpr uint8_t pinServoDryer = SCK;	//36
constexpr uint8_t pinServoInverter = 5;
constexpr uint8_t pinServoConverter = 6;

constexpr uint8_t pinSenseWasher = 17;
constexpr uint8_t pinSenseDryer = 18;
constexpr uint8_t pinSenseInverter = 14;
constexpr uint8_t pinSenseConverter = 12;

constexpr uint8_t pinLedWasher = TX;	// 43
constexpr uint8_t pinLedDryer = SDA;	// 8
constexpr uint8_t pinLedInverter = MISO;	// 37
constexpr uint8_t pinLedConverter = SCL;	// 9

constexpr uint8_t pinButtonWasher = 7;
constexpr uint8_t pinButtonDryer = 3;
constexpr uint8_t pinButtonInverter = 1;
constexpr uint8_t pinButtonConverter = 38;

constexpr uint8_t statusIndexWasher = 0;
constexpr uint8_t statusIndexDryer = 1;
constexpr uint8_t statusIndexInverter = 2;
constexpr uint8_t statusIndexConverter = 3;

//--------------------------------------------------------------------
constexpr uint16_t ADC_Max = 4095;     // This is the default ADC max value on the ESP32 (12 bit ADC width);

constexpr uint16_t adcOnLevel = 2400; // ADC value above which the breaker is considered ON
constexpr uint16_t adcOffLevel = 2200; // ADC value above which the breaker is considered OFF

constexpr uint16_t servoMinUSec = 500;
constexpr uint16_t servoMaxUSec = 2500;

constexpr double servoDisable = NAN;
constexpr double servoMinAngle = -90;
constexpr double servoMaxAngle = 90;

constexpr int servoCenterAngle = 0;

constexpr uint32_t servoPushTimeMS = 250;

constexpr uint32_t senseChangeTimeMS = servoPushTimeMS + 50;
constexpr uint32_t breakerDelayTimeMS = servoPushTimeMS + 100;
constexpr uint32_t buddyDelayTimeMS = 0;

constexpr uint8_t ledDarkBrightness = 10;
constexpr uint8_t ledBrightBrightness = 100;

constexpr uint8_t statusStripDarkBrightness = 1;
constexpr uint8_t statusStripBrightBrightness = 20;

constexpr uint16_t darkLowThreshold = 700;
constexpr uint16_t darkHighThreshold = 1000;

constexpr uint32_t lightCheckRateMS = 1000;

constexpr uint32_t statusUpdateRateMS = 1000;

//--------------------------------------------------------------------

constexpr const char* displayName = "PwrCenter-Controller";
constexpr const char* modelName = "PwrCenter-Controller-ESP32-S3";
constexpr const char* versionString = "v1.0";

//--------------------------------------------------------------------

Pixel2 indicator(indicatorDataPin, "GRB");
uint8_t pixelBrightness = 32;

//--------------------------------------------------------------------

Preferences preferences;

const char* prefsPartitionName = "PwrPrefs";
const char* movePrefName = "moveAngle";

constexpr int16_t initialServoMoveAngle = 20;
int16_t servoMoveAngle = initialServoMoveAngle;

//--------------------------------------------------------------------

constexpr uint8_t ledStatusLevel = 0x10;

#define MAKE_RGB(r, g, b) (r<<16 | g<<8 | b)
constexpr uint32_t flashColor = MAKE_RGB(ledStatusLevel, 0, ledStatusLevel);
constexpr uint32_t startColor = MAKE_RGB(ledStatusLevel, 0, 0);
constexpr uint32_t connectingColor = MAKE_RGB(0, 0, ledStatusLevel);
constexpr uint32_t readyColor = MAKE_RGB(0, ledStatusLevel, 0);

constexpr uint32_t unknownColor = 0xFFFFFFFE;

uint32_t currentIndicatorColor = unknownColor;

//--------------------------------------------------------------------

uint64_t millis64() {
	volatile static uint32_t low32 = 0, high32 = 0;
	uint32_t new_low32 = millis();

	if (new_low32 < low32)
		high32++;

	low32 = new_low32;

	return (uint64_t) high32 << 32 | low32;
}

//--------------------------------------------------------------------

uint16_t analogReadClean(uint8_t pin) {
	analogRead(pin);
	analogRead(pin);
	return analogRead(pin);
}

//--------------------------------------------------------------------

void enablePower() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);
}

//--------------------------------------------------------------------

void printAndBackUp(const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	size_t n = SerPrintf(fmt, args);
	va_end(args);

	for (auto _=0; _< n; _++) {
		Serial.print('\b');
	}
}

//--------------------------------------------------------------------

inline uint8_t mul8x8(uint8_t a, uint8_t b) { return (a * b) / 255; }
inline uint8_t pixBright(uint8_t value) {
	if (value) { return max((uint8_t)1, mul8x8(value, pixelBrightness)); }
	else return 0;
}

void setIndicatorBrightness(uint8_t brightness) {
	pixelBrightness = brightness;
}

void setIndicator(uint32_t color) {
	if (color != currentIndicatorColor) {
		currentIndicatorColor = color;
		indicator.set(Pixel::RGB(pixBright(color>>16), pixBright((color>>8) & 0xFF), pixBright(color & 0xFF)), 1);
	}
}

//--------------------------------------------------------------------
bool waitForY() {
	while (true) {
		if (Serial.available()) {
			char c = Serial.read();
			if (c == 'Y' || c == 'y') {
				return true;
			}
			else if (c == 'N' || c == 'n') {
				return false;
			}
		}
	}
}

//--------------------------------------------------------------------
typedef enum {
	breakerStateOff = 0,
	breakerStateOn = 1,
	breakerStateTripped = -1,
	breakerStateUnknown = -2
} BreakerState_t;

struct StatusStrip {
	Pixel2 *_strip = NULL;
	uint16_t _length;
	uint8_t _brightness = 255;
	BreakerState_t* _status = NULL;
	Pixel::Color* _colors = NULL;

	void setBrightness(uint8_t brightness, bool show = true) {
		if (brightness != _brightness) {
			_brightness = brightness;
			if (show) {
				showPixels();
			}
		}
	}

	Pixel::Color colorWithBrightness(uint8_t red, uint8_t green, uint8_t blue) {
		return Pixel::RGB(mul8x8(red, _brightness), mul8x8(green, _brightness), mul8x8(blue, _brightness));
	}

	Pixel::Color colorForState(BreakerState_t state) {
		switch (state) {
			case breakerStateOff: 		return colorWithBrightness(255, 0, 0); 		// offColor
			case breakerStateOn: 		return colorWithBrightness(0, 255, 0); 		// onColor
			case breakerStateTripped: 	return colorWithBrightness(255, 255, 0); 	// trippedColor
			default: 					return colorWithBrightness(0, 0, 0); 		// blackColor
		}
	}

	void showPixels() {
		if (_strip == NULL || _status == NULL || _colors == NULL) {
			return;
		}

		for (int i = 0; i < _length; i++) {
			_colors[i] = colorForState(_status[i]);
		}
		_strip->set(_colors, _length);
	}

	void setPixel(uint8_t index, BreakerState_t state, bool show = true) {
		if (index < _length && state != _status[_length - 1 - index]) {
			_status[_length - 1 - index] = state;
			if (show) {
				showPixels();
			}
		}
	}

	void clear(bool show = true) {
		for (auto i = 0; i < _length; i++) {
			_status[i] = breakerStateUnknown;
		}
		if (show) {
			showPixels();
		}
	}

	void begin(uint8_t pin, uint16_t length) {
		_brightness = 255;
		_length = length;
		_strip = new Pixel2(pin, "RGB");
		_strip->setTiming(0.35, 0.8, 0.7, 0.6, 80.0);
		_status = new BreakerState_t[_length];
		_colors = new Pixel::Color[_length];
		for (auto i=0; i<_length; i++) {
			_status[i] = breakerStateUnknown;
		}
		showPixels();
	}
};

StatusStrip statusStrip;

constexpr int trippedSensorTrue = Characteristic::ContactSensorState::DETECTED;
constexpr int trippedSensorFalse = Characteristic::ContactSensorState::NOT_DETECTED;

struct BreakerController : Service::Outlet {
	SpanCharacteristic* _power;
	SpanCharacteristic* _tripped;

	ServoPin* _servo = NULL;
	double _servoAngle = servoDisable;
	double _angleOffset = 0;

	BreakerController* _buddyBreaker = NULL;
	StatusStrip* _statusStrip = NULL;

	uint8_t _pinSense;
	uint16_t _senseOffLevel;
	uint16_t _senseOnLevel;
	int _statusIndex;

	LedPin* _led = NULL;
	int _ledState = -1;
	bool _dark = false;

	int _changeToValue = -1;

	int _delayedChangeTo = -1;
	uint64_t _delayedTime = 0;

	uint64_t _servoMoveTime = 0;
	uint64_t _senseHoldTime = 0;

	bool _breakerOn;

	String _anglePrefName;
	String _onPrefName;
	String _offPrefName;

	BreakerController(int servoPin, bool reverseServo, int pinButton, int pinSense, int ledPin, StatusStrip* statusStrip, int statusIndex) : Service::Outlet(){

		new Characteristic::OutletInUse(true);

		_pinSense = pinSense;
		BreakerState_t state = breakerState();
		_breakerOn = state == breakerStateOn;
		_power = new Characteristic::On(_breakerOn);

		new SpanButton(pinButton, 1000, 5, 0, SpanButton::TRIGGER_ON_LOW);

		_servo = new ServoPin(servoPin, servoDisable, servoMinUSec, servoMaxUSec, reverseServo ? servoMinAngle : servoMaxAngle, reverseServo ? servoMaxAngle : servoMinAngle);
		_led = new LedPin(ledPin, 0);

		_statusStrip = statusStrip;
		_statusIndex = statusIndex;

		String indexString = String(_statusIndex);
		_anglePrefName = String("aOffset") + indexString;
		_onPrefName = String("on") + indexString;
		_offPrefName = String("off") + indexString;

		_angleOffset = preferences.getFloat(_anglePrefName.c_str(), 0.0f);
		_senseOnLevel = preferences.getUInt(_onPrefName.c_str(), adcOnLevel);
		_senseOffLevel = preferences.getUInt(_offPrefName.c_str(), adcOffLevel);

		setLED(_breakerOn);
		setStatus(state);

		new Service::ContactSensor();
			_tripped = new Characteristic::ContactSensorState(trippedSensorFalse);

		SerPrintf("Breaker %d - servoPin: %d, pinButton: %d, pinSense: %d, ledPin: %d, angleOffset: %.1f, onLevel: %d, offLevel: %d\n",
						_statusIndex, servoPin, pinButton, _pinSense, ledPin, _angleOffset, _senseOnLevel, _senseOffLevel);

		centerServo();
	}

	void centerServo() {
		_servoMoveTime = millis64() - servoPushTimeMS;	// trigger setting servo to center position
	}

	BreakerState_t breakerState(uint16_t* value = NULL) {
		uint16_t adcVal = analogReadClean(_pinSense);

		if (value) {
			*value = adcVal;
		}

		if (adcVal >= adcOnLevel) {
			return breakerStateOn;
		}
		else if (adcVal <= adcOffLevel) {
			return breakerStateOff;
		}
		else {
			return breakerStateTripped;
		}
	}

	void setStatus(BreakerState_t state) {
		_statusStrip->setPixel(_statusIndex, state);
	}

	void updateLED() {
		_led->set(_ledState ? (_dark ? ledDarkBrightness : ledBrightBrightness) : 0);
	}

	void setDark(bool dark) {
		if (dark != _dark) {
			_dark = dark;
			updateLED();
		}
	}

	void setLED(bool on) {
		if (on != _ledState) {
			_ledState = on;
			updateLED();
		}
	}

	void setServoAngle(double angle, bool silent = false) {
		if (!isnan(angle)) {
			angle += _angleOffset;
		}
		if (_servo && angle != _servoAngle) {
			if (!silent) {
				if (isnan(angle)) {
					SerPrintf("%6lld: Breaker %d - servo disabled\n", millis64(), _statusIndex);
				}
				else {
					SerPrintf("%6lld: Breaker %d - set servo to %dº\n", millis64(), _statusIndex, (int)angle);
				}
			}
			_servoAngle = angle;
			_servo->set(angle);
		}
	}
	
	double getServoAngle() {
		return isnan(_servoAngle) ? servoDisable : _servoAngle - _angleOffset;
	}

	bool update() {
		if (_power->updated()) {
			bool newPower = _power->getNewVal();

			if (newPower != power()) {
				_changeToValue = newPower;
			}	
		}	
		return true;
	}	

	void loop() {
		uint64_t curTime = millis64();
		uint16_t adcVal = 0;
		BreakerState_t state = breakerState(&adcVal);
		bool curOnState = state == breakerStateOn;

		if (_servoMoveTime > 0) {
			if (curTime > _servoMoveTime + servoPushTimeMS * 2) {
				setServoAngle(servoDisable);
				_servoMoveTime = 0;
			}
			else if (curTime > _servoMoveTime + servoPushTimeMS) {
				if (getServoAngle() != servoCenterAngle) {
					setServoAngle(servoCenterAngle);
				}
			}
			else if (curTime > _servoMoveTime - servoPushTimeMS) {
				if (_breakerOn && getServoAngle() != servoMoveAngle) {
					setServoAngle(servoMoveAngle);
				}
			}
		}
		else if (_changeToValue != -1) {
			if (_changeToValue != _breakerOn) {
				_breakerOn = _changeToValue;
				if (state == breakerStateTripped) {
					setServoAngle(-servoMoveAngle);
					_servoMoveTime = curTime + servoPushTimeMS * 2;
				}
				else {
					setServoAngle(_changeToValue ? servoMoveAngle : -servoMoveAngle);
					_servoMoveTime = curTime;
				}
				_senseHoldTime = _servoMoveTime + senseChangeTimeMS;
			}
			_changeToValue = -1;
		}
		else if (_delayedChangeTo != -1) {
			if (curTime > _delayedTime) {
				_power->setVal(_delayedChangeTo);
				_changeToValue = _delayedChangeTo;
				_delayedChangeTo = -1;
			}
		}
		else if (curTime > _senseHoldTime) {
			int tripped = (state == breakerStateTripped) ? trippedSensorTrue : trippedSensorFalse;

			if (tripped != _tripped->getVal()) {
				_tripped->setVal(tripped);
				SerPrintf("%6lld: Breaker %d - tripped state changed to %s\n", curTime, _statusIndex, tripped == trippedSensorTrue ? "TRIPPED" : "NOT TRIPPED");
			}
			
			if (curOnState != _power->getVal()) {
				SerPrintf("%6lld: Breaker %d - state mismatch, setting to %d (state=%d, adc=%d, on=%d, off=%d)\n", curTime, _statusIndex, curOnState, state, adcVal, _senseOnLevel, _senseOffLevel);
				_breakerOn = curOnState;
				_power->setVal(curOnState);
			}
		}

		setLED(curOnState);
		setStatus(state);
	}

	void setPower(bool value, uint32_t afterDelayMS = 0) {
		if (value != power()) {
			SerPrintf("%6lld: Breaker %d - setPower to %d\n", millis64(), _statusIndex, value);
			if (_delayedChangeTo != -1) {
				_delayedChangeTo = -1;
			}
			else if (afterDelayMS) {
				_delayedChangeTo = value;
				_delayedTime = millis64() + afterDelayMS;
			}
			else {
				_power->setVal(value);
				_changeToValue = value;
			}
		}
	}

	bool power() {
		return (_delayedChangeTo != -1) ? _delayedChangeTo : _power->getVal();
	}

	void setBuddy(BreakerController* buddy) {
		_buddyBreaker = buddy;
	}

	void button(int pin, int pressType) override {
		if (pressType == SpanButton::SINGLE) {
			SerPrintf("%6lld: Breaker %d - Button Press - toggle power\n", millis64(), _statusIndex);
			setPower(!power());
		}
		else if (_buddyBreaker && pressType == SpanButton::LONG) {
			bool newState = !power();

			SerPrintf("%6lld: Breaker %d - Button Long Press - toggle power, local and buddy\n", millis64(), _statusIndex);
			setPower(newState);
			_buddyBreaker->setPower(newState, buddyDelayTimeMS);
		}
	}

	void calibrateCenter() {
		if (_servo) {
			bool changed = false;
			bool moved = true;
			bool done = false;

			setServoAngle(servoCenterAngle, true);

			SerPrintf("%6lld: Breaker %d - calibrate center\n", millis64(), _statusIndex);
			SerPrintf("Press 'u' for up and 'd' for down to adjust\n");
			SerPrintf("Press <return> to save, press <esc> to exit\n");
			while (!done) {
				if (moved) {
					printAndBackUp("Current angle offset: % 4dº", (int)_angleOffset);
					moved = false;
				}
				if (Serial.available()) {
					char c = Serial.read();
					if (c == '\n' || c == '\r') {
						Serial.print("\n");
						done = true;
					}
					else if (c == 27) { // ESC
						SerPrintf("\n\nCalibration cancelled.\n");
						done = true;
						changed = false;
					}
					else if (c == 'u' || c == 'U') { // up
						Serial.print('\b');
						_angleOffset += 1.0;
						moved = true;
					}
					else if (c == 'd' || c == 'D') { // down
						Serial.print('\b');
						_angleOffset -= 1.0;
						moved = true;
					}
				}
				if (moved) {
					setServoAngle(servoCenterAngle, true);
					changed = true;
				}
			}
			if (changed) {
				SerPrintf("\nCalibration complete. Saving new center angle: %2dº\n", (int)_angleOffset);
				preferences.putFloat(_anglePrefName.c_str(), _angleOffset);
			}
			else {
				SerPrintf("\nNo changes made to center angle.\n");
			}

			setServoAngle(servoDisable, true);
		}
	}

	void calibrateSenseLevels() {
		SerPrintf("%6lld: Breaker %d - calibrate sense levels\n", millis64(), _statusIndex);
		SerPrintf("Measuring On... %d\n", _senseOnLevel);

		uint16_t onValue, offValue;

		setServoAngle(servoMoveAngle, true);
		delay(senseChangeTimeMS);
		breakerState(&onValue);

		SerPrintf("Measuring Off... %d\n", _senseOffLevel);

		setServoAngle(-servoMoveAngle, true);
		delay(senseChangeTimeMS);
		breakerState(&offValue);

		setServoAngle(servoCenterAngle, true);
		delay(servoPushTimeMS);

		uint16_t newOnLevel = offValue + (onValue - offValue) * 3 / 4;
		uint16_t newOffLevel = offValue + (onValue - offValue) / 2;

		SerPrintf("\nNew On Level: %d, New Off Level: %d (on=%d, off=%d)\n", newOnLevel, newOffLevel, onValue, offValue);
		SerPrintf("Press 'Y' to accept these new levels, or any other key to reject.\n");

		if (waitForY()) {
			_senseOnLevel = newOnLevel;
			_senseOffLevel = newOffLevel;
			preferences.putUInt(_onPrefName.c_str(), _senseOnLevel);
			preferences.putUInt(_offPrefName.c_str(), _senseOffLevel);
			SerPrintf("\nNew sense levels ACCEPTED.\n");
		}
		else {
			SerPrintf("\nNew sense levels REJECTED.\n");
		}
		centerServo();
	}
};

//--------------------------------------------------------------------
BreakerController* washer;
BreakerController* dryer;
BreakerController* inverter;
BreakerController* converter;

struct PowerLevel : Service::LightBulb {
	SpanCharacteristic* _power;

	int _amps;

	PowerLevel(int amps) : Service::LightBulb() {
		_amps = amps;
		_power = new Characteristic::On(_amps == currentAmps());
	}

	int currentAmps() {
		int amps = -1;
		bool invPower = inverter->power();
		bool convPower = converter->power();

		if (invPower && convPower) { amps = 50; }
		else if (!invPower && convPower) { amps = 20; }
		else if (!invPower && !convPower) { amps = 15; }

		return amps;
	}

	void activate() {
		if (_amps != currentAmps()) {
			SerPrintf("PowerLevel: Activate - %dA\n", _amps);

			bool invPower = inverter->power();
			bool convPower = converter->power();

			if (_amps == 50) {
				converter->setPower(true);
				inverter->setPower(true, !convPower * breakerDelayTimeMS);
			}
			else if (_amps == 20) {
				inverter->setPower(false);
				converter->setPower(true, invPower * breakerDelayTimeMS);
			}
			else if (_amps == 15) {
				inverter->setPower(false);
				converter->setPower(false, invPower * breakerDelayTimeMS);
			}
		}
	}

	boolean update() {
		if (_power->getNewVal()) {
			activate();
		}
		return(true);
	}

	void loop() {
		bool on = (_amps == currentAmps());

		if (on != _power->getVal()) {
			_power->setVal(on);
		}
	}
};

void calibrateUpDown(BreakerController* breaker) {
	bool changed = false;
	bool moved = true;
	bool done = false;
	bool up = true;

	breaker->setServoAngle(up ? servoMoveAngle : -servoMoveAngle, true);

	SerPrintf("%6lld: Breaker %d - calibrate up/down angle\n", millis64(), breaker->_statusIndex);
	SerPrintf("Press 'u' for up and 'd' for down to adjust\n");
	SerPrintf("Press <space> to toggle direction\n");
	SerPrintf("Press <return> to save, press <esc> to exit\n");
	while (!done) {
		bool update = false;
		if (moved) {
			printAndBackUp("Current up/down angle: % 2dº", servoMoveAngle);
			moved = false;
		}
		if (Serial.available()) {
			char c = Serial.read();
			if (c == '\n' || c == '\r') {
				Serial.print("\n");
				done = true;
			}
			else if (c == 27) { // ESC
				SerPrintf("\n\nCalibration cancelled.\n");
				done = true;
				changed = false;
			}
			else if (c == 'u' || c == 'U') { // up
				Serial.print('\b');
				servoMoveAngle += 1.0;
				moved = true;
			}
			else if (c == 'd' || c == 'D') { // down
				Serial.print('\b');
				servoMoveAngle -= 1.0;
				moved = true;
			}
			else if (c == ' ') { // space
				up = !up;
				Serial.print('\b');
				update = true;
			}
		}
		if (moved) {
			changed = true;
		}
		if (update || moved) {
			breaker->setServoAngle(up ? servoMoveAngle : -servoMoveAngle, true);
		}
	}
	if (changed) {
		SerPrintf("Calibration complete. Saving new up/down angle: %dº\n", servoMoveAngle);
		preferences.putFloat(movePrefName, servoMoveAngle);
	}
	else {
		SerPrintf("No changes made to up/down angle.\n");
	}

	breaker->centerServo();
}

//--------------------------------------------------------------------

PowerLevel* fullPower;
PowerLevel* reducedPower;
PowerLevel* minimumPower;

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

		SPAN_ACCESSORY("Washer");
			washer = new BreakerController(pinServoWasher, true, pinButtonWasher, pinSenseWasher, pinLedWasher, &statusStrip, statusIndexWasher);

		SPAN_ACCESSORY("Dryer");
			dryer = new BreakerController(pinServoDryer, true, pinButtonDryer, pinSenseDryer, pinLedDryer, &statusStrip, statusIndexDryer);

		SPAN_ACCESSORY("Inverter");
			inverter = new BreakerController(pinServoInverter, true, pinButtonInverter, pinSenseInverter, pinLedInverter, &statusStrip, statusIndexInverter);

		SPAN_ACCESSORY("Converter");
			converter = new BreakerController(pinServoConverter, false, pinButtonConverter, pinSenseConverter, pinLedConverter, &statusStrip, statusIndexConverter);

		washer->setBuddy(dryer);
		dryer->setBuddy(washer);

		SPAN_ACCESSORY("Full Power");
			fullPower = new PowerLevel(50);

		SPAN_ACCESSORY("Reduced Power");
			reducedPower = new PowerLevel(20);

		SPAN_ACCESSORY("Minimum Power");
			minimumPower = new PowerLevel(15);
}

BreakerController* getBreakerFromCmd(const char *buf) {
	switch (buf[1] - '0') {
		case statusIndexWasher: return washer;
		case statusIndexDryer: return dryer;
		case statusIndexInverter: return inverter;
		case statusIndexConverter: return converter;
		default: {
			SerPrintf("Invalid index, must be 0-3\n");
			return NULL;
		}
	}
}

void cmdCalibrateCenter(const char *buf){
	BreakerController* breaker = getBreakerFromCmd(buf);
	if (breaker) {
		breaker->calibrateCenter();
	}
}

void cmdCalibrateUpDown(const char *buf){
	BreakerController* breaker = getBreakerFromCmd(buf);
	if (breaker) {
		calibrateUpDown(breaker);
	}
}

void cmdMeasureSenseLevels(const char *buf){
	BreakerController* breaker = getBreakerFromCmd(buf);
	if (breaker) {
		breaker->calibrateSenseLevels();
	}
}

void cmdSetBreakerPower(const char *buf){
	BreakerController* breaker = getBreakerFromCmd(buf);
	if (breaker) {
		if (buf[2] == '0') {
			SerPrintf("Setting Breaker %d to OFF\n", breaker->_statusIndex);
			breaker->setPower(false);
		}
		else if (buf[2] == '1') {
			SerPrintf("Setting Breaker %d to ON\n", breaker->_statusIndex);
			breaker->setPower(true);
		}
		else {
			SerPrintf("Invalid command, must be 2 digits, 0-3 followed by 0 or 1\n");
		}
	}
}

void cmdSetPowerLevel(const char *buf){
	if (strcasecmp(buf+1, "min") == 0) {
		SerPrintf("Setting power level to MINIMUM\n");
		minimumPower->activate();
	}
	else if (strcasecmp(buf+1, "low") == 0) {
		SerPrintf("Setting power level to LOW\n");
		reducedPower->activate();
	}
	else if (strcasecmp(buf+1, "full") == 0) {
		SerPrintf("Setting power level to FULL\n");
		fullPower->activate();
	}
	else {
		SerPrintf("Invalid power level. Use 'min', 'low', or 'full'.\n");
	}
}

void cmdClearPreferences(const char *buf) {
	SerPrintf("%6lld: Clear preferences\n", millis64());
	SerPrintf("This will remove all saved settings, including breaker angles and sense levels.\n");
	SerPrintf("Press 'Y' to confirm and restart, or any other key to cancel.\n");

	if (waitForY()) {
		preferences.clear();
		preferences.end();
		SerPrintf("Preferences cleared. Restarting...\n");
		ESP.restart();
	}
	else {
		SerPrintf("Clear preferences cancelled.\n");
	}
}

void cmdUpdateAccessories(const char *buf){

	if(homeSpan.updateDatabase()) {
		SerPrintf("Accessories Database updated.  New configuration number broadcast...\n");
	}
	else {
		SerPrintf("Nothing to update - no changes were made!\n");
	}
}

void cmdShowCPUStats(const char *buff = nullptr) {
	SerPrintf("\n*** CPU Stats ***\n\n");

	SerPrintf("CPU frequency: %ldMHz\n", getCpuFrequencyMhz());
	SerPrintf("Xtal frequency: %ldMHz\n", getXtalFrequencyMhz());
	SerPrintf("Bus frequency: %ldMHz\n", getApbFrequency()/1000000);
	SerPrintf("Total heap: %ld\n", ESP.getHeapSize());
	SerPrintf("Free heap: %ld\n", ESP.getFreeHeap());
	SerPrintf("Total PSRAM: %ld\n", ESP.getPsramSize());
	SerPrintf("Free PSRAM: %ld\n", ESP.getFreePsram());

	SerPrintf("\n*** CPU Stats ***\n\n");
}

bool adcStatusEnabled = false;

void cmdToggleADCStatus(const char *buf) {
	adcStatusEnabled = !adcStatusEnabled;
	SerPrintf("ADC Status: %s\n", adcStatusEnabled ? "ON" : "OFF");
}

void addCommands() {
	new SpanUserCommand('s',"show CPU stats", cmdShowCPUStats);
	new SpanUserCommand('u',"update accessory database", cmdUpdateAccessories);
	new SpanUserCommand('a',"toggle ADC status", cmdToggleADCStatus);
	new SpanUserCommand('p',"set breaker N to 0=off, 1=on)", cmdSetBreakerPower);
	new SpanUserCommand('w',"set power level - 'min', 'low', 'full'", cmdSetPowerLevel);
	new SpanUserCommand('x',"clear preferences", cmdClearPreferences);
	new SpanUserCommand('l',"measure sense levels for breaker N", cmdMeasureSenseLevels);
	new SpanUserCommand('m',"calibrate servo up/down move for breaker N", cmdCalibrateUpDown);
	new SpanUserCommand('c',"calibrate servo center for breaker N", cmdCalibrateCenter);
}

//--------------------------------------------------------------------

void flashIndicator(uint32_t color, uint16_t count, uint16_t period) {
	for (auto _=0; _<count; _++) {
		setIndicator(color);
		delay(period/4);
		setIndicator(0);
		delay(period*3/4);
	}
}

void statusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		setIndicator(connectingColor);
		currentIndicatorColor = connectingColor;
		SerPrintf("Lost WIFI Connection...\n");
	}
}

void wifiReady(int ready) {
	setIndicator(readyColor);
	SerPrintf("WIFI: Ready.\n");
}

TaskHandle_t t;

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
	enablePower();
	statusStrip.begin(statusStripPin, statusStripLength);

	flashIndicator(flashColor, 20, 100);
	setIndicator(startColor);

	Serial.begin(115200);
	SerPrintf("Power Center Controller Startup\n");

	SerPrintf("Setup status LED strip\n");
	statusStrip.setBrightness(statusStripBrightBrightness, false);

	for (auto i=-1; i<=statusStripLength; i++) {
		statusStrip.setPixel(i-1, breakerStateUnknown, false);
		statusStrip.setPixel(i+1, breakerStateOff, false);
		statusStrip.setPixel(i, breakerStateOn, false);
		statusStrip.showPixels();
		delay(100);
	}

	SerPrintf("Initialize Preferences\n");
	preferences.begin(prefsPartitionName, false);
	servoMoveAngle = preferences.getFloat(movePrefName, initialServoMoveAngle);
	SerPrintf("Servo move angle: %d\n", servoMoveAngle);

	SerPrintf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setConnectionCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.begin(Category::Bridges, displayName, DEFAULT_HOST_NAME, modelName);

	SerPrintf("Create devices\n");
	createDevices();
	addCommands();

	SerPrintf("Wait for WiFi...\n");
	setIndicator(connectingColor);

	SerPrintf("Init complete.\n");
}

void checkAmbientLight() {
	static uint64_t ambientCheckTime = 0;
	uint64_t curTime = millis64();

	if (curTime >= ambientCheckTime) {
		ambientCheckTime = curTime + lightCheckRateMS;
		uint16_t ambientLight = analogReadClean(ambientLightPin);
		// SerPrintf("Ambient Light: %d\n", ambientLight);
		if (ambientLight <= darkLowThreshold) {
			statusStrip.setBrightness(statusStripDarkBrightness);
		}
		else if (ambientLight >= darkHighThreshold) {
			statusStrip.setBrightness(statusStripBrightBrightness);
		}
	}
}

void showADCStatus() {
	if (adcStatusEnabled) {
		static uint64_t nextStatusTime = 0;

		uint64_t curTime = millis64();
		if (curTime >= nextStatusTime) {
			nextStatusTime = curTime + statusUpdateRateMS;

			uint16_t adcWasher = analogReadClean(pinSenseWasher);
			uint16_t adcDryer = analogReadClean(pinSenseDryer);
			uint16_t adcInverter = analogReadClean(pinSenseInverter);
			uint16_t adcConverter = analogReadClean(pinSenseConverter);

			SerPrintf("%6lld: ADC Washer: %4d, Dryer: %4d, Inverter: %4d, Converter: %4d\n", curTime, adcWasher, adcDryer, adcInverter, adcConverter);
		}
	}
}

void loop() {
	homeSpan.poll();

	checkAmbientLight();
	showADCStatus();
}

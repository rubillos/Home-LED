#include "Arduino.h"

#include "HomeSpan.h" 
#include <Preferences.h>

//--------------------------------------------------------------------

#if defined(OTADEBUG) || defined(DEBUG)
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

constexpr uint8_t servoPinWasher = MOSI; // 35
constexpr uint8_t servoPinDryer = SCK;	//36
constexpr uint8_t servoPinInverter = 5;
constexpr uint8_t servoPinConverter = 6;

constexpr uint8_t sensePinWasher = 17;
constexpr uint8_t sensePinDryer = 18;
constexpr uint8_t sensePinInverter = 14;
constexpr uint8_t sensePinConverter = 12;

constexpr uint8_t ledPinWasher = TX;	// 43
constexpr uint8_t ledPinDryer = SDA;	// 8
constexpr uint8_t ledPinInverter = MISO;	// 37
constexpr uint8_t ledPinConverter = SCL;	// 9

constexpr uint8_t buttonPinWasher = 7;
constexpr uint8_t buttonPinDryer = 3;
constexpr uint8_t buttonPinInverter = 1;
constexpr uint8_t buttonPinConverter = 38;

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
constexpr uint32_t servoTotalTimeMS = servoPushTimeMS * 2;

constexpr uint32_t senseChangeTimeMS = servoPushTimeMS + 50;
constexpr uint32_t breakerDelayTimeMS = servoPushTimeMS + 100;
constexpr uint32_t buddyDelayTimeMS = 0;

constexpr uint8_t ledDarkBrightness = 10;

constexpr uint8_t statusStripDarkBrightness = 1;
constexpr uint8_t statusStripBrightBrightness = 20;

constexpr uint16_t darkLowThreshold = 700;
constexpr uint16_t darkHighThreshold = 1000;

constexpr uint32_t lightCheckTimeMS = 1000;

//--------------------------------------------------------------------

Pixel indicator(indicatorDataPin, "GRB");

//--------------------------------------------------------------------

Preferences preferences;

const char* movePrefName = "moveAngle";
constexpr int16_t initialServoMoveAngle = 20;
int16_t servoMoveAngle = initialServoMoveAngle;

//--------------------------------------------------------------------

constexpr const char* versionString = "v0.5";

//--------------------------------------------------------------------

constexpr float ledGamma = 2.8;
constexpr uint8_t maxIndicatorValue = 20;

constexpr uint8_t ledStatusLevel = 0x10;

#define MAKE_RGB(r, g, b) (r<<16 | g<<8 | b)
constexpr uint32_t flashColor = MAKE_RGB(ledStatusLevel, 0, ledStatusLevel);
constexpr uint32_t startColor = MAKE_RGB(ledStatusLevel, 0, 0);
constexpr uint32_t connectingColor = MAKE_RGB(0, 0, ledStatusLevel);
constexpr uint32_t readyColor = MAKE_RGB(0, ledStatusLevel, 0);

constexpr uint8_t v255 = 0xFF;
constexpr uint32_t unknownColor = 0xFFFFFFFE;

uint32_t currentIndicatorColor = unknownColor;
uint32_t savedIndicatorColor = 0;

constexpr uint32_t whiteColorFlag = 1 << 24;

//--------------------------------------------------------------------

constexpr const char* displayName = "PwrCenter-Controller";
constexpr const char* modelName = "PwrCenter-Controller-ESP32-S3";

//--------------------------------------------------------------------

uint64_t millis64() {
	volatile static uint32_t low32 = 0, high32 = 0;
	uint32_t new_low32 = millis();

	if (new_low32 < low32)
		high32++;

	low32 = new_low32;

	return (uint64_t) high32 << 32 | low32;
}

volatile uint64_t lastIndicatorChange;

uint8_t pixelBrightness = 32;

inline uint8_t mul8x8(uint8_t a, uint8_t b) { return (a * b) / 255; }
inline uint8_t pixBright(uint8_t value) { return mul8x8(value, pixelBrightness); }

void setupIndicator() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);
}

void setIndicatorBrightness(uint8_t brightness) {
	pixelBrightness = brightness;
}

void indicatorSetColor(uint32_t color) {
	indicator.set(Pixel::RGB(pixBright(color>>16), pixBright((color>>8)&v255), pixBright(color&v255)), 1);
}

void setIndicator(uint32_t color, bool saveColor = false, bool updateChanged = true) {
	if (color != currentIndicatorColor) {
		currentIndicatorColor = color;
		if (saveColor) {
			savedIndicatorColor = color;
		}
		if (color & whiteColorFlag) {
			color &= v255;
			indicatorSetColor(color | (color << 16) | (color << 8));
		}
		else {
			indicatorSetColor(color);
		}

		if (updateChanged) {
			lastIndicatorChange = millis64();
		}
	}
}

void setIndicator8(uint8_t color8) {
	setIndicator(whiteColorFlag | color8);
}

//--------------------------------------------------------------------
typedef enum {
	breakerStateOff = 0,
	breakerStateOn = 1,
	breakerStateTripped = -1,
	breakerStateUnknown = 2
} BreakerState_t;

struct StatusStrip {
	Pixel *_strip;
	uint16_t _length;
	uint8_t _brightness = 255;
	BreakerState_t* _status;
	Pixel::Color* _colors;

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

	void setup(uint8_t pin, uint16_t length) {
		_brightness = 255;
		_length = length;
		_strip = new Pixel(pin, "RGB");
		_status = new BreakerState_t[_length];
		_colors = new Pixel::Color[_length];
		for (auto i=0; i<_length; i++) {
			_status[i] = breakerStateUnknown;
		}
		showPixels();
	}
};

StatusStrip statusStrip;

struct BreakerController : Service::Outlet {
	SpanCharacteristic* _power;

	ServoPin* _servo = NULL;
	double _servoAngle = servoDisable;
	double _angleOffset = 0;

	BreakerController* _buddyBreaker = NULL;
	StatusStrip* _statusStrip = NULL;

	int _sensePin;
	uint16_t _senseOffLevel;
	uint16_t _senseOnLevel;
	int _statusIndex;

	LedPin* _led = NULL;
	int _ledState = -1;
	bool _dark = false;

	int _homekitChangeTo = -1;

	int _delayedChangeTo = -1;
	uint64_t _delayTime = 0;

	uint64_t _senseHoldTime = 0;

	bool _breakerOn;

	uint64_t _moveTime = 0;

	String _anglePrefName;
	String _onPrefName;
	String _offPrefName;

	BreakerController(int servoPin, bool reverseServo, int buttonPin, int sensePin, int ledPin, StatusStrip* statusStrip, int statusIndex) : Service::Outlet(){

		new SpanButton(buttonPin, 1000, 5, 0, SpanButton::TRIGGER_ON_LOW);

		_servo = new ServoPin(servoPin, servoDisable, servoMinUSec, servoMaxUSec, (reverseServo) ? servoMinAngle : servoMaxAngle, (reverseServo) ? servoMaxAngle : servoMinAngle);

		_sensePin = sensePin;

		_led = new LedPin(ledPin, 0);

		_statusStrip = statusStrip;
		_statusIndex = statusIndex;

		_anglePrefName = String("aOffset") + String(_statusIndex);
		_angleOffset = preferences.getFloat(_anglePrefName.c_str(), 0.0f);

		_onPrefName = String("on") + String(_statusIndex);
		_senseOnLevel = preferences.getUInt(_onPrefName.c_str(), adcOnLevel);
		_offPrefName = String("off") + String(_statusIndex);
		_senseOffLevel = preferences.getUInt(_offPrefName.c_str(), adcOffLevel);

		BreakerState_t state = breakerState();
		_breakerOn = state == breakerStateOn;
		_power = new Characteristic::On(_breakerOn);

		setLED(_breakerOn);
		setStatus(state);

		new Characteristic::OutletInUse(true);

		SerPrintf("Breaker %d - servoPin: %d, buttonPin: %d, sensePin: %d, ledPin: %d, angleOffset: %.1f, onLevel: %d, offLevel: %d\n",
						_statusIndex, servoPin, buttonPin, _sensePin, ledPin, _angleOffset, _senseOnLevel, _senseOffLevel);

		centerServo();
	}

	void centerServo() {
		_moveTime = millis64() - servoPushTimeMS;	// trigger setting servo to center position
	}

	BreakerState_t breakerState(uint16_t* value = NULL) {
		uint16_t adcVal = analogRead(_sensePin);

		adcVal = analogRead(_sensePin);
		adcVal = analogRead(_sensePin);

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
		_led->set(_ledState ? ((_dark)? ledDarkBrightness : 100) : 0);
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

	bool update() {
		if (_power->updated()) {
			bool newPower = _power->getNewVal();

			if (newPower != _breakerOn) {
				_homekitChangeTo = newPower;
			}
		}
		return true;
	}

	void setServoAngle(double angle, bool silent = false) {
		if (angle != servoDisable) {
			angle += _angleOffset;
		}
		if (_servo && angle != _servoAngle) {
			if (!silent) {
				SerPrintf("%6lld: Breaker %d - set servo to %2.1fº\n", millis64(), _statusIndex, angle);
			}
			_servoAngle = angle;
			_servo->set(angle);
		}
	}

	double getServoAngle() {
		if (_servoAngle != servoDisable) {
			return _servoAngle - _angleOffset;
		}
		return servoDisable;
	}

	void loop() {
		uint64_t curTime = millis64();
		uint16_t adcVal = 0;
		BreakerState_t state = breakerState(&adcVal);
		bool curOnState = state == breakerStateOn;

		if (_moveTime > 0) {
			if (curTime >= _moveTime + servoTotalTimeMS) {
				SerPrintf("%6lld: Breaker %d - disable\n", curTime, _statusIndex);
				setServoAngle(servoDisable);
				_moveTime = 0;
			}
			else if (curTime >= _moveTime + servoPushTimeMS && getServoAngle() != servoCenterAngle) {
				SerPrintf("%6lld: Breaker %d - center\n", curTime, _statusIndex);
				setServoAngle(servoCenterAngle);
			}
		}
		else if (_homekitChangeTo != -1) {
			if (_homekitChangeTo != _breakerOn) {
				int servoAngle = (_homekitChangeTo) ? servoMoveAngle : -servoMoveAngle;
				SerPrintf("%6lld: Breaker %d - set to %d\n", curTime, _statusIndex, servoAngle);
				setServoAngle(servoAngle);
				_moveTime = curTime;
				_breakerOn = _homekitChangeTo;
			}
			_homekitChangeTo = -1;
		}
		else if (_delayedChangeTo != -1) {
			if (curTime > _delayTime) {
				_power->setVal(_delayedChangeTo);
				_homekitChangeTo = _delayedChangeTo;
				_delayedChangeTo = -1;
			}
		}
		else if (curTime > _senseHoldTime && curOnState != _power->getVal()) {
			SerPrintf("%6lld: Breaker %d - state mismatch, setting to %d (state= %d, adc= %d)\n", curTime, _statusIndex, curOnState, state, adcVal);
			_breakerOn = curOnState;
			_senseHoldTime = curTime + senseChangeTimeMS;
			_power->setVal(curOnState);
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
				_delayTime = millis64() + afterDelayMS;
			}
			else {
				_power->setVal(value);
				_homekitChangeTo = value;
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
			SerPrintf("%6lld: Breaker %d - Button Press\n", millis64(), _statusIndex);
			setPower(1 - power());
		}
		else if (_buddyBreaker && pressType == SpanButton::LONG) {
			bool newState = 1 - power();

			SerPrintf("%6lld: Breaker %d - Button Long Press\n", millis64(), _statusIndex);
			setPower(newState);
			_buddyBreaker->setPower(newState, buddyDelayTimeMS);
		}
	}

	void calibrateCenter() {
		if (_servo) {
			bool changed = false;
			bool moved = true;
			bool done = false;
			const char* offsetMsg = "Current angle offset: ";
			String backspaces;
			for (size_t i = 0; i < strlen(offsetMsg) + 7; ++i) {
				backspaces += '\b';
			}

			setServoAngle(servoCenterAngle, true);

			SerPrintf("%6lld: Breaker %d - calibrate center\n", millis64(), _statusIndex);
			SerPrintf("Use 'u' for up and 'd' for down to adjust the center position\n");
			SerPrintf("Press <return> to save, press <esc> to exit\n");
			while (!done) {
				if (moved) {
					SerPrintf("%s% 4.1fº", offsetMsg, _angleOffset);
					Serial.print(backspaces);
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
					else if (c == 'u') { // up
						Serial.print('\b');
						_angleOffset += 1.0;
						moved = true;
					}
					else if (c == 'd') { // down
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
				SerPrintf("Calibration complete. Saving new center angle: %2.1fº\n", _angleOffset);
				preferences.putFloat(_anglePrefName.c_str(), _angleOffset);
			}
			else {
				SerPrintf("No changes made to center angle.\n");
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

		bool done = false;
		while (!done) {
			if (Serial.available()) {
				char c = Serial.read();
				if (c == 'Y' || c == 'y') {
					_senseOnLevel = newOnLevel;
					_senseOffLevel = newOffLevel;
					preferences.putUInt(_onPrefName.c_str(), _senseOnLevel);
					preferences.putUInt(_offPrefName.c_str(), _senseOffLevel);
					SerPrintf("\nNew sense levels accepted.\n");
					done = true;
				}
				else {
					SerPrintf("\nNew sense levels rejected.\n");
					done = true;
				}
			}
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
			SerPrintf("Activate - %dA\n", _amps);

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
		if (_power->getNewVal() && (_amps != currentAmps())) {
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
	const char* offsetMsg = "Current up/down angle: ";
	String backspaces;
	for (size_t i = 0; i < strlen(offsetMsg) + 7; ++i) {
		backspaces += '\b';
	}

	breaker->setServoAngle(up ? servoMoveAngle : -servoMoveAngle, true);

	SerPrintf("%6lld: Breaker %d - calibrate up/down angle\n", millis64(), breaker->_statusIndex);
	SerPrintf("Use 'u' for up and 'd' for down to adjust the current move amount\n");
	SerPrintf("Press <space> to toggle direction\n");
	SerPrintf("Press <return> to save, press <esc> to exit\n");
	while (!done) {
		bool update = false;
		if (moved) {
			SerPrintf("%s% 2dº", offsetMsg, servoMoveAngle);
			Serial.print(backspaces);
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
			else if (c == 'u') { // up
				Serial.print('\b');
				servoMoveAngle += 1.0;
				moved = true;
			}
			else if (c == 'd') { // down
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
			update = true;
		}
		if (update) {
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
			washer = new BreakerController(servoPinWasher, true, buttonPinWasher, sensePinWasher, ledPinWasher, &statusStrip, statusIndexWasher);

		SPAN_ACCESSORY("Dryer");
			dryer = new BreakerController(servoPinDryer, true, buttonPinDryer, sensePinDryer, ledPinDryer, &statusStrip, statusIndexDryer);

		SPAN_ACCESSORY("Inverter");
			inverter = new BreakerController(servoPinInverter, true, buttonPinInverter, sensePinInverter, ledPinInverter, &statusStrip, statusIndexInverter);

		SPAN_ACCESSORY("Converter");
			converter = new BreakerController(servoPinConverter, false, buttonPinConverter, sensePinConverter, ledPinConverter, &statusStrip, statusIndexConverter);

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

void cmdCalibrateSenseLevels(const char *buf){
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
			SerPrintf("Invalid command, must be '0' or '1'\n");
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
		SerPrintf("Invalid power level. Use 'min', 'low', 'full'.\n");
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

void addCommands() {
	new SpanUserCommand('s',"show CPU stats", cmdShowCPUStats);
	new SpanUserCommand('u',"update accessory database", cmdUpdateAccessories);
	new SpanUserCommand('p',"set power for breaker N - 0=off, 1=on)", cmdSetBreakerPower);
	new SpanUserCommand('w',"set power level - 'min', 'low', 'full'", cmdSetPowerLevel);
	new SpanUserCommand('l',"calibrate sense levels for breaker N", cmdCalibrateSenseLevels);
	new SpanUserCommand('m',"calibrate servo up/down move for breaker N", cmdCalibrateUpDown);
	new SpanUserCommand('c',"calibrate servo center for breaker N", cmdCalibrateCenter);
}

//--------------------------------------------------------------------

void flashIndicator(uint32_t color, uint16_t count, uint16_t period) {
	for (auto i=0; i<count; i++) {
		setIndicator(color);
		delay(period/4);
		setIndicator(0);
		delay(period*3/4);
	}
}

void statusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		if (!(currentIndicatorColor & whiteColorFlag)) {
			setIndicator(connectingColor, true);
		}
		currentIndicatorColor = connectingColor;
		SerPrintf("Lost WIFI Connection...\n");
	}
}

void wifiReady(int ready) {
	if (!(currentIndicatorColor & whiteColorFlag)) {
		setIndicator(readyColor, true);
	}
	SerPrintf("WIFI: Ready.\n");
}

TaskHandle_t t;

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
	setupIndicator();
	statusStrip.setup(statusStripPin, statusStripLength);

	flashIndicator(flashColor, 20, 100);
	setIndicator(startColor, true);

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

	SerPrintf("Initialize Preferences System\n");
	preferences.begin("PwrPrefs", false);
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
	setIndicator(connectingColor, true);

	SerPrintf("Init complete.\n");
}

void checkAmbientLight() {
	static uint64_t ambientCheckTime = 0;
	uint64_t curTime = millis64();
	if (curTime >= ambientCheckTime) {
		ambientCheckTime = curTime + lightCheckTimeMS;
		analogRead(ambientLightPin);
		analogRead(ambientLightPin);
		uint16_t ambientLight = analogRead(ambientLightPin);
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
	static uint64_t nextStatusTime = 0;

	uint64_t curTime = millis64();
	if (curTime >= nextStatusTime) {
		nextStatusTime = curTime + 1000;

		uint16_t adcWasher = analogRead(sensePinWasher);
		uint16_t adcDryer = analogRead(sensePinDryer);
		uint16_t adcInverter = analogRead(sensePinInverter);
		uint16_t adcConverter = analogRead(sensePinConverter);

		SerPrintf("%6lu: ADC Washer: %4d, Dryer: %4d, Inverter: %4d, Converter: %4d\n", millis(), adcWasher, adcDryer, adcInverter, adcConverter);
	}
}

void loop() {
	homeSpan.poll();

	checkAmbientLight();
	showADCStatus();
}

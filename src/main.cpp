#include "Arduino.h"

#include "HomeSpan.h" 
#include <Preferences.h>
#include "Pixel2.h"
#include "BlinkableLEDPin.h"

//--------------------------------------------------------------------

#if defined(DEBUG)
#define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//--------------------------------------------------------------------

constexpr const char* displayName = "PwrCenter-Controller";
constexpr const char* modelName = "PwrCenter-Controller-ESP32-S3";
constexpr const char* versionString = "v1.0";

//--------------------------------------------------------------------

constexpr uint8_t statusStripPin = 33;
constexpr uint8_t statusStripLength = 4;

#define STATUS_STRIP_COLOR_ORDER "RGB"
#define STATUS_STRIP_TIMING 0.35, 0.8, 0.7, 0.6, 80.0

//--------------------------------------------------------------------

constexpr uint8_t indicatorDataPin = RGB_DATA;	// 40
constexpr uint8_t indicatorPowerPin = RGB_PWR;	// 39

#define INDICATOR_COLOR_ORDER "GRB"

//--------------------------------------------------------------------

constexpr uint8_t ambientLightPin = 4;
constexpr uint16_t ambientLightReadCount = 3;

//--------------------------------------------------------------------

constexpr uint16_t analogReadStableCount = 5;
constexpr uint16_t analogReadCountLimit = 30;
constexpr uint16_t analogReadMaxVariation = 50;

//--------------------------------------------------------------------
// Breaker Pins

constexpr uint8_t pinServoWasher = MOSI; // 35
constexpr uint8_t pinServoDryer = SCK;	//36
constexpr uint8_t pinServoInverter = 5;
constexpr uint8_t pinServoConverter = 6;

constexpr uint8_t pinSenseWasher = 17;
constexpr uint8_t pinSenseDryer = 18;
constexpr uint8_t pinSenseInverter = 14;
constexpr uint8_t pinSenseConverter = 12;

constexpr uint8_t pinLedWasher = SDA;	// 8
constexpr uint8_t pinLedDryer = TX;	// 43
constexpr uint8_t pinLedInverter = MISO;	// 37
constexpr uint8_t pinLedConverter = SCL;	// 9

constexpr uint8_t pinButtonWasher = 3;
constexpr uint8_t pinButtonDryer = 7;
constexpr uint8_t pinButtonInverter = 1;
constexpr uint8_t pinButtonConverter = 38;

constexpr uint8_t statusIndexWasher = 1;
constexpr uint8_t statusIndexDryer = 0;
constexpr uint8_t statusIndexInverter = 2;
constexpr uint8_t statusIndexConverter = 3;

//--------------------------------------------------------------------
constexpr uint16_t ADC_Max = 4095;     // This is the default ADC max value on the ESP32 (12 bit ADC width);

constexpr uint16_t adcInitialOnLevel = 2400; // ADC value above which the breaker is considered ON
constexpr uint16_t adcInitialOffLevel = 2200; // ADC value above which the breaker is considered OFF

constexpr uint16_t hysterisisRange = 100; // ADC value range for hysterisis to avoid flickering

constexpr uint16_t tripUpperPercentage = 80;
constexpr uint16_t tripLowerPercentage = 20;

constexpr uint16_t servoMinUSec = 500;
constexpr uint16_t servoMaxUSec = 2500;

constexpr int16_t servoDisable = 10000;
constexpr int16_t servoMinAngle = -90;
constexpr int16_t servoMaxAngle = 90;

constexpr int16_t servoInitialMoveAngle = 5;

constexpr uint32_t servoPushTimeMS = 250;
constexpr uint32_t servoResetTimeMS = 100;

constexpr uint32_t senseChangeTimeMS = servoPushTimeMS + 50;
constexpr uint32_t breakerDelayTimeMS = servoPushTimeMS + 300;
constexpr uint32_t buddyDelayTimeMS = 0;

constexpr uint32_t trippedStableDurationMS = 300;

constexpr uint8_t ledDarkBrightness = 10;
constexpr uint8_t ledBrightBrightness = 100;

constexpr uint16_t ledTrippedRateMS = 500;
constexpr float ledTrippedPeriod = 0.4;

constexpr uint8_t statusStripDarkBrightness = 1;
constexpr uint8_t statusStripBrightBrightness = 20;

constexpr uint16_t darkLowThreshold = 700;
constexpr uint16_t darkHighThreshold = 1000;

constexpr uint32_t lightCheckRateMS = 300;

constexpr uint32_t statusUpdateRateMS = 1000;

//--------------------------------------------------------------------

Pixel2 indicator(indicatorDataPin, INDICATOR_COLOR_ORDER);
uint8_t pixelBrightness = 32;

//--------------------------------------------------------------------

Preferences preferences;

const char* prefsPartitionName = "PwrPrefs";

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

uint16_t analogReadClean(uint8_t pin, uint16_t stableReads = analogReadStableCount) {
	uint16_t stableCount = 0, readCount = 0, maxStableCount = 0;
	uint16_t adcValue = 0, minValue = 0, maxValue = 0;

	while (stableCount < stableReads && readCount < analogReadCountLimit) {
		adcValue = analogRead(pin);
		readCount++;

		if (stableCount == 0) {
			minValue = adcValue;
			maxValue = adcValue;
			stableCount++;
		}
		else {
			minValue = min(minValue, adcValue);
			maxValue = max(maxValue, adcValue);

			if (maxValue - minValue > analogReadMaxVariation) {
				stableCount = 0;
			}
			else {
				stableCount++;
			}
		}
		maxStableCount = max(maxStableCount, stableCount);
	}

	if (readCount == analogReadCountLimit) {
		SerPrintf("Hit analogRead limit! Longest stable run: %d, diff=%d\n", maxStableCount, maxValue - minValue);
	}

	return adcValue;
}

//--------------------------------------------------------------------

void enablePower() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);
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

void printAndBackUp(const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
 	size_t n = Serial.vprintf(fmt, args);
	va_end(args);

	for (auto _=0; _< n; _++) {
		Serial.print('\b');
	}
}

//--------------------------------------------------------------------
bool waitForY() {
	while (true) {
		if (Serial.available()) {
			char c = Serial.read();
			if (c == 'Y' || c == 'y') {
				Serial.print('\b');
				return true;
			}
			else {
				Serial.print('\n');
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
	bool _reversed;
	bool _dark;
	BreakerState_t* _status = NULL;
	Pixel::Color* _colors = NULL;

	inline uint8_t brightness() {
		return _dark ? statusStripDarkBrightness : statusStripBrightBrightness;
	}

	void setDark(bool dark, bool show = true) {
		if (dark != _dark) {
			_dark = dark;
			if (show) {
				showPixels();
			}
		}
	}

	Pixel::Color colorRGB(uint8_t red, uint8_t green, uint8_t blue) {
		uint8_t bright = brightness();
		return Pixel::RGB(mul8x8(red, bright), mul8x8(green, bright), mul8x8(blue, bright));
	}

	Pixel::Color colorForState(BreakerState_t state) {
		switch (state) {
			case breakerStateOff: 		return colorRGB(255, 0, 0); 		// offColor
			case breakerStateOn: 		return colorRGB(0, 255, 0); 		// onColor
			case breakerStateTripped: 	return colorRGB(255, 255, 0); 	// trippedColor
			default: 					return colorRGB(0, 0, 0); 		// blackColor
		}
	}

	void showPixels() {
		if (_strip == NULL || _status == NULL || _colors == NULL) return;

		for (auto i = 0; i < _length; i++) {
			_colors[i] = colorForState(_status[i]);
		}
		_strip->set(_colors, _length);
	}

	void setPixel(uint8_t index, BreakerState_t state, bool show = true) {
		if (_status == NULL) return;
		
		if (index < _length) {
			if (_reversed) {
				index = _length - 1 - index;
			}
			if (state != _status[index]) {
				_status[index] = state;
				if (show) {
					showPixels();
				}
			}
		}
	}

	void clear(bool show = true) {
		if (_status == NULL) return;

		for (auto i = 0; i < _length; i++) {
			_status[i] = breakerStateUnknown;
		}
		if (show) {
			showPixels();
		}
	}

	void blank() {
		if (_strip == NULL) return;

		Pixel::Color blankColor = Pixel::RGB(0, 0, 0);
		_strip->set(blankColor, _length);
	}

	void begin(uint8_t pin, uint16_t length, bool reversed) {
		_dark = true;
		_reversed = reversed;
		_length = length;
		_strip = new Pixel2(pin, STATUS_STRIP_COLOR_ORDER);
		_strip->setTiming(STATUS_STRIP_TIMING);
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
	const char* _name;

	SpanCharacteristic* _power;
	SpanCharacteristic* _tripped;

	int _pinButton;

	ServoPin* _servo = NULL;
	int _pinServo;
	int16_t _servoAngle = servoDisable;
	int16_t _centerAngle = 0;
	int16_t _upAngle = servoInitialMoveAngle;
	int16_t _downAngle = -servoInitialMoveAngle;

	BreakerController* _buddyBreaker = NULL;

	StatusStrip* _statusStrip = NULL;
	int _statusIndex;

	uint8_t _pinSense;
	uint16_t _senseOffLevel;
	uint16_t _senseOnLevel;
	int16_t _senseHysterisisValue = -1;
	bool _lastTripped = false;
	uint64_t _lastTrippedTime = 0;

	BlinkableLEDPin* _led = NULL;
	Blinker* _blinker = NULL;
	int _pinLED;
	BreakerState_t _ledState = breakerStateUnknown;
	bool _dark = false;

	bool _breakerOn;

	int _changeToValue = -1;

	int _delayedChangeTo = -1;
	uint64_t _delayedTime = 0;

	uint64_t _servoMoveTime = 0;
	uint64_t _senseHoldTime = 0;

	String _centerPrefName;
	String _upPrefName;
	String _downPrefName;
	String _onPrefName;
	String _offPrefName;
	String _trippedName;

	BreakerController(int pinServo, bool reverseServo, int pinButton, int pinSense, int pinLED, StatusStrip* statusStrip, int statusIndex, const char* name) : Service::Outlet(){
		_name = name;

		new Characteristic::OutletInUse(true);

		_pinButton = pinButton;
		new SpanButton(pinButton, 1000, 5, 0, SpanButton::TRIGGER_ON_LOW);

		_pinServo = pinServo;
		_servo = new ServoPin(pinServo, NAN, servoMinUSec, servoMaxUSec, reverseServo ? servoMinAngle : servoMaxAngle, reverseServo ? servoMaxAngle : servoMinAngle);

		_pinLED = pinLED;
		_led = new BlinkableLEDPin(pinLED, 0);
		_blinker = new Blinker(_led);
		setDark(_dark, true);

		_statusStrip = statusStrip;
		_statusIndex = statusIndex;

		String indexString = String(_statusIndex);
		_centerPrefName = String("angCent") + indexString;
		_upPrefName = String("angUp") + indexString;
		_downPrefName = String("angDown") + indexString;
		_onPrefName = String("on") + indexString;
		_offPrefName = String("off") + indexString;

		_centerAngle = preferences.getInt(_centerPrefName.c_str(), 0);
		_upAngle = preferences.getInt(_upPrefName.c_str(), servoInitialMoveAngle);
		_downAngle = preferences.getInt(_downPrefName.c_str(), -servoInitialMoveAngle);
		_senseOnLevel = preferences.getUInt(_onPrefName.c_str(), adcInitialOnLevel);
		_senseOffLevel = preferences.getUInt(_offPrefName.c_str(), adcInitialOffLevel);

		_pinSense = pinSense;
		BreakerState_t state = breakerState();
		_breakerOn = state == breakerStateOn;
		_power = new Characteristic::On(_breakerOn);

		setLED(state);
		setStatus(state);

		new Service::ContactSensor();
			_tripped = new Characteristic::ContactSensorState(trippedSensorFalse);
			_trippedName = String(name) + String(" - Tripped");
			new Characteristic::ConfiguredName(_trippedName.c_str());

		#ifdef DEBUG
		showInfo();
		#endif

		centerServo();
	}

	const char* name() {
		return _name;
	}

	void showInfo() {
		Serial.printf("Breaker %d (%s) - pinServo:%d, pinButton:%d, pinSense:%d, pinLED:%d, center:%d, up:%d, down:%d, onLevel:%d, offLevel:%d, hysterisis:%d\n",
						_statusIndex, _name, _pinServo, _pinButton, _pinSense, _pinLED, _centerAngle, _upAngle, _downAngle, _senseOnLevel, _senseOffLevel, _senseHysterisisValue);
	}

	void showState() {
		Serial.printf("Breaker %d (%s) - State: %s%s\n", _statusIndex, _name, _breakerOn ? "On" : "Off", _tripped->getVal()==trippedSensorTrue ? " (TRIPPED!)" : "");
	}

	void centerServo() {
		_servoMoveTime = millis64() - servoPushTimeMS;	// trigger setting servo to center position
	}

	BreakerState_t breakerState(uint16_t* value = NULL, bool useHysterisis = false) {
		uint16_t adcVal = analogReadClean(_pinSense);

		if (value) {
			*value = adcVal;
		}
		if (!useHysterisis) {
			_senseHysterisisValue = -1;
		}

		int senseOn = _senseOnLevel;
		int senseOff = _senseOffLevel;

		if (_senseHysterisisValue!=-1) {
			senseOn = min(senseOn + hysterisisRange / 2, max(senseOn, _senseHysterisisValue + hysterisisRange));
			senseOff = max(senseOff - hysterisisRange / 2, min(senseOff, _senseHysterisisValue - hysterisisRange));
		}

		if (adcVal >= senseOn) {
			_senseHysterisisValue = -1;
			return breakerStateOn;
		}
		else if (adcVal <= senseOff) {
			_senseHysterisisValue = -1;
			return breakerStateOff;
		}
		else {
			if (useHysterisis) {
				_senseHysterisisValue = adcVal;
			}
			return breakerStateTripped;
		}
	}

	void setStatus(BreakerState_t state) {
		_statusStrip->setPixel(_statusIndex, state);
	}

	void updateLED() {
		switch(_ledState) {
			case breakerStateOn:		_blinker->on();											break;
			case breakerStateTripped:	_blinker->start(ledTrippedRateMS, ledTrippedPeriod);	break;
			default:					_blinker->off();										break;
		}
	}

	void setDark(bool dark, bool force) {
		if (dark != _dark || force) {
			_dark = dark;
			_led->setOnLevel(_dark ? ledDarkBrightness : ledBrightBrightness);
			updateLED();
		}
	}

	void setLED(BreakerState_t state) {
		if (state != _ledState) {
			_ledState = state;
			updateLED();
		}
	}

	void setServoAngle(int16_t angle, bool silent = false) {
		if (_servo && angle != _servoAngle) {
			if (!silent) {
				if (angle == servoDisable) {
					SerPrintf("%6lld: Breaker %d (%s) - servo disabled\n", millis64(), _statusIndex, _name);
				}
				else {
					SerPrintf("%6lld: Breaker %d (%s) - set servo to %dº\n", millis64(), _statusIndex, _name, angle);
				}
			}
			_servoAngle = angle;
			_servo->set(angle == servoDisable ? NAN : angle);
		}
	}

	int16_t getServoAngle() {
		return _servoAngle;
	}

	bool update() {
		if (_power->updated()) {
			bool newPower = _power->getNewVal();

			if (newPower != power()) {
				SerPrintf("%6lld: Breaker %d (%s) - HomeKit set power to %s\n", millis64(), _statusIndex, _name, newPower ? "ON" : "OFF");
				_changeToValue = newPower;
			}	
		}	
		return true;
	}	

	void loop() {
		uint64_t curTime = millis64();
		uint16_t adcVal = 0;
		bool senseAllowed = curTime >= _senseHoldTime;
		BreakerState_t state = breakerState(&adcVal, senseAllowed);
		bool curOnState = state == breakerStateOn;

		if (_servoMoveTime > 0) {
			if (curTime > _servoMoveTime + servoPushTimeMS * 2) {
				setServoAngle(servoDisable);
				_servoMoveTime = 0;
			}
			else if (curTime > _servoMoveTime + servoPushTimeMS) {
				if (getServoAngle() != _centerAngle) {
					setServoAngle(_centerAngle);
				}
			}
			else if (curTime > _servoMoveTime - servoResetTimeMS) {
				if (_breakerOn && getServoAngle() != _upAngle) {
					setServoAngle(_upAngle);
				}
			}
		}
		else if (_changeToValue != -1) {
			if (_changeToValue != _breakerOn) {
				_breakerOn = _changeToValue;
				if (state == breakerStateTripped) {
					setServoAngle(_downAngle);
					_servoMoveTime = curTime + servoPushTimeMS + servoResetTimeMS;
				}
				else {
					setServoAngle(_changeToValue ? _upAngle : _downAngle);
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
		else if (senseAllowed) {
			bool tripped = state == breakerStateTripped;
			int trippedValue = tripped ? trippedSensorTrue : trippedSensorFalse;

			if (tripped != _lastTripped) {
				_lastTripped = tripped;
				_lastTrippedTime = curTime;
			}

			if (curTime > _lastTrippedTime + trippedStableDurationMS && trippedValue != _tripped->getVal()) {
				_tripped->setVal(trippedValue);
				SerPrintf("%6lld: Breaker %d (%s) - tripped state changed to %s - (adc=%d of %d-%d)\n", curTime, _statusIndex, _name, trippedValue == trippedSensorTrue ? "TRIPPED" : "NOT TRIPPED", adcVal, _senseOffLevel, _senseOnLevel);
			}
			
			if (curOnState != _power->getVal()) {
				SerPrintf("%6lld: Breaker %d (%s) - state mismatch, setting to %d (state=%d, adc=%d of %d-%d)\n", curTime, _statusIndex, _name, curOnState, state, adcVal, _senseOffLevel, _senseOnLevel);
				_breakerOn = curOnState;
				_power->setVal(curOnState);
			}
		}

		setLED(state);
		setStatus(state);
	}

	void setPower(bool value, uint32_t afterDelayMS = 0) {
		if (value != power()) {
			SerPrintf("%6lld: Breaker %d (%s) - setPower to %d\n", millis64(), _statusIndex, _name, value);
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
			SerPrintf("%6lld: Breaker %d (%s) - Button Press - toggle power\n", millis64(), _statusIndex, _name);
			setPower(!power());
		}
		else if (_buddyBreaker && pressType == SpanButton::LONG) {
			bool newState = !power();

			SerPrintf("%6lld: Breaker %d (%s) - Button Long Press - toggle power, local and buddy\n", millis64(), _statusIndex, _name);
			setPower(newState);
			_buddyBreaker->setPower(newState, buddyDelayTimeMS);
		}
	}

	bool adjustAngles() {
		bool result = false;

		if (_servo) {
			int16_t *values[3] = { &_centerAngle, &_upAngle, &_downAngle };
			constexpr const char* names[3] = { "CENTER", "UP    ", "DOWN  " };
			constexpr uint16_t indexes[] = { 0, 1, 0, 2 };	// center, up, center, down
			constexpr uint16_t numIndexes = sizeof(indexes) / sizeof(*indexes);
			uint16_t curIndex = 0;
			bool changed = false;
			bool update = true;
			bool done = false;

			Serial.printf("\n%6lld: Breaker %d (%s) - Calibrate angles\n", millis64(), _statusIndex, _name);
			Serial.printf("To adjust press 'u' for up and 'd' for down\n");
			Serial.printf("Press <space> to select center angle, up angle, or down angle\n");
			Serial.printf("Press <return> to save, press <esc> to exit\n");
			while (!done) {
				if (update) {
					printAndBackUp("%6lld: Current %s angle: % 3dº", millis64(), names[indexes[curIndex]], *values[indexes[curIndex]]);
					setServoAngle(*values[indexes[curIndex]], true);
					update = false;
				}
				if (Serial.available()) {
					char c = Serial.read();
					if (c == '\n' || c == '\r') {
						Serial.printf("\n");
						done = true;
						result = true;
					}
					else if (c == 27) { // ESC
						Serial.printf("\n\nCalibration cancelled.\n");
						done = true;
						changed = false;
					}
					else if (c == 'u' || c == 'U') { // up
						Serial.print('\b');
						*values[indexes[curIndex]] += 1;
						changed = true;
						update = true;
					}
					else if (c == 'd' || c == 'D') { // down
						Serial.print('\b');
						*values[indexes[curIndex]] -= 1;
						changed = true;
						update = true;
					}
					else if (c == ' ') { // space
						Serial.print('\b');
						curIndex = (curIndex + 1) % numIndexes;
						update = true;
					}
				}
			}
			if (changed) {
				Serial.printf("\nCalibration complete. Saving new angles: center=%dº, up=%dº, down=%dº\n", _centerAngle, _upAngle, _downAngle);
				preferences.putInt(_centerPrefName.c_str(), _centerAngle);
				preferences.putInt(_upPrefName.c_str(), _upAngle);
				preferences.putInt(_downPrefName.c_str(), _downAngle);
			}
			else {
				Serial.printf("\nNo changes made to center angle.\n");
			}

			centerServo();
		}
		return result;
	}

	bool calibrateSenseLevels() {
		bool result = false;

		if (_servo) {
			Serial.printf("\n%6lld: Breaker %d (%s) - Calibrate sense levels\n", millis64(), _statusIndex, _name);
			Serial.printf("Measuring On... %d\n", _senseOnLevel);

			uint16_t onValue, offValue;

			setServoAngle(_upAngle, true);
			delay(senseChangeTimeMS);
			breakerState(&onValue);

			Serial.printf("Measuring Off... %d\n", _senseOffLevel);

			setServoAngle(_downAngle, true);
			delay(senseChangeTimeMS);
			breakerState(&offValue);

			setServoAngle(_centerAngle, true);
			delay(servoPushTimeMS);

			uint16_t newOnLevel = offValue + (onValue - offValue) * tripUpperPercentage / 100;
			uint16_t newOffLevel = offValue + (onValue - offValue) * tripLowerPercentage / 100;

			Serial.printf("\nNew On Level: %d, New Off Level: %d (on=%d, off=%d)\n", newOnLevel, newOffLevel, onValue, offValue);
			Serial.printf("Press 'Y' to accept these new levels, or any other key to reject.\n");

			if (waitForY()) {
				_senseOnLevel = newOnLevel;
				_senseOffLevel = newOffLevel;
				preferences.putUInt(_onPrefName.c_str(), _senseOnLevel);
				preferences.putUInt(_offPrefName.c_str(), _senseOffLevel);
				result = true;
				Serial.printf("\nNew sense levels ACCEPTED.\n");
			}
			else {
				Serial.printf("\nNew sense levels REJECTED.\n");
			}

			centerServo();
		}
		return result;
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
			SerPrintf("\n%6lld: PowerLevel: Activate - %dA\n", millis64(), _amps);

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

//--------------------------------------------------------------------

PowerLevel* fullPower;
PowerLevel* reducedPower;
PowerLevel* minimumPower;

BreakerController* SPAN_BREAKER(const char* name, int pinServo, bool reverse, int pinButton, int pinSense, int pinLed, int statusIndex) {
	SPAN_ACCESSORY(name);
	return new BreakerController(pinServo, reverse, pinButton, pinSense, pinLed, &statusStrip, statusIndex, name);
}

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

		washer = SPAN_BREAKER("Washer", pinServoWasher, true, pinButtonWasher, pinSenseWasher, pinLedWasher, statusIndexWasher);
		dryer = SPAN_BREAKER("Dryer", pinServoDryer, true, pinButtonDryer, pinSenseDryer, pinLedDryer, statusIndexDryer);
		inverter = SPAN_BREAKER("Inverter", pinServoInverter, true, pinButtonInverter, pinSenseInverter, pinLedInverter, statusIndexInverter);
		converter = SPAN_BREAKER("Converter", pinServoConverter, false, pinButtonConverter, pinSenseConverter, pinLedConverter, statusIndexConverter);

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
			Serial.printf("\n%6lld: Invalid index, must be 0-3\n", millis64());
			return NULL;
		}
	}
}

void cmdAdjustAngles(const char *buf){
	BreakerController* breaker = getBreakerFromCmd(buf);
	if (breaker) {
		breaker->adjustAngles();
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
			Serial.printf("\n%6lld: Setting Breaker %d (%s) to OFF\n", millis64(), breaker->_statusIndex, breaker->_name);
			breaker->setPower(false);
		}
		else if (buf[2] == '1') {
			Serial.printf("\n%6lld: Setting Breaker %d (%s) to ON\n", millis64(), breaker->_statusIndex, breaker->_name);
			breaker->setPower(true);
		}
		else {
			Serial.printf("\n%6lld: Invalid command, must be 2 digits, 0-3 followed by 0 or 1\n", millis64());
		}
	}
}

void cmdShowInfo(const char *buf){
	if (buf[1] == 0) {
		washer->showInfo();
		dryer->showInfo();
		inverter->showInfo();
		converter->showInfo();
	}
	else {
		BreakerController* breaker = getBreakerFromCmd(buf);
		if (breaker) {
			breaker->showInfo();
		}
	}
}

void cmdShowState(const char *buf){
	if (buf[1] == 0) {
		washer->showState();
		dryer->showState();
		inverter->showState();
		converter->showState();
	}
	else {
		BreakerController* breaker = getBreakerFromCmd(buf);
		if (breaker) {
			breaker->showState();
		}
	}
}

bool strMatchAny(const char* str1, ...) {
	va_list args;
	va_start(args, str1);
	const char* str2;
	while ((str2 = va_arg(args, const char*)) != NULL) {
		if (strcasecmp(str1, str2) == 0) {
			va_end(args);
			return true;
		}
	}
	va_end(args);
	return false;
}

void cmdPowerLevel(const char *buf){
	buf++;
	if (buf[0] == 0) {
		Serial.printf("\n%6lld: Current power level: %dA\n", millis64(), fullPower->currentAmps());
	}
	else if (strMatchAny(buf, "15", "min", "minimum")) {
		Serial.printf("\n%6lld: Setting power level to MINIMUM (15A)\n", millis64());
		minimumPower->activate();
	}
	else if (strMatchAny(buf, "20", "low", "reduced")) {
		Serial.printf("\n%6lld: Setting power level to LOW (20A)\n", millis64());
		reducedPower->activate();
	}
	else if (strMatchAny(buf, "50", "full")) {
		Serial.printf("\n%6lld: Setting power level to FULL (50A)\n", millis64());
		fullPower->activate();
	}
	else {
		Serial.printf("\n%6lld: Invalid power level. Use '15/min/minimum', '20/low/reduced', or '50/full'.\n", millis64());
	}
}

void cmdClearPreferences(const char *buf) {
	Serial.printf("\n%6lld: Clear preferences\n", millis64());
	Serial.printf("This will remove all saved settings, including breaker angles and sense levels.\n");
	Serial.printf("Press 'Y' to confirm and restart, or any other key to cancel.\n");

	if (waitForY()) {
		preferences.clear();
		preferences.end();
		Serial.printf("\nPreferences cleared. Restarting...\n");
		ESP.restart();
	}
	else {
		Serial.printf("\nClear preferences cancelled.\n");
	}
}

void cmdUpdateAccessories(const char *buf){

	if(homeSpan.updateDatabase()) {
		Serial.printf("Accessories Database updated.  New configuration number broadcast...\n");
	}
	else {
		Serial.printf("Nothing to update - no changes were made!\n");
	}
}

void cmdShowCPUStats(const char *buff = nullptr) {
	Serial.printf("\n%6lld: *** CPU Stats ***\n\n", millis64());

	Serial.printf("CPU frequency: %ldMHz\n", getCpuFrequencyMhz());
	Serial.printf("Xtal frequency: %ldMHz\n", getXtalFrequencyMhz());
	Serial.printf("Bus frequency: %ldMHz\n", getApbFrequency()/1000000);
	Serial.printf("Total heap: %ld\n", ESP.getHeapSize());
	Serial.printf("Free heap: %ld\n", ESP.getFreeHeap());
	Serial.printf("Total PSRAM: %ld\n", ESP.getPsramSize());
	Serial.printf("Free PSRAM: %ld\n", ESP.getFreePsram());

	Serial.printf("\n*** CPU Stats ***\n\n");
}

bool adcStatusEnabled = false;

void cmdToggleADCStatus(const char *buf) {
	adcStatusEnabled = !adcStatusEnabled;
	Serial.printf("ADC Status: %s\n", adcStatusEnabled ? "ON" : "OFF");
}

void addCommands() {
	new SpanUserCommand('s',"show CPU stats", cmdShowCPUStats);
	new SpanUserCommand('u',"update accessory database", cmdUpdateAccessories);
	new SpanUserCommand('x',"clear preferences", cmdClearPreferences);
	new SpanUserCommand('a',"toggle ADC/Ambient value display", cmdToggleADCStatus);
	new SpanUserCommand('b',"set breaker N to 0=off, 1=on)", cmdSetBreakerPower);
	new SpanUserCommand('p',"show panel power level, or set panel power level to N amps, or 'min', 'low', 'full'", cmdPowerLevel);
	new SpanUserCommand('n',"adjust servo angles for breaker N", cmdAdjustAngles);
	new SpanUserCommand('l',"measure sense levels for breaker N", cmdMeasureSenseLevels);
	new SpanUserCommand('i',"show info for all breakers, or for breaker N", cmdShowInfo);
	new SpanUserCommand('t',"show state for all breakers, or for breaker N", cmdShowState);
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

bool wifiConnected = false;

void statusChanged(HS_STATUS status) {
	if (wifiConnected && status == HS_WIFI_CONNECTING) {
		wifiConnected = false;
		setIndicator(connectingColor);
		SerPrintf("Lost WIFI Connection...\n");
	}
}

void wifiReady(int ready) {
	wifiConnected = true;
	setIndicator(readyColor);
	SerPrintf("WIFI: Connected.\n");
}

void setup() {
	enablePower();
	statusStrip.begin(statusStripPin, statusStripLength, true);

	flashIndicator(flashColor, 20, 100);
	setIndicator(startColor);

	Serial.begin(115200);
	SerPrintf("Power Center Controller Startup\n");

	SerPrintf("Setup status LED strip\n");
	statusStrip.setDark(false, false);

	for (auto i=-1; i<=statusStripLength; i++) {
		statusStrip.setPixel(i-1, breakerStateUnknown, false);
		statusStrip.setPixel(i+1, breakerStateOff, false);
		statusStrip.setPixel(i, breakerStateOn, false);
		statusStrip.showPixels();
		delay(100);
	}

	statusStrip.setDark(true, false);

	SerPrintf("Initialize Preferences\n");
	preferences.begin(prefsPartitionName, false);

	SerPrintf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setConnectionCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.begin(Category::Bridges, displayName, DEFAULT_HOST_NAME, modelName);
	setIndicator(connectingColor);

	SerPrintf("Create devices\n");
	createDevices();

	SerPrintf("Add Commands\n");
	addCommands();

	SerPrintf("Init complete.\n");
}

void checkAmbientLight() {
	static uint64_t ambientCheckTime = 0;
	uint64_t curTime = millis64();

	if (curTime >= ambientCheckTime) {
		ambientCheckTime = curTime + lightCheckRateMS;
		uint16_t ambientLight = analogReadClean(ambientLightPin, ambientLightReadCount);

		if (ambientLight <= darkLowThreshold) {
			statusStrip.setDark(true);
		}
		else if (ambientLight >= darkHighThreshold) {
			statusStrip.setDark(false);
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
			uint16_t ambientLight = analogReadClean(ambientLightPin, ambientLightReadCount);

			Serial.printf("%6lld: ADC Washer: %4d, Dryer: %4d, Inverter: %4d, Converter: %4d - Ambient Light: %4d\n", curTime, adcWasher, adcDryer, adcInverter, adcConverter, ambientLight);
		}
	}
}

void loop() {
	homeSpan.poll();

	checkAmbientLight();
	showADCStatus();
}

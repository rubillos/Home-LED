#include "Arduino.h"

#include "HomeSpan.h" 
#include "Wire.h"
#include "Adafruit_HUSB238.h"

Adafruit_HUSB238 husb238;

//////////////////////////////////////////////

constexpr const char* versionString = "v0.5";

//////////////////////////////////////////////

constexpr uint32_t ledUpdateRate = 100;
constexpr uint32_t ledUpdateRateInterval = 1000 / ledUpdateRate;

constexpr uint32_t defaultFadeUpTime = 500;
constexpr uint32_t defaultFadeDownTime = 2000;
constexpr uint32_t defaultFadeFastTime = 300;

constexpr uint32_t indicatorOffTime = 30 * 1000;

constexpr float homeKitBrightnessMax = 100.0;

constexpr uint32_t dimmerTaskStackSize = 3000;

//////////////////////////////////////////////

constexpr float ledGamma = 2.8;
constexpr uint8_t maxIndicatorValue = 20;

constexpr uint8_t ledStatusLevel = 0x10;

#define MAKE_RGB(r, g, b) (r<<16 | g<<8 | b)
constexpr uint32_t flashColor = MAKE_RGB(ledStatusLevel, 0, ledStatusLevel);
constexpr uint32_t startColor = MAKE_RGB(ledStatusLevel, 0, 0);
constexpr uint32_t connectingColor = MAKE_RGB(0, 0, ledStatusLevel);
constexpr uint32_t readyColor = MAKE_RGB(0, ledStatusLevel, 0);

uint32_t currentIndicatorColor = 0xFFFFFFFF;
uint32_t savedIndicatorColor = 0;

constexpr uint32_t whiteColorFlag = 1 << 24;

//////////////////////////////////////////////

typedef enum {
	output12V = 0,
	output9V = 1,
	outputOther = 3
} OutputVoltage_t;

OutputVoltage_t availableOutputVoltage = output12V;

constexpr uint32_t color9V = MAKE_RGB(ledStatusLevel, ledStatusLevel, 0);
constexpr uint32_t colorLowV = MAKE_RGB(ledStatusLevel, 0, ledStatusLevel/2);

//////////////////////////////////////////////

#if defined(OTADEBUG) || defined(DEBUG)
#define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//////////////////////////////////////////////

typedef enum {
	Device_RGB = 0,
	Device_RGBW,
	Device_W,

	Device_DotStar_RGB,
	Device_DotStar_W,

	Device_LED,

	Device_Switch,
	Device_Fan,

	Device_Button,
} StripType;

constexpr uint16_t SwitchToggleBit = 0x4000;
constexpr uint16_t SwitchForceFullBit = 0x2000;
constexpr uint16_t SwitchGroupMask = 0x1C00;
constexpr uint16_t SwitchGroupShift = 10;
constexpr uint16_t SwitchPinMask = 0x03FF;

constexpr uint32_t SwitchDoublePressTime = 500;

#define SWITCH_GROUP_BITS(groupID) (groupID << SwitchGroupShift)
#define SWITCH_GROUP_ID(buttonPin) ((buttonPin & SwitchGroupMask) >> SwitchGroupShift)

//////////////////////////////////////////////

class SetValue {
	public:
		SetValue(StripType type=Device_RGB) : _type(type) {};
		virtual void setValue(float value) {};
		uint32_t brightnessToColor(float value) {
			switch (_type) {
				case Device_RGB:
				case Device_DotStar_RGB: {
					uint8_t w = value / homeKitBrightnessMax * 255;
					return w<<16 | w<<8 | w;
				}
				case Device_RGBW: {
					uint8_t w = value / homeKitBrightnessMax * 255;
					return w<<24;;
				}
				case Device_W:
				case Device_DotStar_W: {
					uint16_t w = value / homeKitBrightnessMax * 255 * 3;
					uint8_t w1 = max(0, w-512);
					uint8_t w2 = min(255, max(0, w-256));
					uint8_t w3 = min(255, max(0, w-0));
					return w1<<16 | w2<<8 | w3;
				}
				default:
					return 0;
					break;
			}
		};

	private:
		StripType _type;
};

class LedPinWithSetValue : public LedPin , public SetValue {
	public:
		LedPinWithSetValue(uint8_t pin, float level=0, uint16_t freq=DEFAULT_PWM_FREQ, boolean invert=false) : LedPin(pin, level, freq, invert) {};
		void setValue(float value) { set(value); };
};

class PixelWithSetValue : public Pixel , public SetValue {
	public:
		PixelWithSetValue(int pin, int pixelCount, StripType type=Device_RGB) : Pixel(pin, ((type==Device_RGBW) ? "GRBW" : "GRB")), SetValue(type) { };
		void setValue(float value) {
			uint32_t color = brightnessToColor(value);
			set(Pixel::Color().RGB((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, color>>24));
		};
};

class DotWithSetValue : public Dot , public SetValue {
	public:
		DotWithSetValue(uint8_t dataPin, uint8_t clockPin, int pixelCount, StripType type=Device_RGB) : Dot(dataPin, clockPin), SetValue(type) { };
		void setValue(float value) {
			uint32_t color = brightnessToColor(value);
			set(Dot::Color().RGB((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF, color>>24));
		};
};

//////////////////////////////////////////////

typedef struct LEDDeviceRec {
	StripType type;
	const char* name;
	uint8_t outputPin;
	int16_t buttonPin = -1;
	int16_t clockPin = -1;
	uint8_t count = 0;
	uint16_t fadeUpDuration = 0;
	uint16_t fadeDownDuration = 0;
	uint16_t fadeFastDuration = 0;

	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, int16_t buttonPin, uint16_t fadeUpDuration, uint16_t fadeDownDuration, uint16_t fadeFastDuration) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin), 
				fadeUpDuration(fadeUpDuration), fadeDownDuration(fadeDownDuration), fadeFastDuration(fadeFastDuration) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t buttonPin, uint16_t fadeUpDuration, uint16_t fadeDownDuration, uint16_t fadeFastDuration) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin), count(count), 
				fadeUpDuration(fadeUpDuration), fadeDownDuration(fadeDownDuration), fadeFastDuration(fadeFastDuration) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t clockPin, int16_t buttonPin, uint16_t fadeUpDuration, uint16_t fadeDownDuration, uint16_t fadeFastDuration) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin), clockPin(clockPin), count(count), 
				fadeUpDuration(fadeUpDuration), fadeDownDuration(fadeDownDuration), fadeFastDuration(fadeFastDuration) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, int16_t buttonPin) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t buttonPin) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin), count(count) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t clockPin, int16_t buttonPin) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin), clockPin(clockPin), count(count) { }
	LEDDeviceRec(StripType type, const char* name, const int16_t buttonPin) :
				type(type), name(name), buttonPin(buttonPin) { }
} LEDDeviceRec;

// Device_RGB/Device_RGBW/Device_W,Device_LED, "name", outputPin, buttonPin
// Device_RGB/Device_RGBW/Device_W,Device_LED, "name", outputPin, count, buttonPin
// Device_RGB/Device_RGBW/Device_W,Device_LED, "name", outputPin, buttonPin, fadeUpDuration, fadeDownDuration, fadeFastDuration
// Device_RGB/Device_RGBW/Device_W,Device_LED, "name", outputPin, count, buttonPin, fadeUpDuration, fadeDownDuration, fadeFastDuration
//
// Device_Dotstar_RGB/Device_DotStar_W, "name", "dataPin", count, clockPin, buttonPin
// Device_Dotstar_RGB/Device_DotStar_W, "name", outputPin, count, clockPin, buttonPin, fadeUpDuration, fadeDownDuration, fadeFastDuration
//
// Device_Switch, "name", outputPin, buttonPin
// Device_LED, "name", outputPin, buttonPin
// Device_Fan, "name", outputPin, unused, neoPixelPin, buttonPin
//
// Device_Button, "name", buttonPin

//////////////////////////////////////////////

// #define PENGUIN_LIGHT
#define PNGXPRS_LOWER_ACCENT
// #define PNGXPRS_UPPER_LED
// #define PNGXPRS_DINING
// #define FAN_LR
// #define FAN_BB

#if defined(TINYS3)

#if defined(PNGXPRS_UPPER_LED)

// Penguin Express Top Lighting Strip
constexpr const char* displayName = "Upper-LED-Controller";
constexpr const char* modelName = "Upper-LED-Controller-ESP32";
LEDDeviceRec deviceList[] = {
	{ Device_LED, "LED", 8, 36 },
	{ Device_Button, "Button", 37 },
};
#define CPU_FREQUENCY 80

#elif defined(FAN_LR)

constexpr const char* displayName = "Fan-Controller";
constexpr const char* modelName = "Fan-Controller-LR";
LEDDeviceRec deviceList[] = {
	{ Device_Fan, "Fan", 2, 0, 3, 4 | SwitchForceFullBit },
};

#elif defined(FAN_BB)
constexpr const char* displayName = "Fan-Controller";
constexpr const char* modelName = "Fan-Controller-BB";
LEDDeviceRec deviceList[] = {
	{ Device_Fan, "Fan", 2, 0, 3, 4 | SwitchForceFullBit },
};

#else
#error No Device Definitions Found - Check build defines
#endif

#elif defined(QTPYS3)
// Penguin Light
#if defined(PENGUIN_LIGHT)

constexpr const char* displayName = "Penguin-Controller";
constexpr const char* modelName = "Penguin-Controller-ESP32";
LEDDeviceRec deviceList[] = {
	{ Device_LED, "Penguin", SCK, 0 },
};

#elif defined(PNGXPRS_LOWER_ACCENT)

constexpr const char* displayName = "Accent-Controller";
constexpr const char* modelName = "Accent-Controller-ESP32";
LEDDeviceRec deviceList[] = {
	{ Device_LED, "Lower Accent", MOSI, 0 },
};
#define CPU_FREQUENCY 80

#elif defined(PNGXPRS_DINING)

constexpr const char* displayName = "Dining-Controller";
constexpr const char* modelName = "Dining-Controller-ESP32";
LEDDeviceRec deviceList[] = {
	{ Device_LED, "Left", MOSI, SCK | SwitchToggleBit | SwitchForceFullBit | SWITCH_GROUP_BITS(1), 100, 1000, defaultFadeFastTime },
	{ Device_LED, "Right", MISO, RX | SwitchToggleBit | SwitchForceFullBit | SWITCH_GROUP_BITS(1), 100, 1000, defaultFadeFastTime },
};
#define CPU_FREQUENCY 80

#else
#error No Device Definitions Found - Check build defines
#endif

#endif

constexpr const char* otaPassword = "seattle";

constexpr auto deviceCount = sizeof(deviceList) / sizeof(LEDDeviceRec);

//////////////////////////////////////////////

#ifdef TINYS3
constexpr uint8_t indicatorDataPin = 18;
constexpr uint8_t indicatorPowerPin = 17;
#elif QTPYS3
constexpr uint8_t indicatorDataPin = PIN_NEOPIXEL;
constexpr uint8_t indicatorPowerPin = NEOPIXEL_POWER;
#endif

uint64_t millis64() {
  volatile static uint32_t low32 = 0, high32 = 0;
  uint32_t new_low32 = millis();

  if (new_low32 < low32)
	high32++;

  low32 = new_low32;

  return (uint64_t) high32 << 32 | low32;
}

volatile uint64_t lastIndicatorChange;
bool indicatorPowered = false;

uint8_t pixelBrightness = 255;

inline uint8_t mul8x8(uint8_t a, uint8_t b) { return (a * b) / 256; }
inline uint8_t pixBright(uint8_t value) { return mul8x8(value, pixelBrightness); }

void setupIndicator() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);
	indicatorPowered = true;
}

void setIndicatorBrightness(uint8_t brightness) {
	pixelBrightness = brightness;
}

void indicatorSetColor(uint32_t color) {
	rgbLedWrite(indicatorDataPin, pixBright(color>>16), pixBright((color>>8)&0xFF), pixBright(color&0xFF));
}

void setIndicator(uint32_t color, bool saveColor = false, bool updateChanged = true) {
	if (!indicatorPowered) {
		pinMode(indicatorPowerPin, OUTPUT);
		digitalWrite(indicatorPowerPin, HIGH);
		indicatorPowered = true;
		currentIndicatorColor = 0xFFFFFFFE;
	}

	if (color != currentIndicatorColor) {
		currentIndicatorColor = color;
		if (saveColor) {
			savedIndicatorColor = color;
		}
		if (color & whiteColorFlag) {
			color &= 0xFF;
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

void powerOffIndicator() {
	if (indicatorPowered) {
		digitalWrite(indicatorPowerPin, LOW);
		indicatorPowered = false;
	}
}

//////////////////////////////////////////////

typedef struct {
	SetValue* led = NULL;

	volatile float targetLevel = 0;
	volatile float currentLevel = 0;
	volatile float currentCorrectedLevel = 0;
	volatile float levelStep = 0;

	volatile float newTargetLevel = 0;
	volatile float newLevelStep = 0;
} DimInfoRec;

DimInfoRec dimmerData[deviceCount];

//////////////////////////////////////////////

inline float gammaCorrect(float value) {
	return pow(value / homeKitBrightnessMax, ledGamma) * homeKitBrightnessMax;
}

void dimmerTask(void* params) {
	TickType_t lastTicks = xTaskGetTickCount();
	float currentIndicatorValue = -1;
	uint32_t loopCount = 0;

	while (true) {
		xTaskDelayUntil(&lastTicks, pdMS_TO_TICKS(ledUpdateRateInterval));

		float firstValue = -1;

		for (auto i=0; i<deviceCount; i++) {
			DimInfoRec* dimInfo = &dimmerData[i];

			if (dimInfo->newLevelStep != 0) {
				dimInfo->targetLevel = dimInfo->newTargetLevel;
				dimInfo->levelStep = dimInfo->newLevelStep;
				dimInfo->newLevelStep = 0;
			}
			if (dimInfo->levelStep) {
				dimInfo->currentLevel += dimInfo->levelStep;

				if ((dimInfo->levelStep > 0 && dimInfo->currentLevel >= dimInfo->targetLevel) || (dimInfo->levelStep < 0 && dimInfo->currentLevel <= dimInfo->targetLevel)) {
					dimInfo->currentLevel = dimInfo->targetLevel;
					dimInfo->levelStep = 0;
				}
				dimInfo->currentCorrectedLevel = gammaCorrect(dimInfo->currentLevel);
				if (dimInfo->led) {
					dimInfo->led->setValue(dimInfo->currentCorrectedLevel);
				}
			}
			if (firstValue==-1 && dimInfo->currentLevel>0) {
				firstValue = dimInfo->currentCorrectedLevel;
			}
		}

		bool updateChange = true;

		if (firstValue == -1) {
			uint64_t time = millis64() - lastIndicatorChange;
			if (time > indicatorOffTime) {
				if (loopCount<295) {
					firstValue = -2;
				}
				else {
					updateChange = false;
				}
			}
		}

		if ((availableOutputVoltage != output12V) && (loopCount > 100) && (loopCount < 150)) {
			uint32_t color = 0;

			if (loopCount > 110 && loopCount < 140) {
				color = availableOutputVoltage == output9V ? color9V : colorLowV;
			}

			setIndicator(color, false, false);
			currentIndicatorValue = -3;
		}
		else if (firstValue != currentIndicatorValue) {
			if (firstValue == -1) {
				setIndicator(savedIndicatorColor, false, updateChange && !(currentIndicatorValue == -3));
			}
			else if (firstValue == -2) {
				powerOffIndicator();
			}
			else {
				uint8_t value255 = firstValue * maxIndicatorValue / homeKitBrightnessMax;
				setIndicator8(max((uint8_t)1, value255));
			}
			currentIndicatorValue = firstValue;
		}

		loopCount = (loopCount + 1) % 300;
	}
}

//////////////////////////////////////////////

struct ButtonDevice;

typedef struct {
	ButtonDevice* buttonDevice;
	uint8_t groupID;	
} ButtonGroupItem;

ButtonGroupItem buttonGroupList[deviceCount];
uint16_t buttonGroupCount = 0;

struct ButtonDevice {
	SpanCharacteristic *_onOff = NULL;
	SpanCharacteristic *_level = NULL;

	SpanButton* _button = NULL;
	bool _forceFull = false;
	uint8_t _groupID = 0;
	uint32_t _lastPressTime;

	DimInfoRec* _dimInfo;

	ButtonDevice(int16_t buttonPin, DimInfoRec* dimInfo) : _dimInfo(dimInfo) {
		if (buttonPin != -1) {
			if (buttonPin & SwitchToggleBit) {
				SerPrintf("Add toggle switch on pin %d\n", buttonPin & SwitchPinMask);
				_button = new SpanToggle(buttonPin & SwitchPinMask);
			}
			else {
				SerPrintf("Add button on pin %d\n", buttonPin & SwitchPinMask);
				_button = new SpanButton(buttonPin & SwitchPinMask);
			}

			_forceFull = (buttonPin & SwitchForceFullBit) != 0;

			if (buttonPin & SwitchGroupMask) {
				_groupID = SWITCH_GROUP_ID(buttonPin);
				SerPrintf("Assign to group %d\n", _groupID);
				buttonGroupList[buttonGroupCount].buttonDevice = this;
				buttonGroupList[buttonGroupCount].groupID = _groupID;
				buttonGroupCount++;
			}
		}
	};

	virtual void refreshOutput() {};

	void setState(bool on) {
		_onOff->setVal(on);
		if (_level != NULL && on && _level->getVal() < (_forceFull ? 1 : 100)) {
			_level->setVal(100);
		}
		refreshOutput();
	}

	void handleButton(int pressType) {
		if (pressType == SpanButton::SINGLE || pressType == SpanToggle::CLOSED || pressType == SpanToggle::OPEN) {
			uint32_t time = millis();
			bool state = _onOff->getVal();
			
			if (_groupID != 0 && ((time-_lastPressTime) < SwitchDoublePressTime)) {
				for (auto i=0; i<buttonGroupCount; i++) {
					if (buttonGroupList[i].groupID == _groupID && buttonGroupList[i].buttonDevice != this) {
						buttonGroupList[i].buttonDevice->setState(state);
					}
				}
			}
			else {
				setState(!state);
			}
			_lastPressTime = time;
		}
	}
};

//////////////////////////////////////////////

float fadeStep(uint16_t duration, uint16_t defaultValue) {
	return homeKitBrightnessMax * ledUpdateRateInterval / (duration==0 ? defaultValue : duration);
}

struct DimmableLED : Service::LightBulb, ButtonDevice {
	float _fadeUpStep;
	float _fadeDownStep;
	float _fadeFastStep;

	DimmableLED(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::LightBulb(), ButtonDevice(device->buttonPin, dimInfo) {
		_onOff = new Characteristic::On();
		_level = new Characteristic::Brightness(100);
		_level->setRange(0, 100, 1);

		switch (device->type) {
			case Device_RGB:
			case Device_RGBW:
			case Device_W:
				_dimInfo->led = new PixelWithSetValue(device->outputPin, device->count, device->type);
				break;
			case Device_DotStar_RGB:
			case Device_DotStar_W:
				_dimInfo->led = new DotWithSetValue(device->outputPin, device->clockPin, device->count, device->type);
				break;
			case Device_LED:
			default:
				_dimInfo->led = new LedPinWithSetValue(device->outputPin, 0, 20000);
				break;
		}

		_fadeUpStep = fadeStep(device->fadeUpDuration, defaultFadeUpTime);
		_fadeDownStep = fadeStep(device->fadeDownDuration, defaultFadeDownTime);
		_fadeFastStep = fadeStep(device->fadeFastDuration, defaultFadeFastTime);
	}

	void refreshLevel(bool fast = false) {
		float currentLevel = _dimInfo->currentLevel;
		float newLevel = _onOff->getNewVal() * _level->getNewVal();

		if (newLevel != currentLevel) {
			float step;

			if (fast) {
				step = (newLevel >= currentLevel) ? _fadeFastStep : -_fadeFastStep;
				step *= abs(newLevel-currentLevel) / homeKitBrightnessMax;
			}
			else {
				step = (newLevel >= currentLevel) ? _fadeUpStep : -_fadeDownStep;
			}

			while (_dimInfo->newLevelStep != 0) {};

			_dimInfo->newTargetLevel = newLevel;
			_dimInfo->newLevelStep = step;
		}
	}

	boolean update() {
		refreshLevel(_level->updated());
		return(true);  
	}

	void refreshOutput() {
		refreshLevel();
	}

	void button(int pin, int pressType) {
		handleButton(pressType);
	}
};

//////////////////////////////////////////////
struct FanPWM : Service::Fan, ButtonDevice {
	SetValue* _output;
	Pixel *_buttonLED = NULL;

	FanPWM(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::Fan(), ButtonDevice(device->buttonPin, dimInfo) {
		_onOff = new Characteristic::Active(false);
		_level = new Characteristic::RotationSpeed(0, true);

		_level->setRange(0, 100, 1);

		_output = new LedPinWithSetValue(device->outputPin, 0, 25000);

		if (device->clockPin != -1) {
			_buttonLED = new Pixel(device->clockPin, "GRB");
		}

		update();
	}

	boolean update() {
		float newSpeed = _onOff->getNewVal() * _level->getNewVal();
		constexpr uint8_t ledBright = 30;
		Pixel::Color color = Pixel::RGB(0, ledBright/3, 0);

		if (newSpeed > 0) {
			uint8_t rL = ledBright*6/10;
			uint8_t rH = ledBright;
			uint8_t gL = ledBright*6/10;
			uint8_t gH = 0;
			uint8_t r = rL + (rH - rL) * newSpeed / homeKitBrightnessMax;
			uint8_t g = gL + (gH - gL) * newSpeed / homeKitBrightnessMax;

			color = Pixel::RGB(r, g, 0);
			newSpeed = 10 + 90 * newSpeed / homeKitBrightnessMax;
		}

		if (_buttonLED) {
			_buttonLED->set(color);
		}

		_output->setValue(newSpeed);
		_dimInfo->currentLevel = newSpeed;
		_dimInfo->currentCorrectedLevel = gammaCorrect(_dimInfo->currentLevel) * 0.1;

		return true;
	}

	void refreshOutput() {
		update();
	}

	void button(int pin, int pressType) {
		handleButton(pressType);
	}
};

//////////////////////////////////////////////
struct DigitalPin : Service::Switch, ButtonDevice {
	int _pin;

	DigitalPin(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::Switch(), ButtonDevice(device->buttonPin, dimInfo), _pin(device->outputPin) {
		_onOff = new Characteristic::On();
		pinMode(_pin,OUTPUT);
		digitalWrite(_pin, LOW);
	}

	boolean update() {            
		bool value = _onOff->getNewVal();
		digitalWrite(_pin, value);
		_dimInfo->currentLevel = value * 100;
		_dimInfo->currentCorrectedLevel = _dimInfo->currentLevel;
		return(true);
	}

	void refreshOutput() {
		update();
	}

	void button(int pin, int pressType) {
		handleButton(pressType);
	}
};
	  
//////////////////////////////////////////////

struct HomeButton : Service::StatelessProgrammableSwitch {
	SpanCharacteristic* _switchEvent;
	SpanButton* _button;
  
	HomeButton(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::StatelessProgrammableSwitch() {
		_switchEvent = new Characteristic::ProgrammableSwitchEvent();
		_button = new SpanButton(device->buttonPin);
	}

	void button(int pin, int pressType) override {
		_switchEvent->setVal(pressType);
	}
};

//////////////////////////////////////////////

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

	for (auto i=0; i<deviceCount; i++) {
		LEDDeviceRec* device = &deviceList[i];
		DimInfoRec* dimInfo = &dimmerData[i];

		SerPrintf("*** Creating \'%s\' on pin %d\n", device->name, device->outputPin);

		SPAN_ACCESSORY(device->name);
			switch (device->type) {
				case Device_Switch:
					new DigitalPin(device, dimInfo);
					break;
				case Device_Fan:
					new FanPWM(device, dimInfo);
					break;
				case Device_Button:
					new HomeButton(device, dimInfo);
					break;
				default:
					new DimmableLED(device, dimInfo);
					break;
			}
	}
}

void setInitialPins() {
	for (auto i=0; i<deviceCount; i++) {
		LEDDeviceRec* device = &deviceList[i];
		switch (device->type) {
			case Device_Switch:
			case Device_Fan:
			case Device_LED:
				pinMode(device->outputPin, OUTPUT);
				digitalWrite(device->outputPin, LOW);
				break;
			default:
				break;
		}
	}
}

//////////////////////////////////////////////

const char* stringForPDReference(HUSB238_PDSelection ref) {
	switch (ref) {
		case PD_SRC_5V: return "5V";
		case PD_SRC_9V: return "9V";
		case PD_SRC_12V: return "12V";
		case PD_SRC_15V: return "15V";
		case PD_SRC_18V: return "18V";
		case PD_SRC_20V: return "20V";
		default: return "Unknown Voltage";
	}
}

bool isPDReferenceValid(HUSB238_PDSelection ref) {
	switch (ref) {
		case PD_SRC_5V:
		case PD_SRC_9V:
		case PD_SRC_12V:
		case PD_SRC_15V:
		case PD_SRC_18V:
		case PD_SRC_20V:
			return true;
		default:
			return false;
	}
}

const char* stringForVoltageDetected(HUSB238_VoltageSetting voltage) {
	switch (voltage) {
		case PD_5V: return "5V Detected";
		case PD_9V: return "9V Detected";
		case PD_12V: return "12V Detected";
		case PD_15V: return "15V Detected";
		case PD_18V: return "18V Detected";
		case PD_20V: return "20V Detected";
		default: return "Unknown Voltage Detected";
	}
}

const char* stringForCurrent(HUSB238_CurrentSetting current) {
	switch (current) {
		case CURRENT_0_5_A: return "0.5A";
		case CURRENT_0_7_A: return "0.7A";
		case CURRENT_1_0_A: return "1.0A";
		case CURRENT_1_25_A: return "1.25A";
		case CURRENT_1_5_A: return "1.5A";
		case CURRENT_1_75_A: return "1.75A";
		case CURRENT_2_0_A: return "2.0A";
		case CURRENT_2_25_A: return "2.25A";
		case CURRENT_2_50_A: return "2.50A";
		case CURRENT_2_75_A: return "2.75A";
		case CURRENT_3_0_A: return "3.0A";
		case CURRENT_3_25_A: return "3.25A";
		case CURRENT_3_5_A: return "3.5A";
		case CURRENT_4_0_A: return "4.0A";
		case CURRENT_4_5_A: return "4.5A";
		case CURRENT_5_0_A: return "5.0A";
		default: return "Unknown Current";
	}
}

const char* stringForPDResponse(HUSB238_ResponseCodes response) {
	switch (response) {
		case NO_RESPONSE: return "No response";
		case SUCCESS: return "Success";
		case INVALID_CMD_OR_ARG: return "Invalid command or argument";
		case CMD_NOT_SUPPORTED: return "Command not supported";
		case TRANSACTION_FAIL_NO_GOOD_CRC: return "Transaction fail";
		default: return "Unknown response code";
	}
}

void printPDStatus() {
	SerPrintf("USB PD is set to %s\n", stringForPDReference(husb238.getSelectedPD()));
	SerPrintf("Current USB Voltage: %s @ %s\n", stringForVoltageDetected(husb238.getPDSrcVoltage()), stringForCurrent(husb238.getPDSrcCurrent()));
}

void cmdScanPD(const char* buf = NULL) {
	// Determine whether attached or unattached
	bool attached = husb238.isAttached();
	SerPrintf("Attachment Status: %s\n", attached ? "Attached" : "Unattached");

	if (! attached) return;
	
	// Test getCCStatus function
	bool ccStatus = husb238.getCCdirection();
	SerPrintf("CC Direction: %s\n",ccStatus ? "CC1 connected" : "CC2 Connected");

	// Check if we can get responses to our PD queries!
	HUSB238_ResponseCodes pdResponse = husb238.getPDResponse();
	SerPrintf("USB PD query response: %s\n", stringForPDResponse(pdResponse));

	if (pdResponse != SUCCESS)
		return;
	
	Serial.println();

	// What voltages and currents are available from this adapter?
	Serial.println("Available PD Voltages and Current Detection Test:");
	for (int i = PD_SRC_5V; i <= PD_SRC_20V; i++) {
		if (isPDReferenceValid((HUSB238_PDSelection)i)) {
			bool voltageDetected = husb238.isVoltageDetected((HUSB238_PDSelection)i);
			HUSB238_CurrentSetting currentDetected = CURRENT_0_5_A;

			if (voltageDetected) {
				currentDetected = husb238.currentDetected((HUSB238_PDSelection)i);
			}

			SerPrintf("Voltage %s:  - %s%s\n", stringForPDReference((HUSB238_PDSelection)i), voltageDetected ? " Available - Max Current: " : " Unavailable", voltageDetected ? stringForCurrent(currentDetected) : "");
		}
	}

	Serial.println();

	printPDStatus();
}

uint32_t lastVoltageCheck = 0;
uint32_t voltageSetStart = 0;
HUSB238_PDSelection currentPD = PD_SRC_12V;
bool voltageSetupComplete = false;

void set12V() {
	if (husb238.isAttached()) {
		husb238.selectPD(PD_SRC_12V);
		husb238.requestPD();
		voltageSetStart = millis64();
	}
	else {
		voltageSetupComplete = true;
	}
}

void updateVoltage() {
	if (!voltageSetupComplete) {
		uint32_t now = millis();

		if (now - lastVoltageCheck > 100) {
			lastVoltageCheck = now;

			HUSB238_VoltageSetting voltage = husb238.getPDSrcVoltage();

			if ((voltage == PD_12V && currentPD == PD_SRC_12V) || (voltage == PD_9V && currentPD == PD_SRC_9V)) {
				voltageSetupComplete = true;
				if (voltage == PD_9V) {
					availableOutputVoltage = output9V;
				}
				printPDStatus();
			}
			else if (now - voltageSetStart > 1000) {
				if (currentPD == PD_SRC_12V) {
					SerPrintf("12V not available, trying 9V...\n");
					currentPD = PD_SRC_9V;
					husb238.selectPD(PD_SRC_9V);
					husb238.requestPD();
					voltageSetStart = now;
				}
				else {
					SerPrintf("Unable to set USB PD voltage to 12V or 9V.  Please check power supply.");
					printPDStatus();
					voltageSetupComplete = true;
					availableOutputVoltage = outputOther;
				}
			}
		}
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

void cmdShowCPUStats(const char *buff = NULL) {
	SerPrintf("\n*** CPU Stats ***\n\n");
	SerPrintf("CPU frequency: %ldMHz\n", getCpuFrequencyMhz());
	SerPrintf("Xtal frequency: %ldMHz\n", getXtalFrequencyMhz());
	SerPrintf("Bus frequency: %ldMHz\n", getApbFrequency()/1000000);

	uint32_t time = micros();
	float f = 0;
	for (auto i=0; i<1000; i++) {
		f += gammaCorrect(50.0 + (float)i/100.0);
	}
	float duration = (micros() - time) / 1000.0 + f * 1e-40;	// multiply f to basically zero, avoids unused variable warning
	SerPrintf("Timing test: pow(x) = %0.2fuS\n", duration);

	SerPrintf("Total heap: %ld\n", ESP.getHeapSize());
	SerPrintf("Free heap: %ld\n", ESP.getFreeHeap());
	SerPrintf("Total PSRAM: %ld\n", ESP.getPsramSize());
	SerPrintf("Free PSRAM: %ld\n", ESP.getFreePsram());

	SerPrintf("\n*** CPU Stats ***\n\n");
}

void addCommands() {
	new SpanUserCommand('s',"show CPU stats", cmdShowCPUStats);
	new SpanUserCommand('u',"update accessory database", cmdUpdateAccessories);
	new SpanUserCommand('d',"scan USB PD", cmdScanPD);
}

//////////////////////////////////////////////

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

void setup() {
	setInitialPins();

	setupIndicator();

	flashIndicator(flashColor, 20, 100);
	setIndicator(startColor, true);

	Serial.begin(115200);
	SerPrintf("Home-LED Startup\n");

	SerPrintf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setConnectionCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.enableOTA(otaPassword);
	homeSpan.begin(Category::Bridges, displayName, DEFAULT_HOST_NAME, modelName);

	SerPrintf("Create devices\n");
	createDevices();
	addCommands();

	SerPrintf("Create dimmer task\n");
	xTaskCreate(dimmerTask, "LED_Dimmer", dimmerTaskStackSize, NULL, 0, &t);

	SerPrintf("Wait for WiFi...\n");
	setIndicator(connectingColor, true);

	#ifdef CPU_FREQUENCY
		setCpuFrequencyMhz(CPU_FREQUENCY);
	#endif

	SerPrintf("Initialize HUSB238...\n");
	if (husb238.begin(HUSB238_I2CADDR_DEFAULT, &Wire)) {
		set12V();
	} else {
		Serial.println("Couldn't find HUSB238, check your wiring?");
	}

  SerPrintf("Init complete.\n");
}

void loop() {
	homeSpan.poll();
	updateVoltage();
}
#include "Arduino.h"

#include "HomeSpan.h" 
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_DotStar.h>

#ifdef DEBUG
#define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//////////////////////////////////////////////

constexpr const char* versionString = "v0.3";

//////////////////////////////////////////////

constexpr uint32_t ledUpdateRate = 100;
constexpr uint32_t ledUpdateRateInterval = 1000 / ledUpdateRate;

constexpr uint32_t defaultFadeUpTime = 500;
constexpr uint32_t defaultFadeDownTime = 2000;
constexpr uint32_t defaultFadeFastTime = 300;

constexpr float homeKitBrightnessMax = 100.0;

constexpr uint32_t dimmerTaskStackSize = 2000;

//////////////////////////////////////////////

constexpr float ledGamma = 2.8;
constexpr uint8_t maxIndicatorValue = 50;

constexpr uint8_t ledStatusLevel = 0x03;

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
	Strip_RGB = 0,
	Strip_RGBW,
	Strip_W,

	Strip_DotStar_RGB,
	Strip_DotStar_W,

	Strip_LED,

	Strip_Switch,
	Strip_Fan,
} StripType;

class SetValue {
	public:
		SetValue(StripType type=Strip_RGB) : _type(type) {};
		virtual void setValue(float value) {};
		uint32_t brightnessToColor(float value) {
			switch (_type) {
				case Strip_RGB:
				case Strip_DotStar_RGB: {
					uint8_t w = value / homeKitBrightnessMax * 255;
					return w<<16 | w<<8 | w;
				}
				case Strip_RGBW: {
					uint8_t w = value / homeKitBrightnessMax * 255;
					return w<<24;
				}
				case Strip_W:
				case Strip_DotStar_W: {
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

class PixelWithSetValue : public Adafruit_NeoPixel , public SetValue {
	public:
		PixelWithSetValue(int pin, int pixelCount, StripType type=Strip_RGB) : Adafruit_NeoPixel(pixelCount, pin, ((type==Strip_RGBW) ? NEO_GRBW : NEO_GRB) + NEO_KHZ800), SetValue(type) { };
		void setValue(float value) {
			fill(brightnessToColor(value));
			show();
		};
};

class DotWithSetValue : public Adafruit_DotStar , public SetValue {
	public:
		DotWithSetValue(uint8_t dataPin, uint8_t clockPin, int pixelCount, StripType type=Strip_RGB) : Adafruit_DotStar(pixelCount, dataPin, clockPin), SetValue(type) { };
		void setValue(float value) {
			fill(brightnessToColor(value));
			show();
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
				type(type), name(name), outputPin(outputPin), count(count), buttonPin(buttonPin), 
				fadeUpDuration(fadeUpDuration), fadeDownDuration(fadeDownDuration), fadeFastDuration(fadeFastDuration) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t clockPin, int16_t buttonPin, uint16_t fadeUpDuration, uint16_t fadeDownDuration, uint16_t fadeFastDuration) : 
				type(type), name(name), outputPin(outputPin), count(count), clockPin(clockPin), buttonPin(buttonPin), 
				fadeUpDuration(fadeUpDuration), fadeDownDuration(fadeDownDuration), fadeFastDuration(fadeFastDuration) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, int16_t buttonPin) : 
				type(type), name(name), outputPin(outputPin), buttonPin(buttonPin) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t buttonPin) : 
				type(type), name(name), outputPin(outputPin), count(count), buttonPin(buttonPin) { }
	LEDDeviceRec(StripType type, const char* name, const uint8_t outputPin, uint8_t count, int16_t clockPin, int16_t buttonPin) : 
				type(type), name(name), outputPin(outputPin), count(count), clockPin(clockPin), buttonPin(buttonPin) { }
} LEDDeviceRec;

// Strip_RGB/Strip_RGBW/Strip_W, "name", outputPin, buttonPin
// Strip_RGB/Strip_RGBW/Strip_W, "name", outputPin, count, buttonPin
// Strip_RGB/Strip_RGBW/Strip_W, "name", outputPin, buttonPin, fadeUpDuration, fadeDownDuration, fadeFastDuration
// Strip_RGB/Strip_RGBW/Strip_W, "name", outputPin, count, buttonPin, fadeUpDuration, fadeDownDuration, fadeFastDuration
//
// Strip_Dotstar_RGB/Strip_DotStar_W, "name", "dataPin", count, clockPin, buttonPin
// Strip_Dotstar_RGB/Strip_DotStar_W, "name", outputPin, count, clockPin, buttonPin, fadeUpDuration, fadeDownDuration, fadeFastDuration
//
// Strip_Switch, "name", outputPin, buttonPin
// Strip_LED, "name", outputPin, buttonPin
// Strip_Fan, "name", outputPin, unused, neoPixelPin, buttonPin

#ifdef TINYS3
LEDDeviceRec deviceList[] = {
	// { Strip_RGB, "LED1", 8, 36 },

	// { Strip_RGB, "LED1", 8, 6 },
	// { Strip_RGB, "LED2", 7, -1 },

	{ Strip_Fan, "Fan", 2, 0, 3, 4 },
};
#elif QTPYS3
LEDDeviceRec deviceList[] = {
	// { Strip_RGB, "LED1", MISO, 0 },
};
#endif

constexpr auto deviceCount = sizeof(deviceList) / sizeof(LEDDeviceRec);

//////////////////////////////////////////////

#ifdef TINYS3
constexpr uint8_t indicatorDataPin = 18;
constexpr uint8_t indicatorPowerPin = 17;
#elif QTPYS3
constexpr uint8_t indicatorDataPin = PIN_NEOPIXEL;
constexpr uint8_t indicatorPowerPin = NEOPIXEL_POWER;
#endif

Adafruit_NeoPixel indicator(1, indicatorDataPin);

void setIndicator(uint32_t color, bool saveColor = false) {
	if (color != currentIndicatorColor) {
		currentIndicatorColor = color;
		if (saveColor) {
			savedIndicatorColor = color;
		}
		if (color & whiteColorFlag) {
			color &= 0xFF;
			indicator.setPixelColor(0, color, color, color);
		}
		else {
			indicator.setPixelColor(0, color>>16, (color >> 8) & 0xFF, color & 0xFF);
		}
		indicator.show();
	}
}

void setIndicator8(uint8_t color8) {
	setIndicator(whiteColorFlag | color8);
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

		if (firstValue != currentIndicatorValue) {
			if (firstValue == -1) {
				setIndicator(savedIndicatorColor);
			}
			else {
				uint8_t value255 = firstValue * maxIndicatorValue / homeKitBrightnessMax;
				setIndicator8(max((uint8_t)1, value255));
			}
			currentIndicatorValue = firstValue;
		}
	}
}

//////////////////////////////////////////////

float fadeStep(uint16_t duration, uint16_t defaultValue) {
	return homeKitBrightnessMax * ledUpdateRateInterval / (duration==0 ? defaultValue : duration);
}

struct DimmableLED : Service::LightBulb {
	SpanCharacteristic *_on;
	SpanCharacteristic *_brightness;
	
	SpanButton* _button = NULL;
	DimInfoRec* _dimInfo;

	float _fadeUpStep;
	float _fadeDownStep;
	float _fadeFastStep;

	DimmableLED(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::LightBulb() {
		_on = new Characteristic::On();
		_brightness = new Characteristic::Brightness(100);
		_brightness->setRange(0, 100, 1);

		_dimInfo = dimInfo;

		switch (device->type) {
			case Strip_RGB:
			case Strip_RGBW:
			case Strip_W:
				_dimInfo->led = new PixelWithSetValue(device->outputPin, device->count, device->type);
				break;
			case Strip_DotStar_RGB:
			case Strip_DotStar_W:
				_dimInfo->led = new DotWithSetValue(device->outputPin, device->clockPin, device->count, device->type);
				break;
			case Strip_LED:
			default:
				_dimInfo->led = new LedPinWithSetValue(device->outputPin, 0, 20000);
				break;
		}

		_fadeUpStep = fadeStep(device->fadeUpDuration, defaultFadeUpTime);
		_fadeDownStep = fadeStep(device->fadeDownDuration, defaultFadeDownTime);
		_fadeFastStep = fadeStep(device->fadeFastDuration, defaultFadeFastTime);

		if (device->buttonPin != -1) {
			_button = new SpanButton(device->buttonPin);
		}
	}

	void refreshLevel(bool fast = false) {
		float currentLevel = _dimInfo->currentLevel;
		float newLevel = _on->getNewVal() * _brightness->getNewVal();

		if (newLevel != currentLevel) {
			float step;

			if (fast) {
				step = (newLevel >= currentLevel) ? _fadeFastStep : -_fadeFastStep;
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
		refreshLevel(_brightness->updated());
		return(true);  
	}

	void button(int pin, int pressType) {
		if (pressType == SpanButton::SINGLE) {
			bool newOn = !_on->getVal();
			_on->setVal(newOn);
			if (newOn && _brightness->getVal() == 0) {
				_brightness->setVal(100);
			}
			refreshLevel();
		}
	}
};

//////////////////////////////////////////////
struct FanPWM : Service::Fan {
	SetValue* _output;
	SpanCharacteristic *_active;
	SpanCharacteristic *_speed;
	Adafruit_NeoPixel *_buttonLED = NULL;
	SpanButton* _button = NULL;
	DimInfoRec* _dimInfo;

	FanPWM(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::Fan() {
		_active = new Characteristic::Active(false);
		_speed = new Characteristic::RotationSpeed(0, true);

		_speed->setRange(0, 100, 25);

		_dimInfo = dimInfo;
		_output = new LedPinWithSetValue(device->outputPin, 0, 25000);

		if (device->clockPin != -1) {
			_buttonLED = new Adafruit_NeoPixel(1, device->clockPin, NEO_GRB + NEO_KHZ800);
			_buttonLED->begin();
			_buttonLED->show();
		}

		if (device->buttonPin != -1) {
			_button = new SpanButton(device->buttonPin);
		}

		update();
	}

	boolean update() {
		float newSpeed = _active->getNewVal() * _speed->getNewVal();
		constexpr uint8_t ledBright = 30;
		uint32_t color;

		if (newSpeed < 5) {
			color = MAKE_RGB(0, ledBright, 0);
			newSpeed = 0;
		}
		else if (newSpeed < 37) {
			color = MAKE_RGB(ledBright*4/10, ledBright*6/10, 0);
			newSpeed = 25;
		}
		else if (newSpeed < 62) {
			color = MAKE_RGB(ledBright*6/10, ledBright*4/10, 0);
			newSpeed = 50;
		}
		else if (newSpeed < 95) {
			color = MAKE_RGB(ledBright*8/10, ledBright*2/10, 0);
			newSpeed = 75;
		}
		else {
			color = MAKE_RGB(ledBright, 0, 0);
			newSpeed = 100;
		}
		if (_buttonLED) {
			_buttonLED->setPixelColor(0, color);
			_buttonLED->show();
		}
		_output->setValue(newSpeed);
		_dimInfo->currentLevel = newSpeed;
		_dimInfo->currentCorrectedLevel = gammaCorrect(_dimInfo->currentLevel) * 0.1;

		return true;
	}

	void button(int pin, int pressType) {
		if (pressType == SpanButton::SINGLE) {
			bool newActive = !_active->getVal();
			_active->setVal(newActive);
			if (newActive && _speed->getVal() == 0) {
				_speed->setVal(100);
			}
			update();
		}
	}
};

//////////////////////////////////////////////
struct DigitalPin : Service::Switch {
	int _pin;
	SpanButton* _button = NULL;
	SpanCharacteristic *_on;
	DimInfoRec* _dimInfo;

	DigitalPin(LEDDeviceRec* device, DimInfoRec* dimInfo) : Service::Switch(){
		_on = new Characteristic::On();
		_pin = device->outputPin;
		_dimInfo = dimInfo;
		pinMode(_pin,OUTPUT);
		digitalWrite(_pin, LOW);

		if (device->buttonPin != -1) {
			_button = new SpanButton(device->buttonPin);
		}
	}

	boolean update() {            
		bool value = _on->getNewVal();
		digitalWrite(_pin, value);
		_dimInfo->currentLevel = value * 100;
		_dimInfo->currentCorrectedLevel = _dimInfo->currentLevel;
		return(true);
	}

	void button(int pin, int pressType) {
		if (pressType == SpanButton::SINGLE) {
			_on->setVal(!_on->getVal());
			update();
		}
	}
};
	  
//////////////////////////////////////////////

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

	for (auto i=0; i<deviceCount; i++) {
		LEDDeviceRec* device = &deviceList[i];
		DimInfoRec* dimInfo = &dimmerData[i];

		SerPrintf("Creating \'%s\' on pin %d\n", device->name, device->outputPin);

		SPAN_ACCESSORY(device->name);
			switch (device->type) {
				case Strip_Switch:
					new DigitalPin(device, dimInfo);
					break;
				case Strip_Fan:
					new FanPWM(device, dimInfo);
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
			case Strip_Switch:
			case Strip_Fan:
			case Strip_LED:
				pinMode(device->outputPin, OUTPUT);
				digitalWrite(device->outputPin, LOW);
				break;
		}
	}
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

void wifiReady() {
	if (!(currentIndicatorColor & whiteColorFlag)) {
		setIndicator(readyColor, true);
	}
	SerPrintf("WIFI Ready.\n");
}

void statusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		if (!(currentIndicatorColor & whiteColorFlag)) {
			setIndicator(connectingColor, true);
		}
		SerPrintf("Lost WIFI Connection...\n");
	}
}

TaskHandle_t t;

void setup() {
	setInitialPins();

	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);

	indicator.begin();
	flashIndicator(flashColor, 20, 200);
	setIndicator(startColor, true);

	SerBegin(115200);
	SerPrintf("Home-LED Startup\n");

	SerPrintf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.begin(Category::Bridges, "LED-Controller", DEFAULT_HOST_NAME, "LED-Controller-ESP32");

	SerPrintf("Create devices\n");
	createDevices();

	SerPrintf("Create dimmer task\n");
	xTaskCreate(dimmerTask, "LED_Dimmer", dimmerTaskStackSize, NULL, 0, &t);

	SerPrintf("Wait for WiFi...\n");

	setIndicator(connectingColor, true);

	SerPrintf("Init complete.\n");
}

void loop() {
	homeSpan.poll();
}
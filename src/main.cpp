#include "Arduino.h"

#include "HomeSpan.h" 
#include "Wire.h"
#include <Adafruit_NeoPixel.h>
#include <Adafruit_DotStar.h>
#include "ESPTelnetStream.h"

//////////////////////////////////////////////

class TelnetOutput : public ESPTelnetStream {
	public:
		size_t write(const uint8_t *buffer, size_t size) {
			Serial.write(buffer, size);
  			if (client && isConnected()) {
				size_t result = client.write(buffer, size);
				uint8_t c = buffer[size-1];
				if (c == 13) {
					client.write(10);
				}
				else if (c == 10) {
					client.write(13);
				}
				return result;
			}
			else {
				return -1;
			}
		}
		size_t write(uint8_t c) {
			return write(&c, 1);
		}
};

TelnetOutput telnet;

//////////////////////////////////////////////

#if defined(OTADEBUG) || defined(DEBUG)
#define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//////////////////////////////////////////////

constexpr const char* versionString = "v0.4";

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
constexpr uint8_t maxIndicatorValue = 20;

constexpr uint8_t ledStatusLevel = 0x01;

#define MAKE_RGB(r, g, b) (r<<16 | g<<8 | b)
constexpr uint32_t flashColor = MAKE_RGB(ledStatusLevel, 0, ledStatusLevel);
constexpr uint32_t startColor = MAKE_RGB(ledStatusLevel, 0, 0);
constexpr uint32_t connectingColor = MAKE_RGB(0, 0, ledStatusLevel);
constexpr uint32_t readyColor = MAKE_RGB(0, ledStatusLevel, 0);
constexpr uint32_t telnetColor = MAKE_RGB(0, ledStatusLevel, ledStatusLevel);

uint32_t currentIndicatorColor = 0xFFFFFFFF;
uint32_t savedIndicatorColor = 0;

constexpr uint32_t whiteColorFlag = 1 << 24;

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
					return w<<24;
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

class PixelWithSetValue : public Adafruit_NeoPixel , public SetValue {
	public:
		PixelWithSetValue(int pin, int pixelCount, StripType type=Device_RGB) : Adafruit_NeoPixel(pixelCount, pin, ((type==Device_RGBW) ? NEO_GRBW : NEO_GRB) + NEO_KHZ800), SetValue(type) { };
		void setValue(float value) {
			fill(brightnessToColor(value));
			show();
		};
};

class DotWithSetValue : public Adafruit_DotStar , public SetValue {
	public:
		DotWithSetValue(uint8_t dataPin, uint8_t clockPin, int pixelCount, StripType type=Device_RGB) : Adafruit_DotStar(pixelCount, dataPin, clockPin), SetValue(type) { };
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

#ifdef TINYS3
// LEDDeviceRec deviceList[] = {
// 	{ Device_RGB, "LED1", 8, 6 },
// };
// LEDDeviceRec deviceList[] = {
// 	{ Device_RGB, "LED2", 7, -1 },
// };

// Penguin Express Top Lighting Strip
// constexpr const char* displayName = "LED-Controller";
// constexpr const char* modelName = "LED-Controller-ESP32";
// LEDDeviceRec deviceList[] = {
// 	{ Device_LED, "LED", 8, 36 },
// 	{ Device_Button, "Button", 37 },
// };

constexpr const char* displayName = "Fan-Controller";
constexpr const char* modelName = "Fan-Controller-LR";
LEDDeviceRec deviceList[] = {
	{ Device_Fan, "Fan", 2, 0, 3, 4 },
};

#elif QTPYS3
// Penguin Light
// constexpr const char* displayName = "Penguin-Controller";
// constexpr const char* modelName = "Penguin-Controller-ESP32";
// LEDDeviceRec deviceList[] = {
// 	{ Device_LED, "Penguin", SCK, 0 },
// };

// Penguin Express Lower Accent Lighting
constexpr const char* displayName = "Accent-Controller";
constexpr const char* modelName = "Accent-Controller-ESP32";
LEDDeviceRec deviceList[] = {
	{ Device_LED, "Lower Accent", MISO, 0 },
};

// Penguin Express Dining Lighting
// constexpr const char* displayName = "Dining-Controller";
// constexpr const char* modelName = "Dining-Controller-ESP32";
// LEDDeviceRec deviceList[] = {
// 	{ Device_LED, "LED-Left", MOSI, SCK | SwitchToggleBit, 100, 800, defaultFadeFastTime },
// 	{ Device_LED, "LED-Right", MISO, RX | SwitchToggleBit, 100, 800, defaultFadeFastTime },
// };

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

uint64_t millis64() {
  static uint32_t low32 = 0, high32 = 0;
  uint32_t new_low32 = millis();

  if (new_low32 < low32)
    high32++;

  low32 = new_low32;

  return (uint64_t) high32 << 32 | low32;
}

class elapsedMillis64
{
  private:
    uint64_t ms;
  public:
    elapsedMillis64(void) { ms = millis64(); }
    elapsedMillis64(uint64_t val) { ms = millis64() - val; }
    elapsedMillis64(const elapsedMillis64 &orig) { ms = orig.ms; }
    operator uint64_t () const { return millis64() - ms; }
    elapsedMillis64 & operator = (const elapsedMillis64 &rhs) { ms = rhs.ms; return *this; }
    elapsedMillis64 & operator = (uint64_t val)       { ms = millis64() - val; return *this; }
    elapsedMillis64 & operator -= (uint64_t val)      { ms += val ; return *this; }
    elapsedMillis64 & operator += (uint64_t val)      { ms -= val ; return *this; }
    elapsedMillis64 operator - (int val) const            { elapsedMillis64 r(*this); r.ms += val; return r; }
    elapsedMillis64 operator - (unsigned int val) const   { elapsedMillis64 r(*this); r.ms += val; return r; }
    elapsedMillis64 operator - (long val) const           { elapsedMillis64 r(*this); r.ms += val; return r; }
    elapsedMillis64 operator - (unsigned long val) const  { elapsedMillis64 r(*this); r.ms += val; return r; }
    elapsedMillis64 operator - (int64_t val) const        { elapsedMillis64 r(*this); r.ms += val; return r; }
    elapsedMillis64 operator - (uint64_t val) const       { elapsedMillis64 r(*this); r.ms += val; return r; }
    elapsedMillis64 operator + (int val) const            { elapsedMillis64 r(*this); r.ms -= val; return r; }
    elapsedMillis64 operator + (unsigned int val) const   { elapsedMillis64 r(*this); r.ms -= val; return r; }
    elapsedMillis64 operator + (long val) const           { elapsedMillis64 r(*this); r.ms -= val; return r; }
    elapsedMillis64 operator + (unsigned long val) const  { elapsedMillis64 r(*this); r.ms -= val; return r; }
    elapsedMillis64 operator + (int64_t val) const        { elapsedMillis64 r(*this); r.ms -= val; return r; }
    elapsedMillis64 operator + (uint64_t val) const       { elapsedMillis64 r(*this); r.ms -= val; return r; }
};

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
	elapsedMillis64 lastChange;

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

		constexpr uint32_t indicatorOutTime = 5 * 60 * 1000;

		if (firstValue == -1 && lastChange > indicatorOutTime) {
			firstValue = -2;
		}

		if (firstValue != currentIndicatorValue) {
			if (firstValue == -1) {
				setIndicator(savedIndicatorColor);
				lastChange = 0;
			}
			else if (firstValue == -2) {
				setIndicator(0);
			}
			else {
				uint8_t value255 = firstValue * maxIndicatorValue / homeKitBrightnessMax;
				setIndicator8(max((uint8_t)1, value255));
				lastChange = 0;
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

		if (device->buttonPin != -1) {
			if (device->buttonPin & SwitchToggleBit) {
				_button = new SpanToggle(device->buttonPin);
			}
			else {
				_button = new SpanButton(device->buttonPin);
			}
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

		_speed->setRange(0, 100, 1);

		_dimInfo = dimInfo;
		_output = new LedPinWithSetValue(device->outputPin, 0, 25000);

		if (device->clockPin != -1) {
			_buttonLED = new Adafruit_NeoPixel(1, device->clockPin, NEO_GRB + NEO_KHZ800);
			_buttonLED->begin();
			_buttonLED->show();
		}

		if (device->buttonPin != -1) {
			if (device->buttonPin & SwitchToggleBit) {
				_button = new SpanToggle(device->buttonPin);
			}
			else {
				_button = new SpanButton(device->buttonPin);
			}
		}

		update();
	}

	boolean update() {
		float newSpeed = _active->getNewVal() * _speed->getNewVal();
		constexpr uint8_t ledBright = 30;
		uint32_t color = MAKE_RGB(0, ledBright/3, 0);

		if (newSpeed > 0) {
			uint8_t rL = ledBright*6/10;
			uint8_t rH = ledBright;
			uint8_t gL = ledBright*6/10;
			uint8_t gH = 0;
			uint8_t r = rL + (rH - rL) * newSpeed / homeKitBrightnessMax;
			uint8_t g = gL + (gH - gL) * newSpeed / homeKitBrightnessMax;

			color = MAKE_RGB(r, g, 0);
			newSpeed = 10 + 90 * newSpeed / homeKitBrightnessMax;
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
			if (device->buttonPin & SwitchToggleBit) {
				_button = new SpanToggle(device->buttonPin);
			}
			else {
				_button = new SpanButton(device->buttonPin);
			}
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

		SerPrintf("Creating \'%s\' on pin %d\n", device->name, device->outputPin);

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
		}
	}
}

//////////////////////////////////////////////

void telnetConnected(String ip) {
  SerPrintf("%s connected.\n", ip);
  setIndicator(telnetColor, true);
  homeSpan.outputStream = &telnet;
}

void telnetDisconnected(String ip) {
  SerPrintf("%s disconnected.\n", ip);
  setIndicator(readyColor, true);
  homeSpan.outputStream = &Serial;
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
		SerPrintf("Lost WIFI Connection...\n");
	}
}

void wifiReady() {
	if (!(currentIndicatorColor & whiteColorFlag)) {
		setIndicator(readyColor, true);
	}
	SerPrintf("WIFI Ready.\n");

	SerPrintf("Telnet.begin: ");
	telnet.onConnect(telnetConnected);
	telnet.onDisconnect(telnetDisconnected);
	telnet.onReconnect(telnetConnected);
	if(telnet.begin()) {
		SerPrintf("Successful\n");
	} else {
		SerPrintf("Failed\n");
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

	Serial.begin(115200);
	SerPrintf("Home-LED Startup\n");

	SerPrintf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.setStatusCallback(statusChanged);
	homeSpan.enableOTA(otaPassword);
	homeSpan.begin(Category::Bridges, displayName, DEFAULT_HOST_NAME, modelName);

	SerPrintf("Create devices\n");
	createDevices();

	SerPrintf("Create dimmer task\n");
	xTaskCreate(dimmerTask, "LED_Dimmer", dimmerTaskStackSize, NULL, 0, &t);

	SerPrintf("Wait for WiFi...\n");

	setIndicator(connectingColor, true);

	SerPrintf("Init complete.\n");
}

void loop() {
	telnet.loop();
	homeSpan.poll();
}
#include "Arduino.h"

#include "HomeSpan.h" 
#include "extras/Pixel.h"

#ifdef DEBUG
#define SerPrintf(...) Serial.printf(__VA_ARGS__)
#define SerBegin(...) Serial.begin(__VA_ARGS__)
#else
#define SerPrintf(...)
#define SerBegin(...)
#endif

//////////////////////////////////////////////

constexpr const char* versionString = "v0.2";

//////////////////////////////////////////////

typedef struct LEDDeviceRec {
    uint8_t ledPin;
    int16_t buttonPin;
    const char* name;
    uint16_t fadeUpDuration;
    uint16_t fadeDownDuration;
    uint16_t fadeFastDuration;

	LEDDeviceRec(const uint8_t ledPin, int16_t buttonPin, const char* name, uint16_t fadeUpDuration, uint16_t fadeDownDuration, uint16_t fadeFastDuration) : 
				ledPin(ledPin), buttonPin(buttonPin), name(name), 
				fadeUpDuration(fadeUpDuration), fadeDownDuration(fadeDownDuration), fadeFastDuration(fadeFastDuration) { }
	LEDDeviceRec(const uint8_t ledPin, int16_t buttonPin, const char* name) : 
				ledPin(ledPin), buttonPin(buttonPin), name(name), 
				fadeUpDuration(0), fadeDownDuration(0), fadeFastDuration(0) { }
} LEDDeviceRec;

#ifdef TINYS3
LEDDeviceRec deviceList[] = {
    { 8, 36, "LED1" },

    // { 8, 6, "LED1" },
    // { 7, -1, "LED2" },
};
#elif QTPYS3
LEDDeviceRec deviceList[] = {
    { MISO, 0, "LED1" },
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

Pixel indicator(indicatorDataPin);

void setIndicator(uint32_t color, bool saveColor = false) {
	if (color != currentIndicatorColor) {
		currentIndicatorColor = color;
		if (saveColor) {
			savedIndicatorColor = color;
		}
		if (color & whiteColorFlag) {
			color &= 0xFF;
			indicator.set(Pixel::RGB(color, color, color));
		}
		else {
			indicator.set(Pixel::RGB(color>>16, (color >> 8) & 0xFF, color & 0xFF));
		}
	}
}

void setIndicator8(uint8_t color8) {
	setIndicator(whiteColorFlag | color8);
}

//////////////////////////////////////////////

typedef struct {
	LedPin* led;

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
				dimInfo->led->set(dimInfo->currentCorrectedLevel);
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
		_dimInfo->led = new LedPin(device->ledPin, 0, 20000);

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

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

	for (auto i=0; i<deviceCount; i++) {
		LEDDeviceRec* device = &deviceList[i];
		SerPrintf("Creating \'%s\' on pin %d\n", device->name, device->ledPin);
		SPAN_ACCESSORY(device->name);
			new DimmableLED(device, &dimmerData[i]);
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
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);

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

	setIndicator(connectingColor, true);

	SerPrintf("Init complete.\n");
}

void loop() {
	homeSpan.poll();
}
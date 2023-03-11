#include "Arduino.h"

#include "HomeSpan.h" 
#include "extras/Pixel.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v0.1";

//////////////////////////////////////////////

typedef struct {
    int16_t ledPin;
    int16_t buttonPin;
    const char* name;
    uint16_t fadeUpDuration;
    uint16_t fadeDownDuration;
    uint16_t fadeFastDuration;
} LEDDeviceRec;

#ifdef TINYS3
LEDDeviceRec deviceList[] = {
    { 8, 36, "LED1", 0, 0, 0 },

    // { 8, 6, "LED1", 0, 0, 0 },
    // { 7, -1, "LED2", 0, 0, 0 },
};
#elif QTPYS3
LEDDeviceRec deviceList[] = {
    { A0, 0, "LED1", 0, 0, 0 },
};
#endif

//////////////////////////////////////////////

#ifdef TINYS3
constexpr uint8_t indicatorDataPin = 18;
constexpr uint8_t indicatorPowerPin = 17;
#elif QTPYS3
constexpr uint8_t indicatorDataPin = PIN_NEOPIXEL;
constexpr uint8_t indicatorPowerPin = NEOPIXEL_POWER;
#endif

Pixel indicator(indicatorDataPin);

//////////////////////////////////////////////

constexpr uint32_t ledFadeRate = 100;
constexpr uint32_t ledFadeRateInterval = 1000 / ledFadeRate;

constexpr uint32_t fadeUpTime = 500;
constexpr uint32_t fadeDownTime = 2000;
constexpr uint32_t fadeFastTime = 300;

constexpr float homeKitBrightnessMax = 100.0;

constexpr float ledGamma = 2.8;
constexpr uint8_t maxIndicatorValue = 50;

constexpr uint8_t ledLevel = 0x03;
Pixel::Color flashColor = Pixel::RGB(ledLevel, 0, ledLevel);
Pixel::Color startColor = Pixel::RGB(ledLevel, 0, 0);
Pixel::Color connectingColor = Pixel::RGB(0, 0, ledLevel);
Pixel::Color readyColor = Pixel::RGB(0, ledLevel, 0);

typedef struct {
	LedPin* led;
	volatile float targetLevel = 0;
	volatile float currentLevel = 0;
	volatile float levelStep = 0;
} DimInfoRec;

volatile DimInfoRec* currentDimmer = NULL;

inline float gammaCorrect(float value) {
	return pow(value / homeKitBrightnessMax, ledGamma) * homeKitBrightnessMax;
}

void dimmerTask(void* params) {
	DimInfoRec *dimInfo = (DimInfoRec*)params;

	TickType_t lastTicks = xTaskGetTickCount();

	while (true) {
		xTaskDelayUntil(&lastTicks, pdMS_TO_TICKS(ledFadeRateInterval));

		if (dimInfo->levelStep) {
			dimInfo->currentLevel += dimInfo->levelStep;

			if ((dimInfo->levelStep > 0 && dimInfo->currentLevel >= dimInfo->targetLevel) || (dimInfo->levelStep < 0 && dimInfo->currentLevel <= dimInfo->targetLevel)) {
				dimInfo->currentLevel = dimInfo->targetLevel;
				dimInfo->levelStep = 0;
			}
			float newValue = gammaCorrect(dimInfo->currentLevel);
			dimInfo->led->set(newValue);

			if (newValue == 0) {
				if (currentDimmer == dimInfo) {
					indicator.set(readyColor);
					currentDimmer = NULL;
				}
			}
			else {
				if (currentDimmer == NULL) {
					currentDimmer = dimInfo;
				}
				if (currentDimmer == dimInfo) {
					uint8_t value255 = newValue * maxIndicatorValue / homeKitBrightnessMax;
					indicator.set(Pixel::RGB(value255, value255, value255));
				}
			}
		}
	}
}

#define FADE_STEP(duration) (homeKitBrightnessMax * ledFadeRateInterval / duration)

float fadeStep(uint16_t duration, uint16_t defaultValue) {
	if (duration > 0) {
		return FADE_STEP(duration);
	}
	else {
		return FADE_STEP(defaultValue);
	}
}

struct DimmableLED : Service::LightBulb {
	SpanCharacteristic *_on;
	SpanCharacteristic *_brightness;
	
	SpanButton* _button;
	DimInfoRec _dimInfo;

	float _fadeUpStep;
	float _fadeDownStep;
	float _fadeFastStep;

	DimmableLED(LEDDeviceRec* device) : Service::LightBulb() {
		_on = new Characteristic::On();
		_brightness = new Characteristic::Brightness(100);
		_brightness->setRange(0, 100, 1);

		_dimInfo.led = new LedPin(device->ledPin, 0, 20000);

		_fadeUpStep = fadeStep(device->fadeUpDuration, fadeUpTime);
		_fadeDownStep = fadeStep(device->fadeDownDuration, fadeDownTime);
		_fadeFastStep = fadeStep(device->fadeFastDuration, fadeFastTime);

		if (device->buttonPin != -1) {
			_button = new SpanButton(device->buttonPin);
		}

		TaskHandle_t t;
		xTaskCreate(dimmerTask, "LED_Dimmer", 10000, &_dimInfo, 0, &t);
	}

	void setNewLevel(float newLevel, bool immediate=false) {
		_dimInfo.levelStep = 0;

		if (newLevel != _dimInfo.currentLevel) {
			float step;

			if (immediate) {
				step = (newLevel >= _dimInfo.currentLevel) ? _fadeFastStep : -_fadeFastStep;
			}
			else {
				step = (newLevel >= _dimInfo.currentLevel) ? _fadeUpStep : -_fadeDownStep;
			}

			_dimInfo.targetLevel = newLevel;
			_dimInfo.levelStep = step;
			Serial.printf("setLevel %0.1f, step=%f.\n", newLevel, step);
		}
	}

	void refreshLevel(bool immediate = false) {
		setNewLevel(_on->getNewVal() * _brightness->getNewVal(), immediate);
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

constexpr uint16_t maxDevices = sizeof(deviceList) / sizeof(LEDDeviceRec);

DimmableLED* services[maxDevices];
uint16_t deviceCount = 0;

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

	for (LEDDeviceRec device : deviceList) {
		Serial.printf("Creating \'%s\' on pin %d\n", device.name, device.ledPin);
		SPAN_ACCESSORY(device.name);
			services[deviceCount++] = new DimmableLED(&device);
	}
}

//////////////////////////////////////////////

void flashIndicator(Pixel::Color color, uint16_t count, uint16_t period) {
	Pixel::Color black = Pixel::RGB(0, 0, 0);

	for (auto i=0; i<count; i++) {
		indicator.set(color);
		delay(period/4);
		indicator.set(black);
		delay(period*3/4);
	}
}

void wifiReady() {
	indicator.set(readyColor);
}

void setup() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);

	flashIndicator(flashColor, 20, 200);
	indicator.set(startColor);

	Serial.begin(115200);
	Serial.printf("HomeSpan Remote LED - Startup\n");

	Serial.printf("Init HomeSpan\n");
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.begin(Category::Bridges, "LED-Controller", DEFAULT_HOST_NAME, "LED-Controller-ESP32");

	createDevices();

	indicator.set(connectingColor);

	Serial.printf("Init complete.\n");
}

void loop() {
	homeSpan.poll();
}
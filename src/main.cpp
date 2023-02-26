#include "Arduino.h"

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include <esp_task_wdt.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "devices.h"

#include "PushButton.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v0.1";

constexpr uint32_t watchDogTimerSeconds = 3;

constexpr uint8_t indicatorDataPin = 18;
constexpr uint8_t indicatorPowerPin = 17;

//////////////////////////////////////////////

constexpr uint32_t ledFadeRate = 100;
constexpr uint32_t ledFadeRateInterval = 1000 / ledFadeRate;

constexpr uint32_t fadeUpTime = 500;
constexpr uint32_t fadeDownTime = 2000;

constexpr float fadeUpStep = 100.0 * ledFadeRateInterval / fadeUpTime;
constexpr float fadeDownStep = -100.0 * ledFadeRateInterval / fadeDownTime;

constexpr uint32_t heatbeatRate = 2000;
constexpr uint32_t heartbeatBlinkTime = 20;

Adafruit_NeoPixel indicator(1, indicatorDataPin, NEO_GRB + NEO_KHZ800);

typedef struct {
	LedPin* led;
	volatile float targetLevel = 0;
	volatile float currentLevel = 0;
	volatile float levelStep = 0;
} DimInfoRec;

inline float gammaCorrect(float value) {
	return pow(value / 100.0, 2.8) * 100.0;
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
			dimInfo->led->set(gammaCorrect(dimInfo->currentLevel));
		}
	}
}

struct DimmableLED : Service::LightBulb {
	SpanCharacteristic *_on;
	SpanCharacteristic *_brightness = NULL;
	
	Button* _button = NULL;
	DimInfoRec _dimInfo;

	DimmableLED(LEDDeviceRec* device) : Service::LightBulb() {
		_on = new Characteristic::On();
		_brightness = new Characteristic::Brightness(0);
		_brightness->setRange(0, 100, 1);

		_dimInfo.led = new LedPin(device->ledPin);

		if (device->buttonPin != -1) {
			_button = new Button(device->buttonPin);
		}

		TaskHandle_t t;
		xTaskCreate(dimmerTask, "LED_Dimmer", 10000, &_dimInfo, 0, &t);
	}

	void setNewLevel(float newLevel, bool immediate=false) {
		if (immediate) {
			Serial.printf("setLevel %0.1f, immediate.\n", newLevel);
			_dimInfo.levelStep = 0;
			_dimInfo.targetLevel = newLevel;
			_dimInfo.currentLevel = newLevel;
			_dimInfo.led->set(gammaCorrect(newLevel));
		}
		else {
			_dimInfo.levelStep = 0;

			if (newLevel != _dimInfo.currentLevel) {
				float step = (newLevel >= _dimInfo.currentLevel) ? fadeUpStep : fadeDownStep;

				_dimInfo.targetLevel = newLevel;
				_dimInfo.levelStep = step;
				Serial.printf("setLevel %0.1f, step=%f.\n", newLevel, step);
			}
		}
	}

	void refreshLevel() {
		setNewLevel(_on->getNewVal() * _brightness->getNewVal());
	}
	
	boolean update() {
		refreshLevel();
		return(true);  
	}

	void loop() {
		if (_button && _button->checkPress() == shortPress) {
			_on->setVal(!_on->getVal());
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

void flashIndicator(uint32_t color, uint16_t count, uint16_t period) {
	for (auto i=0; i<count; i++) {
		indicator.setPixelColor(0, color);
		indicator.show();
		delay(period/4);
		indicator.setPixelColor(0, 0);
		indicator.show();
		delay(period*3/4);
	}
}

void wifiReady() {
	indicator.setPixelColor(0, indicator.Color(0, 0, 0x3F));
	indicator.show();
}

void setup() {
	pinMode(indicatorPowerPin, OUTPUT);
	digitalWrite(indicatorPowerPin, HIGH);

	indicator.begin();
	flashIndicator(indicator.Color(0x0F, 0, 0), 20, 200);

	indicator.setPixelColor(0, indicator.Color(0x3F, 0, 0));
	indicator.show();

	Serial.begin(115200);
	Serial.printf("HomeSpan Remote LED - Startup\n");

	Serial.printf("Init HomeSpan\n");
	homeSpan.setWifiCredentials(ssid, sspwd);
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.begin(Category::Bridges, "LED-Controller", DEFAULT_HOST_NAME, "LED-Controller-ESP32");

	createDevices();

	// Serial.printf("Setup Watchdog Timer\n");
	// esp_task_wdt_init(watchDogTimerSeconds, true); //enable panic so ESP32 restarts
	// esp_task_wdt_add(NULL); //add current thread to WDT watch

	Serial.printf("Init complete.\n");

	indicator.setPixelColor(0, indicator.Color(0, 0x3F, 0));
	indicator.show();
}

void loop() {
	static elapsedMillis heartbeatTime;

	// esp_task_wdt_reset();

	homeSpan.poll();
}
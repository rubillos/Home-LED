#include <stdint.h>

typedef struct {
    int16_t ledPin;
    int16_t buttonPin;
    const char* name;
} LEDDeviceRec;

LEDDeviceRec deviceList[] = {
    { 8, 6, "LED1" },
    { 7, -1, "LED2" },
};

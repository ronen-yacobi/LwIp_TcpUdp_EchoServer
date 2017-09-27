#ifndef BLINKLED_H_
#define BLINKLED_H_
#include "stm32f4xx.h"
// ----------------------------------------------------------------------------
typedef struct {
	uint32_t port;
	uint32_t bit;
	uint32_t active_low;
}blinkLed_t;

void initLeds(void);

void turnOn (uint8_t led);

void turnOff (uint8_t led);

void toggle (uint8_t led);

void isOn (uint8_t led);

// ----------------------------------------------------------------------------

#endif // BLINKLED_H_

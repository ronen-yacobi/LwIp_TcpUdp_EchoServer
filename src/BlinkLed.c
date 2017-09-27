/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "BlinkLed.h"

#define BLINK_GPIOx(_N)       ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))
#define BLINK_PIN_MASK(_N)    (1 << (_N))
#define BLINK_RCC_MASKx(_N)   (RCC_AHB1ENR_GPIOAEN << (_N))


#define BLINK_PORT_NUMBER         (1)
#define BLINK_PIN_NUMBER          (0)
#define BLINK_PIN_NUMBER_GREEN    (7)
#define BLINK_PIN_NUMBER_RED      (14)
#define BLINK_ACTIVE_LOW          (0)

blinkLed_t blinkLeds[2] =
{
	{ BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_GREEN, BLINK_ACTIVE_LOW },
	{ BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_RED, BLINK_ACTIVE_LOW },
};

// ----------------------------------------------------------------------------

void initLeds()
{
  for (size_t i = 0; i < (sizeof(blinkLeds) / sizeof(blinkLeds[0])); ++i)
  {
	  // Enable GPIO Peripheral clock
	  RCC->AHB1ENR |= BLINK_RCC_MASKx(blinkLeds[i].port);

	  GPIO_InitTypeDef GPIO_InitStructure;

	  // Configure pin in output push/pull mode
	  GPIO_InitStructure.Pin =  BLINK_PIN_MASK(blinkLeds[i].bit);
	  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	  GPIO_InitStructure.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init (BLINK_GPIOx(blinkLeds[i].port), &GPIO_InitStructure);

	  // Start with led turned off
	  turnOff(i);
  }
}

void turnOn (uint8_t led)
{
  if (blinkLeds[led].active_low)
    {
      BLINK_GPIOx(blinkLeds[led].port)->BSRR = BLINK_PIN_MASK(blinkLeds[led].bit + 16);
    }
  else
    {
      BLINK_GPIOx(blinkLeds[led].port)->BSRR = BLINK_PIN_MASK(blinkLeds[led].bit);
    }
}

void turnOff (uint8_t led)
{
  if (blinkLeds[led].active_low)
    {
      BLINK_GPIOx(blinkLeds[led].port)->BSRR = BLINK_PIN_MASK(blinkLeds[led].bit) ;
    }
  else
    {
      BLINK_GPIOx(blinkLeds[led].port)->BSRR = BLINK_PIN_MASK(blinkLeds[led].bit + 16);
    }
}


// ----------------------------------------------------------------------------

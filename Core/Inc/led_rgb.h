/*
 * MIT License
 *
 * Copyright (c) 2021 Jared Anderson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "stm32l4xx_hal.h"

/*
 * RGB led driven by PWM
 * Assume the PWM is set with a period of 8191 and that 50% duty cycle is with ADC_FS (4095)
 * Treat the LED like a PWM thing itself - blink is the period; blink_duty is the duty cycle
 * blink_count increments and resets when it hits the period
 * if the count is less than duty_cycle = the led is on, otherwise off
 * The PWM timer needs to be set up before calling init and passing the relevent config
 */
typedef struct led_rgb {
  uint16_t r, g, b;
  uint16_t on, blink, blink_duty, blink_count;
  TIM_HandleTypeDef *tim;
  uint32_t rchan, gchan, bchan;
} led_rgb_t;

void init_led_rgb(led_rgb_t *l, TIM_HandleTypeDef *tim, uint32_t rchan, uint32_t gchan, uint32_t bchan);
void process_led(led_rgb_t *l);

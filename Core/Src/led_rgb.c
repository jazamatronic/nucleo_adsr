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

#include "led_rgb.h"

void init_led_rgb(led_rgb_t *l, TIM_HandleTypeDef *tim, uint32_t rchan, uint32_t gchan, uint32_t bchan) {
  l->r = 0;
  l->g = 0;
  l->b = 0;
  l->on = 1;
  l->blink = 0;
  l->blink_duty  = 0;
  l->blink_count = 0;
  l->tim = tim;
  l->rchan = rchan;
  l->gchan = gchan;
  l->bchan = bchan;
}

void process_led(led_rgb_t *l) {
  if (l->blink_count <= l->blink) {
    l->blink_count++;
  } else {
    l->blink_count = 0;
  }
  l->on = (l->blink_count < l->blink_duty) ? 1 : 0; 
  __HAL_TIM_SET_COMPARE(l->tim, l->rchan, l->on ? l->r : 0);
  __HAL_TIM_SET_COMPARE(l->tim, l->gchan, l->on ? l->g : 0);
  __HAL_TIM_SET_COMPARE(l->tim, l->bchan, l->on ? l->b : 0);
}

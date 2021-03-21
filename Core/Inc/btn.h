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

// With a 10KHz sampling rate this should give us ms counters
#define BTN_TICK     10 

#define DBNCE_CNT    55
#define DBL_CLK_CNT  300
#define LNG_CLK_CNT  600

enum click {NONE, DBNCE_RISE, RISE, DBNCE_FALL, FALL, SINGLE, DBNCE_DBL, DOUBLE, LONG};

typedef struct btn {
  enum click meta_state; 
  enum click click_state; 
  uint16_t count, last_click_count;
} btn_t;

void init_btn(btn_t *b);
void process_btn(btn_t *b, uint16_t level);


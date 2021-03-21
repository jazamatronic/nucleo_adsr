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

#include "btn.h"

void init_btn(btn_t *b) {
  b->meta_state	 = NONE;
  b->click_state = NONE;
  b->count = 0;
  b->last_click_count = 0;
}

/*
 * Button process
 *
 * Use the meta_state to keep clicks in until everything is confirmed (no double or long)
 * Detect a click - wait for debounce to make sure it was a real click
 * Keep track of how long it's pressed for.
 * If it's short and there's not another click within DBL_CLK_CNT then it's a single click
 * If it stays pressed for LNG_CLK_CNT then it's a long click
 *
 * click_state only stays that way for one cycle so better act on it
 *
 */
void process_btn(btn_t *b, uint16_t level) {
  b->click_state = NONE;
  b->count++;


  switch (b->meta_state) {

    case NONE:
      if (level) {
	b->meta_state = DBNCE_RISE;
      	b->last_click_count = b->count;
      }
      break;

    case DBNCE_RISE:
      //check we're still high after debounce
      if ((uint16_t)(b->last_click_count + DBNCE_CNT) == b->count) {
	if (level) {
      	  b->meta_state = RISE;
	  b->last_click_count = b->count;
      	} else {
	  b->meta_state = NONE;
      	}
      }
      break;

    case RISE:
      if (level) {
	if ((uint16_t)(b->last_click_count + LNG_CLK_CNT) == b->count) {
      	  b->meta_state = LONG;
      	  b->click_state = LONG;
      	}
      } else {
	b->meta_state = DBNCE_FALL;
	b->last_click_count = b->count;
      }
      break;

    case DBNCE_FALL:
      if ((uint16_t)(b->last_click_count + DBNCE_CNT) == b->count) {
	if (level) {
	  // False fall - back to rise
      	  b->meta_state = RISE;
      	} else {
      	  b->meta_state = FALL;
      	  b->last_click_count = b->count;
	}
      }
      break;

    case FALL:
      if (level) {
	b->meta_state = DBNCE_DBL;
      	b->last_click_count = b->count;
      } else if ((uint16_t)(b->last_click_count + DBL_CLK_CNT) == b->count) {
	b->meta_state = NONE;
	b->click_state = SINGLE;
      }
      break;

    case DBNCE_DBL:
      if ((uint16_t)(b->last_click_count + DBNCE_CNT) == b->count) {
	if (level) {
      	  b->meta_state = NONE;
      	  b->click_state = DOUBLE;
	} else {
      	  b->meta_state = NONE;
      	  b->click_state = SINGLE;
	}
      }
      break;

    case LONG:
      if (level) {
	b->meta_state = LONG;
    	b->click_state = LONG;
      } else {
	b->meta_state = NONE;
      }
      break;
    
    default:
      b->meta_state = NONE;
      break;
      
  }

}

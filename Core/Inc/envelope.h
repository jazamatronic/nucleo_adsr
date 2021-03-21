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
#include "arm_math.h"

#define ADSR_TABLE_LEN	256
#define ENV_SR		10000 //sampling rate of around 10k - check TIM6 config in main.c
#define NUM_ENVS	3 
#define NUM_CURVES	3     //needs to match the number of enum curves
#define NUM_MODES	6     //needs to match the number of enum modes
#define LONGEST_PHASE_S 5    //longest slope we can ask for. 60s seems too long
#define ADC_FS		4095  // ADC full scale is 12 bits so 4095
#define BTM_THRD	((uint16_t)(ADC_FS / 3))
#define MID_THRD	((uint16_t)(2 * ADC_FS / 3))

// Gravity sux, down with gravity
#define GRAVITY		((float32_t)-9.8/ENV_SR) 

// some helper defines to tell which modes we are in, take care of gates/triggers and make inverted modes transparent
#define ENV_INV(_ENV_)	  ((((_ENV_)->mode == INV_ONESHOT) || ((_ENV_)->mode == INV_LOOP) || ((_ENV_)->mode == INV_TRIG)) ? (ADC_FS - (_ENV_)->val) : (_ENV_)->val)
#define ONES_MODE(_ENV_)  ((((_ENV_)->mode == ONESHOT) || ((_ENV_)->mode == INV_ONESHOT)) ? 1 : 0)
#define LOOP_MODE(_ENV_)  ((((_ENV_)->mode == LOOP) || ((_ENV_)->mode == INV_LOOP)) ? 1 : 0)
#define TRIG_MODE(_ENV_)  ((((_ENV_)->mode == TRIG) || ((_ENV_)->mode == INV_TRIG)) ? 1 : 0)
#define GATE(_ENV_, gate) ((((_ENV_)->triggered) || (gate && !TRIG_MODE(_ENV_))) ? 1 : 0)

enum state  {ATTACK, DECAY, SUSTAIN, RELEASE, IDLE, BOUNCE};
enum curves {LIN, EXP, LOG};
enum modes  {ONESHOT, INV_ONESHOT, TRIG, INV_TRIG, LOOP, INV_LOOP, BOUNCY};

typedef struct env_gen {
  uint16_t val;
  uint16_t adsr[4];
  uint16_t release_from;
  uint16_t triggered;
  uint16_t end;
  float32_t phase, phase_inc;
  enum state current_state;
  enum modes mode;
  enum curves acurve, dcurve, rcurve;
} env_gen_t;


void init_env(env_gen_t *env);
void process_env(env_gen_t *env, uint16_t gate, uint16_t trigger);
uint16_t interp_adsr_seg(enum curves curve, float32_t phase);

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

#include "envelope.h"

// tclsh for linear segment: for {set i 1} { $i <= 256 } { incr i } { puts [format "0x%03x" [expr int(($i / 256.0) * 4095)]] }
//           exp    segment: for {set i 1} { $i <= 256 } { incr i } { puts [format "0x%03x" [expr int(pow(4095, ($i / 256.0)))]] }
//           log    segment: for {set i 1} { $i <= 256 } { incr i } { puts [format "0x%03x" [expr int(log10(1 + (9 * ($i/256.0))) * 4095)]] }
static uint16_t adsr_curves[NUM_CURVES][ADSR_TABLE_LEN] = { \
  {0x00f, 0x01f, 0x02f, 0x03f, 0x04f, 0x05f, 0x06f, 0x07f, 0x08f, 0x09f, 0x0af, \
  0x0bf, 0x0cf, 0x0df, 0x0ef, 0x0ff, 0x10f, 0x11f, 0x12f, 0x13f, 0x14f, 0x15f, \
  0x16f, 0x17f, 0x18f, 0x19f, 0x1af, 0x1bf, 0x1cf, 0x1df, 0x1ef, 0x1ff, 0x20f, \
  0x21f, 0x22f, 0x23f, 0x24f, 0x25f, 0x26f, 0x27f, 0x28f, 0x29f, 0x2af, 0x2bf, \
  0x2cf, 0x2df, 0x2ef, 0x2ff, 0x30f, 0x31f, 0x32f, 0x33f, 0x34f, 0x35f, 0x36f, \
  0x37f, 0x38f, 0x39f, 0x3af, 0x3bf, 0x3cf, 0x3df, 0x3ef, 0x3ff, 0x40f, 0x41f, \
  0x42f, 0x43f, 0x44f, 0x45f, 0x46f, 0x47f, 0x48f, 0x49f, 0x4af, 0x4bf, 0x4cf, \
  0x4df, 0x4ef, 0x4ff, 0x50f, 0x51f, 0x52f, 0x53f, 0x54f, 0x55f, 0x56f, 0x57f, \
  0x58f, 0x59f, 0x5af, 0x5bf, 0x5cf, 0x5df, 0x5ef, 0x5ff, 0x60f, 0x61f, 0x62f, \
  0x63f, 0x64f, 0x65f, 0x66f, 0x67f, 0x68f, 0x69f, 0x6af, 0x6bf, 0x6cf, 0x6df, \
  0x6ef, 0x6ff, 0x70f, 0x71f, 0x72f, 0x73f, 0x74f, 0x75f, 0x76f, 0x77f, 0x78f, \
  0x79f, 0x7af, 0x7bf, 0x7cf, 0x7df, 0x7ef, 0x7ff, 0x80f, 0x81f, 0x82f, 0x83f, \
  0x84f, 0x85f, 0x86f, 0x87f, 0x88f, 0x89f, 0x8af, 0x8bf, 0x8cf, 0x8df, 0x8ef, \
  0x8ff, 0x90f, 0x91f, 0x92f, 0x93f, 0x94f, 0x95f, 0x96f, 0x97f, 0x98f, 0x99f, \
  0x9af, 0x9bf, 0x9cf, 0x9df, 0x9ef, 0x9ff, 0xa0f, 0xa1f, 0xa2f, 0xa3f, 0xa4f, \
  0xa5f, 0xa6f, 0xa7f, 0xa8f, 0xa9f, 0xaaf, 0xabf, 0xacf, 0xadf, 0xaef, 0xaff, \
  0xb0f, 0xb1f, 0xb2f, 0xb3f, 0xb4f, 0xb5f, 0xb6f, 0xb7f, 0xb8f, 0xb9f, 0xbaf, \
  0xbbf, 0xbcf, 0xbdf, 0xbef, 0xbff, 0xc0f, 0xc1f, 0xc2f, 0xc3f, 0xc4f, 0xc5f, \
  0xc6f, 0xc7f, 0xc8f, 0xc9f, 0xcaf, 0xcbf, 0xccf, 0xcdf, 0xcef, 0xcff, 0xd0f, \
  0xd1f, 0xd2f, 0xd3f, 0xd4f, 0xd5f, 0xd6f, 0xd7f, 0xd8f, 0xd9f, 0xdaf, 0xdbf, \
  0xdcf, 0xddf, 0xdef, 0xdff, 0xe0f, 0xe1f, 0xe2f, 0xe3f, 0xe4f, 0xe5f, 0xe6f, \
  0xe7f, 0xe8f, 0xe9f, 0xeaf, 0xebf, 0xecf, 0xedf, 0xeef, 0xeff, 0xf0f, 0xf1f, \
  0xf2f, 0xf3f, 0xf4f, 0xf5f, 0xf6f, 0xf7f, 0xf8f, 0xf9f, 0xfaf, 0xfbf, 0xfcf, \
  0xfdf, 0xfef, 0xfff }, \
  {0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, \
  0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x001, 0x002, \
  0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, 0x002, \
  0x003, 0x003, 0x003, 0x003, 0x003, 0x003, 0x003, 0x003, 0x003, 0x004, 0x004, \
  0x004, 0x004, 0x004, 0x004, 0x004, 0x005, 0x005, 0x005, 0x005, 0x005, 0x005, \
  0x006, 0x006, 0x006, 0x006, 0x007, 0x007, 0x007, 0x007, 0x007, 0x008, 0x008, \
  0x008, 0x009, 0x009, 0x009, 0x00a, 0x00a, 0x00a, 0x00b, 0x00b, 0x00b, 0x00c, \
  0x00c, 0x00d, 0x00d, 0x00d, 0x00e, 0x00e, 0x00f, 0x00f, 0x010, 0x010, 0x011, \
  0x012, 0x012, 0x013, 0x013, 0x014, 0x015, 0x015, 0x016, 0x017, 0x018, 0x018, \
  0x019, 0x01a, 0x01b, 0x01c, 0x01d, 0x01e, 0x01f, 0x020, 0x021, 0x022, 0x023, \
  0x024, 0x026, 0x027, 0x028, 0x029, 0x02b, 0x02c, 0x02e, 0x02f, 0x031, 0x032, \
  0x034, 0x036, 0x038, 0x03a, 0x03b, 0x03d, 0x03f, 0x042, 0x044, 0x046, 0x048, \
  0x04b, 0x04d, 0x050, 0x052, 0x055, 0x058, 0x05b, 0x05e, 0x061, 0x064, 0x068, \
  0x06b, 0x06f, 0x072, 0x076, 0x07a, 0x07e, 0x082, 0x087, 0x08b, 0x090, 0x094, \
  0x099, 0x09e, 0x0a4, 0x0a9, 0x0af, 0x0b4, 0x0ba, 0x0c1, 0x0c7, 0x0ce, 0x0d4, \
  0x0db, 0x0e3, 0x0ea, 0x0f2, 0x0fa, 0x102, 0x10b, 0x114, 0x11d, 0x126, 0x130, \
  0x13a, 0x144, 0x14f, 0x15a, 0x166, 0x171, 0x17e, 0x18a, 0x197, 0x1a5, 0x1b3, \
  0x1c1, 0x1d0, 0x1df, 0x1ef, 0x1ff, 0x210, 0x222, 0x234, 0x246, 0x25a, 0x26e, \
  0x282, 0x297, 0x2ad, 0x2c4, 0x2db, 0x2f3, 0x30c, 0x326, 0x341, 0x35c, 0x379, \
  0x396, 0x3b5, 0x3d4, 0x3f4, 0x416, 0x438, 0x45c, 0x481, 0x4a7, 0x4ce, 0x4f7, \
  0x521, 0x54c, 0x579, 0x5a7, 0x5d7, 0x609, 0x63c, 0x670, 0x6a7, 0x6df, 0x719, \
  0x755, 0x793, 0x7d3, 0x815, 0x85a, 0x8a0, 0x8e9, 0x935, 0x982, 0x9d3, 0xa26, \
  0xa7c, 0xad4, 0xb30, 0xb8f, 0xbf0, 0xc55, 0xcbd, 0xd29, 0xd98, 0xe0b, 0xe82, \
  0xefd, 0xf7c, 0xfff}, \
  {0x03d, 0x078, 0x0b2, 0x0ea, 0x11f, 0x154, 0x187, 0x1b8, 0x1e8, 0x217, 0x245, \
  0x271, 0x29d, 0x2c7, 0x2f1, 0x319, 0x341, 0x367, 0x38d, 0x3b2, 0x3d7, 0x3fa, \
  0x41d, 0x440, 0x461, 0x482, 0x4a2, 0x4c2, 0x4e2, 0x500, 0x51e, 0x53c, 0x559, \
  0x576, 0x592, 0x5ae, 0x5c9, 0x5e4, 0x5ff, 0x619, 0x633, 0x64c, 0x665, 0x67e, \
  0x696, 0x6af, 0x6c6, 0x6de, 0x6f5, 0x70c, 0x722, 0x738, 0x74e, 0x764, 0x77a, \
  0x78f, 0x7a4, 0x7b8, 0x7cd, 0x7e1, 0x7f5, 0x809, 0x81c, 0x830, 0x843, 0x856, \
  0x868, 0x87b, 0x88d, 0x89f, 0x8b1, 0x8c3, 0x8d5, 0x8e6, 0x8f8, 0x909, 0x91a, \
  0x92a, 0x93b, 0x94c, 0x95c, 0x96c, 0x97c, 0x98c, 0x99c, 0x9ab, 0x9bb, 0x9ca, \
  0x9d9, 0x9e8, 0x9f7, 0xa06, 0xa15, 0xa23, 0xa32, 0xa40, 0xa4f, 0xa5d, 0xa6b, \
  0xa79, 0xa86, 0xa94, 0xaa2, 0xaaf, 0xabc, 0xaca, 0xad7, 0xae4, 0xaf1, 0xafe, \
  0xb0b, 0xb17, 0xb24, 0xb31, 0xb3d, 0xb49, 0xb56, 0xb62, 0xb6e, 0xb7a, 0xb86, \
  0xb92, 0xb9e, 0xba9, 0xbb5, 0xbc0, 0xbcc, 0xbd7, 0xbe3, 0xbee, 0xbf9, 0xc04, \
  0xc0f, 0xc1a, 0xc25, 0xc30, 0xc3b, 0xc45, 0xc50, 0xc5b, 0xc65, 0xc70, 0xc7a, \
  0xc84, 0xc8f, 0xc99, 0xca3, 0xcad, 0xcb7, 0xcc1, 0xccb, 0xcd5, 0xcdf, 0xce9, \
  0xcf2, 0xcfc, 0xd06, 0xd0f, 0xd19, 0xd22, 0xd2c, 0xd35, 0xd3e, 0xd48, 0xd51, \
  0xd5a, 0xd63, 0xd6c, 0xd75, 0xd7e, 0xd87, 0xd90, 0xd99, 0xda2, 0xdaa, 0xdb3, \
  0xdbc, 0xdc4, 0xdcd, 0xdd6, 0xdde, 0xde7, 0xdef, 0xdf7, 0xe00, 0xe08, 0xe10, \
  0xe19, 0xe21, 0xe29, 0xe31, 0xe39, 0xe41, 0xe49, 0xe51, 0xe59, 0xe61, 0xe69, \
  0xe71, 0xe79, 0xe80, 0xe88, 0xe90, 0xe97, 0xe9f, 0xea7, 0xeae, 0xeb6, 0xebd, \
  0xec5, 0xecc, 0xed4, 0xedb, 0xee2, 0xeea, 0xef1, 0xef8, 0xeff, 0xf07, 0xf0e, \
  0xf15, 0xf1c, 0xf23, 0xf2a, 0xf31, 0xf38, 0xf3f, 0xf46, 0xf4d, 0xf54, 0xf5b, \
  0xf62, 0xf69, 0xf6f, 0xf76, 0xf7d, 0xf84, 0xf8a, 0xf91, 0xf98, 0xf9e, 0xfa5, \
  0xfab, 0xfb2, 0xfb8, 0xfbf, 0xfc5, 0xfcc, 0xfd2, 0xfd9, 0xfdf, 0xfe5, 0xfec, \
  0xff2, 0xff8, 0xfff} \
};


/*
 * process an envelope
 *
 * Walks through the phases of an ADSR in the following way:
 *
 *   Start in the idle state.  If gate is low, stay in idle.
 *   If gate is high, start the attack phase.
 *   While in the attack state, index the table until the end in the time given by attack.
 *   If gate goes low during attack phase, move directly to release.
 *   If attack finishes, move to decay state.
 *   Decay moves from max to sustain value in the given decay time
 *   If gate goes low during decay phase, move directly to release.
 *   If decay finishes, if sustain is zero move to idle, otherwise move to sustain state.
 *
 *   When an envelope is done we set the end flag to trigger other envs if needed
 *
 *   Sustain state holds that value while gate is high. On gate low move to release state
 *   In release state, move from sustain down to 0 in the time given by release.
 *   Once release is completed move to idle
 *   If gate goes high during release phase, restart the attack phase (TODO: make it restart from where it is, not 0)
 *
 *   For calculating durations - assumption is we're doing 10kHz sampling with a table length of 256 entries
 *   Time between samples will be 0.1ms so 256 entries gives 25ms or so as the fastest transition
 *   Care must be taken with an adc value of 0 - it should mean a phase_inc of a single step
 *
 *   
 *   Let's say we have full adc range = 5s (60s is tooo long)
 *   @ 10kHz that equals 50k samples 
 *
 *
 *   full scale adc = 4095
 *   table length of 256 / number of samples for 5s (50k) = 0.0512
 *   phase_inc = (4095 / <adc_value>) * 0.0512
 *
 *   doesnt return anything, the value to write to the DAC is stored in the envelope - should be 12 bit right aligned values from the tables
 *
 *   Adding loop modes which do AD loops repeat - sustain level will be used as time between repeats
 *
 *   TRIG or INV_TRIG modes - AD oneshot envelope
 *    At the end of its cycle, a triggering envelope will be used to trigger a simple AD of another envelope
 *    The triggered envelope will ignore all gate information and do a single cycle
 *    env0 triggers env1, env1 triggers env2 and env2 triggers env0 - maybe there's a way to get them in a loop, not sure
 *    The RGB led will show two of three colours to show that the envelope is in trigger mode - the selected env plus the triggering env will be lit
 *    inverted trigger mode will have the two leds, but flashing
 *   
 *   LMNC bouncing ball easter egg mode
 *    A = initial height -> full scale = 4m
 *    D = rebound -> FS = 2
 *    R = gravity -> FS = 2 * Gravity which is -9.8 ms^-2 - as we're updating at ~10kHz this should meters per 0.1ms
 *    Do the math for the height in phase, store the velocity in phase_inc - cast it into val at the end
 *    Get into bouncing ball mode by using long press and moving sustain until it's at the top of its range (blue led)
 *    Coming out of bounce goes straight to oneshot
 */
void process_env(env_gen_t *env, uint16_t gate, uint16_t trigger)
{
  // default to staying exactly where we are
  enum state next_state = env->current_state;

  // check to see if we're triggered
  if (TRIG_MODE(env) && trigger) {
    env->triggered = 1;
  }

  // clear end of env flag
  if (env->end) {
    env->end = 0;
  }

  switch (env->current_state) {

    case IDLE:
      if (!GATE(env, gate)) { 
	env->val = 0;
      } else {
	//start the ball rolling
	if (env->mode == BOUNCY) {
	  next_state = BOUNCE;
	  //attack is initial height
	  env->phase = env->adsr[ATTACK];
	  env->val = env->adsr[ATTACK]; 
	} else {
	  next_state = ATTACK;
	  env->phase = 0;	
	}
      }
      break;

    case ATTACK:
      //gate == 0 && !env->triggered
      if (!GATE(env, gate)) { 
	env->phase = (uint16_t)ADSR_TABLE_LEN - 1;
	env->release_from = env->val;
	next_state = RELEASE;
      } else {
	if (env->adsr[ATTACK] > 0) {
	  env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * (ADC_FS / env->adsr[ATTACK]);
	} else {
	  //attack is zero, phase_inc is twice what it would be if it were one
	  env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * 2 * ADC_FS;
	}
	env->phase += env->phase_inc;
	if (env->phase > ADSR_TABLE_LEN) {
	  env->phase = (uint16_t)ADSR_TABLE_LEN - 1;
	  env->val = ADC_FS;
	  next_state = DECAY;
	} else {
	  env->val = interp_adsr_seg(env->acurve, env->phase);
	}
      }
      break;

    case DECAY:
      if (!GATE(env, gate)) { 
	env->phase = (uint16_t)ADSR_TABLE_LEN - 1;
	env->release_from = env->val;
	next_state = RELEASE;
      } else {
	if (env->adsr[DECAY] > 0) {
	  env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * (ADC_FS / env->adsr[DECAY]);
	} else {
	  //decay is zero, phase_inc is twice what it would be if it were one
	  env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * 2 * ADC_FS;
	}
	env->phase -= env->phase_inc;
	if (env->phase < 0) {
	  next_state = SUSTAIN;
	  if (LOOP_MODE(env)) {
	    env->phase = 0;
	    env->val = 0;
	    // maybe having loop modes signal the end of their AD cycle can be interesting - we will have looping triggers?
	    env->end = 1;
	  } else if (TRIG_MODE(env)) {
	    env->phase = 0;
	    env->val = 0;
	    env->triggered = 0;
	    env->end = 1;
	    next_state = IDLE;
	  } else {
	    env->val = env->adsr[SUSTAIN];
	  }
	} else {
	  if (ONES_MODE(env)) {
	    env->val = (uint16_t)(env->adsr[SUSTAIN] + ((ADC_FS - env->adsr[SUSTAIN]) * (interp_adsr_seg(env->dcurve, env->phase) / (float32_t)ADC_FS)));
	  } else {
	    env->val = interp_adsr_seg(env->dcurve, env->phase);
	  }
	}
      }
      break;

    case SUSTAIN:
      if (!GATE(env, gate)) { 
	env->phase = (uint16_t)ADSR_TABLE_LEN - 1;
	env->release_from = env->adsr[SUSTAIN];
	if (LOOP_MODE(env)) {
	  //have it trigger each time it finishes its AD cycle, not when the gate goes off
	  //env->end = 1;
	  next_state = IDLE;
	} else {
	  next_state = RELEASE;
	}
      } else if (LOOP_MODE(env)) {
	if (env->adsr[SUSTAIN] > 0) {
	  env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * (ADC_FS / env->adsr[SUSTAIN]);
	} else {
	  //sustain is zero, phase_inc is twice what it would be if it were one
	  env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * 2 * ADC_FS;
	}
	env->phase += env->phase_inc;
	if (env->phase > ADSR_TABLE_LEN) {
	  next_state = ATTACK;
	  env->phase = 0;	
	}
      }
      break;

    case RELEASE:
      if (GATE(env, gate)) { 
	next_state = ATTACK;
      } else {
	if (env->adsr[SUSTAIN] == 0) {
	  // skip right to idle if sustain is 0
	  env->end = 1;
	  next_state = IDLE;
	  env->val = 0;
	} else {
	  if (env->adsr[RELEASE] > 0) {
	    env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * (ADC_FS / env->adsr[RELEASE]);
	  } else {
	    //release is zero, phase_inc is twice what it would be if it were one
	    env->phase_inc = ((float32_t)ADSR_TABLE_LEN / (LONGEST_PHASE_S * ENV_SR)) * 2 * ADC_FS;
	  }
	  env->phase -= env->phase_inc;
	  if (env->phase <= 0) {
	    env->phase = 0;
	    env->val = 0;
	    env->end = 1;
	    next_state = IDLE;
	  } else {
	    env->val = (uint16_t)(env->release_from * (interp_adsr_seg(env->rcurve, env->phase) / (float32_t)ADC_FS));
	  }
	}
      }
      break;

    case BOUNCE:
      if (!GATE(env, gate)) { 
	// shut it down as soon as the gate goes off
	next_state = IDLE;
	env->val = 0;
      } else {
	env->phase_inc += (2 * ((float32_t)env->adsr[RELEASE] / ADC_FS) * GRAVITY); // phase_inc is the velocity, increment it by gravity
	env->phase += env->phase_inc;						    // calculate our new height - add velocity to our current position
	// if height is less than zero it means we've hit the ground - bounce by reversing our velocity and multiplying it by our rebound factor
	if (env->phase <= 0) {
	  env->phase = 0;
	  env->phase_inc *= (-2 * ((float32_t)env->adsr[DECAY] / ADC_FS));
	}
	//finally - it's all done - put it into the value that goes to the dac.
	env->val = (uint16_t)env->phase;
      }
      break;

    default:
	// We should never get here.
	next_state = IDLE;
	env->val = 0;
	break;
  }

  //shouldn't have to do this - but maybe things go heywire - a distinct possibility in bouncing ball mode
  if (env->val > ADC_FS) { env->val = ADC_FS; }
  env->current_state = next_state;

}

/*
 * Inits an envelope
 * sets most things to zero
 */
void init_env(env_gen_t *env) {
  env->val = 0;
  env->mode = ONESHOT;
  env->adsr[ATTACK] = 0;
  env->adsr[DECAY] = 0;
  env->adsr[SUSTAIN] = 0;
  env->adsr[RELEASE] = 0;
  env->release_from = 0;
  env->triggered = 0;
  env->end = 0;
  env->phase = 0;
  env->phase_inc = 0;
  env->current_state = IDLE;
  env->acurve = LIN;
  env->dcurve = LIN;
  env->rcurve = LIN;
}

uint16_t interp_adsr_seg(enum curves curve, float32_t phase) {

  uint16_t idx = phase;

  if (idx >= ADSR_TABLE_LEN) {
    idx = ADSR_TABLE_LEN - 1;
  }

  uint16_t next_idx = idx + 1;
  float32_t r = phase - idx;
  uint16_t s0, s1, sf;

  s0 = adsr_curves[curve][idx];

  if (next_idx >= ADSR_TABLE_LEN) {
    s1 = adsr_curves[curve][ADSR_TABLE_LEN - 1];
  } else {
    s1 = adsr_curves[curve][next_idx];
  }
  sf = (uint16_t)(s0 + r * (s1 - s0));

  return sf;
}

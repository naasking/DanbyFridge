#ifndef ROTARY_H
#define ROTARY_H

/**
 * Copyright 2021 Sandro Magi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

/**
 * @file rotary.h
 * Quadrature Rotary Encoder Debouncing
 * 
 * Process quadrature rotary events in an interrupt handler so you don't miss
 * steps:
 * 
 * static volatile unsigned rot_state;
 * static volatile unsigned rot_pos;
 * 
 * void setup() {
 *    attachInterrupt(digitalPinToInterrupt(PIN_X), &rot_onchange, CHANGE);
 * }
 * 
 * static void rot_onchange() {
 *    rot_pos += rotary_step(&rot_state, digitalRead(PIN_A), digitalRead(PIN_B));
 * }
 */

/**
 * A bitmask encoding valid state transitions.
 */
enum ROTARY_TRANSITIONS {
  ROTARY_CW  = 10260, //=B0010100000010100,
  ROTARY_CCW = 16770, //=B0100000110000010,
};

/**
 * @brief Check whether last rotary encoder move was clockwise.
 * 
 * @param r Rotary encoder state
 * @return 1 if last move was clockwise, 0 otherwise
 */
#define rotary_cw(r) (((uint16_t)1 << (r)) & ROTARY_CW)

/**
 * @brief Check whether last rotary encoder move was counterclockwise.
 * 
 * @param r Rotary encoder state
 * @return 1 if last move was counterclockwise, 0 otherwise
 */
#define rotary_ccw(r) (((uint16_t)1 << (r)) & ROTARY_CCW)

/**
 * @brief Process a rotary encoder step.
 * 
 * @param rotary  Rotary encoder state
 * @param rotb    Rotary encoder B pin state (MSB)
 * @param rota    Rotary encoder A pin state (LSB)
 * @return 1 = one step clockwise, -1 = one step counter clockwise, 0 = invalid step
 * @remarks rotary state rolling history of the last 4 states, which are used to debounce invalid state transitions
 */
static inline int8_t rotary_step(volatile uint8_t *rotary, uint8_t rotb, uint8_t rota) {
  *rotary = (uint8_t)((((uint16_t)(*rotary) << 2) | (rotb << 1) | rota) & 0xff);
  return rotary_cw(*rotary) ? 1:
         rotary_ccw(*rotary)?-1:
                              0;
}

#endif
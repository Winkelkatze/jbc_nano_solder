/*
 * MIT License
 *
 * Copyright (c) 2020 Daniel Frejek
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

#pragma once

#include <stdbool.h>
#include <stdint.h>

void input_init(void);

// call once per systick
void input_update(void);

// encoder ticks != 0
bool input_encoder_has_ticks(void);

// get current encoder ticks
int input_encoder_ticks(void);

// get current encoder ticks and reset the counter
int input_encoder_ticks_reset(void);

// encoder switch pressed?
bool input_encoder_switch(void);
// idle detect switch
bool input_idle_switch(void);


// delay 'ticks' or resume immediately if an input event occurred
void input_delay_or_evt(uint32_t ticks);

// add the current encode value to the given number and clamp within range
// resets the encoder ticks
uint32_t input_add_encoder_value_bound(uint32_t val, uint32_t lower_bound, uint32_t upper_bound);
uint32_t input_add_encoder_value_bound_step(uint32_t val, uint32_t lower_bound, uint32_t upper_bound, uint32_t step);

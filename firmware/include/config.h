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

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	uint16_t limit_max;
	uint16_t limit_min;
	uint16_t standby_temp;
	uint16_t standby_delay; // 10s of seconds
	uint16_t standby_time;  // minutes
	uint16_t idle_detect_threshold; // % of max power
	uint16_t controller_max_pwm;
	uint16_t controller_period;
	uint16_t controller_window;
	uint16_t controller_int_limit;  // K offset
	uint16_t controller_int_speed;  // mK / period
	uint16_t tc_v_per_deg;
	uint16_t tc_offset;
} config_t;


extern config_t config;

bool config_load(void);
void config_load_default(void);
void config_save(void);

uint16_t config_load_last_temperature(void);
void config_set_last_temperature(uint16_t temp);

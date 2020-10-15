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

#include "input.h"
#include "gpio_conf.h"

#include <libopencm3/cm3/nvic.h>

#define ENCODER_DEBOUNCE_TICKS 2
#define BUTTON_DEBOUNCE_TICKS 2
#define IDLE_SW_DEBOUNCE_TICKS 200


volatile struct
{
	int ticks;
	bool phase;
	uint32_t debounce;
} encoder;

volatile struct
{
	bool stable_state;
	bool raw_state;
	uint32_t debounce;
} encoder_sw, idle_sw;

extern uint32_t tickcounter;

// Interrupt for GPIO4-15
void exti4_15_isr(void)
{
	exti_reset_request(EXTI_ENCODER_A | EXTI_ENCODER_SW | EXTI_IDLE_SW);

	// update encoder
	if ((gpio_get(GPIOA, GPIO_ENCODER_A) != 0) != encoder.phase)
	{
		encoder.phase = !encoder.phase;
		if (tickcounter - encoder.debounce > ENCODER_DEBOUNCE_TICKS)
		{
			if (encoder.phase != (gpio_get(GPIOA, GPIO_ENCODER_B) != 0))
			{
				encoder.ticks++;
			}
			else
			{
				encoder.ticks--;
			}
			encoder.debounce = tickcounter;
		}
	}

	bool esw = !gpio_get(GPIOA, GPIO_ENCODER_SW);
	bool isw = !gpio_get(GPIOA, GPIO_IDLE_SW);
	if (encoder_sw.raw_state != esw)
	{
		encoder_sw.debounce = tickcounter;
		encoder_sw.raw_state = esw;
	}

	if (idle_sw.raw_state != isw)
	{
		idle_sw.debounce = tickcounter;
		idle_sw.raw_state = isw;
	}
}

void input_init(void)
{
	encoder.phase = (gpio_get(GPIOA, GPIO_ENCODER_A) != 0);
	encoder_sw.raw_state = !gpio_get(GPIOA, GPIO_ENCODER_SW);
	idle_sw.raw_state = !gpio_get(GPIOA, GPIO_IDLE_SW);
}

void input_update(void)
{
	if (tickcounter - encoder_sw.debounce > BUTTON_DEBOUNCE_TICKS)
	{
		encoder_sw.stable_state = encoder_sw.raw_state;
	}

	if (tickcounter - idle_sw.debounce > IDLE_SW_DEBOUNCE_TICKS)
	{
		idle_sw.stable_state = idle_sw.raw_state;
	}
}


bool input_encoder_has_ticks(void)
{
	return encoder.ticks != 0;
}


int input_encoder_ticks(void)
{
	return encoder.ticks;
}


int input_encoder_ticks_reset(void)
{
	int ret = encoder.ticks;
	encoder.ticks = 0;
	return ret;
}


bool input_encoder_switch(void)
{
	return encoder_sw.stable_state;
}


bool input_idle_switch(void)
{
	return idle_sw.stable_state;
}


void input_delay_or_evt(uint32_t ticks)
{
	bool old_esw = input_encoder_switch();
	uint32_t start = tickcounter;
	while (tickcounter - start > ticks)
	{
		if (input_encoder_has_ticks() ||
			old_esw != input_encoder_switch())
		{
			return;
		}
	}
}

uint32_t input_add_encoder_value_bound(uint32_t val, uint32_t lower_bound, uint32_t upper_bound)
{
	return input_add_encoder_value_bound_step(val, lower_bound, upper_bound, 1);
}

uint32_t input_add_encoder_value_bound_step(uint32_t val, uint32_t lower_bound, uint32_t upper_bound, uint32_t step)
{
	int t = input_encoder_ticks_reset() * step;

	if (t > 0 && val + t > upper_bound)
	{
		return upper_bound;
	}
	else if (t < 0 && val < -t + lower_bound)
	{
		return lower_bound;
	}
	return val + t;
}

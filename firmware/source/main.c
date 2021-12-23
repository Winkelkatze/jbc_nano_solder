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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#include "tinyprintf.h"
#include "ssd1306.h"
#include "eeprom_i2c.h"
#include "config.h"
#include "input.h"
#include "menu.h"
#include "gpio_conf.h"


#define HEATER_TIMER TIM3
#define HEATER_TIMER_PRESCALER 48 // 1us per tick, so max period is 65ms
#define HEATER_TIMER_OC TIM_OC4
#define HEATER_TIMER_OC_IRQ TIM_DIER_CC4IE

#define ADC_TRIGGER (3 << ADC_CFGR1_EXTSEL_SHIFT) // Timer3


#define TEMP_SAVE_INTERVAL 1000

// panic cutoff disables the heater until the mcu is reset
// this is at a relative temp of 440K assuming 9 uV/K and a VCC of 3.3V
#define PANIC_CUTOFF_ADC_VALUE ((440ULL * 9 * 501 * 4096) / 3300000) // ~430 C @ 3.3V

// Panic, if the temperature difference is below this for a given amount of time while the heater is on
// This should prevent overheating, if the tip is connected in the wrong polarity
#define PANIC_MINIMUM_TEMP_DIFF 20
#define PANIC_MINIMUM_TEMP_DIFF_TIMEOUT 1000

// If the measured supply voltage is not within this range, we set the error state
#define PANIC_MIN_SUPPLY_VOLTAGE  6000
#define PANIC_MAX_SUPPLY_VOLTAGE 15000

// Somehow we get some bad measurements (I think its interference from the display, but not sure...)
// In order to filter those, we add an "outlier detection"
// This works by calculating the average deviation from the average over the last measurements.
// If a measurement is more than THRESHOLD percent wrong, we skip controller cycle and use the average last value instead.
#define OUTLIER_DETECTION_WND 16
// Threshold should be at least 200, since this is what we expect for a monotonous increase
#define OUTLIER_DETECTION_THRESHOLD 230
// Minimum avg error (mK), this prevents generating lots of outliers when the temperature is stable
#define OUTLIER_DETECTION_MIN_AVG_ERR 2000

static void update_stby_state(void);
static bool outlier_detection(uint32_t value);

/*
 * Basic mode of operation
 * HEATING    |    DEAD_TIME    |    ADC_SAMPLE    | ...
 *            ^                 ^                  ^
 *        Timer OC   Timer update (adc trg)   ADC cplt int
 */

volatile uint32_t tickcounter = 0;


#define ADC_VALUE_VREF 0
#define ADC_VALUE_VDC  1
#define ADC_VALUE_TC   2
static volatile uint16_t adc_values[3];

// This is the panic shutdown
// Value is set to true, if the temperature exceeds the panic threshold
// It's never cleared, so device needs to be restart.
static volatile bool kill_heater = false;
static const char *err_reason = NULL;

// derived from period and max_pwm
static uint16_t heater_max_period;

static uint16_t temp_set;
static uint32_t temp_current1000;
static uint16_t controller_offset1000 = 0;
static uint32_t current_duty_smoothed = 0;
static uint32_t current_duty_lt_avg100 = 0;
static uint32_t temp_diff_panic_timer = 0;
static uint32_t outlier_detection_window[OUTLIER_DETECTION_WND] = {};
static size_t outlier_detection_index = 0;
static size_t outlier_detection_sum = 0;


static struct
{
	uint32_t idle_detect_timer;
	uint32_t timer;

	bool manual_off;
	bool off;
	bool stby;
	bool idle_sw;
} stby_state;



void sys_tick_handler(void)
{
	tickcounter++;
	input_update();
	update_stby_state();
}


void dma1_channel1_isr(void)
{
	// DMA interrupt from ADC DMA
	// only used for DMA1_TCIF
	dma_clear_interrupt_flags(DMA1, 1, DMA_TCIF);

	if (adc_values[ADC_VALUE_TC] > PANIC_CUTOFF_ADC_VALUE)
	{
		kill_heater = true;
		err_reason = "T HIGH";
	}

	uint16_t ts = temp_set;

	if (stby_state.off)
	{
		ts = 0;
	}
	else if (stby_state.stby && config.standby_temp < ts)
	{
		ts = config.standby_temp;
	}

	// thermocouple voltahe in 0.1uV
	uint32_t vtc_uv10 = (((33000000ULL / 501) * ST_VREFINT_CAL) * adc_values[ADC_VALUE_TC]) / adc_values[ADC_VALUE_VREF] / 4096;

	// temperature in mK
	// Of course, our resolution isn't that high, but it makes our controller more stable if we use more precise values.
	uint32_t tc1000 = (1000 * vtc_uv10) / config.tc_v_per_deg;

	if (!kill_heater)
	{
		if (ts == 0 || tc1000 > PANIC_MINIMUM_TEMP_DIFF * 1000)
		{
			temp_diff_panic_timer = tickcounter;
		}
		else if ((tickcounter - temp_diff_panic_timer) > PANIC_MINIMUM_TEMP_DIFF_TIMEOUT)
		{
			kill_heater = true;
			err_reason = "T LOW";
		}

		uint32_t vdc = ((3300ULL * 11 * ST_VREFINT_CAL) * adc_values[ADC_VALUE_VDC]) / adc_values[ADC_VALUE_VREF] / 4096;
		if (vdc < PANIC_MIN_SUPPLY_VOLTAGE)
		{
			kill_heater = true;
			err_reason = "V LOW";
		}
		if (vdc > PANIC_MAX_SUPPLY_VOLTAGE)
		{
			kill_heater = true;
			err_reason = "V HIGH";
		}
	}

	tc1000 += config.tc_offset * 1000;

	uint16_t new_period = 0;

	// outlier detection
	if (outlier_detection(tc1000))
	{
		temp_current1000 = (temp_current1000 * 7 + tc1000) / 8;
		int err1000 = (int)(ts * 1000) - (int)tc1000;

		int int_adjust1000;
		if (err1000 > 0 && err1000 > config.controller_int_speed)
		{
			int_adjust1000 = config.controller_int_speed;
		}
		else if (err1000 < 0 && -err1000 > config.controller_int_speed)
		{
			int_adjust1000 = -(int)config.controller_int_speed;
		}
		else
		{
			int_adjust1000 = err1000;
		}

		if (int_adjust1000 > 0 && controller_offset1000 + int_adjust1000 > config.controller_int_limit * 1000)
		{
			controller_offset1000 = config.controller_int_limit * 1000;
		}
		else if (int_adjust1000 < 0 && controller_offset1000 < -int_adjust1000)
		{
			controller_offset1000 = 0;
		}
		else
		{
			controller_offset1000 += int_adjust1000;
		}

		err1000 += controller_offset1000;
		//printf("tc=%u, ts=%u, e=%i co=%u, ia=%i\n", tc1000/1000, temp_set, err1000, controller_offset1000, int_adjust1000);


		if (err1000 > config.controller_window * 1000)
		{
			// max power
			new_period = heater_max_period;
		}
		else if (err1000 > 0)
		{
			// (err / wnd) * max_period
			new_period = (err1000 * heater_max_period) / (config.controller_window * 1000);
		}
	}
	else if (!stby_state.off && ts > config.tc_offset)
	{
		// detected an outlier, so we just re-use the average
		new_period = current_duty_smoothed;
	}

	timer_set_oc_value(HEATER_TIMER, HEATER_TIMER_OC, new_period);

	current_duty_smoothed = ((current_duty_smoothed * 63) + new_period) >> 6;
	current_duty_lt_avg100 = ((current_duty_lt_avg100 * 1023) + (new_period * 100)) >> 10;

	//printf("s=%u c=%u err=%i period=%u period_max=%u\r\n", temp_set, temp_current, err, new_period, heater_max_period);

	// Re-arm adc trigger by setting the start bit
	adc_start_conversion_regular(ADC1);

	// Start the timer again
	timer_enable_counter(HEATER_TIMER);

	if (!kill_heater && new_period)
	{
		gpio_clear(GPIOB, GPIO_HEATER);
	}
}


void tim3_isr(void)
{
	// Only used for the OC interrupt
	// We should turn the heater OFF now
	timer_clear_flag(HEATER_TIMER, HEATER_TIMER_OC_IRQ);

	iwdg_reset();
	gpio_set(GPIOB, GPIO_HEATER);
}


static void tpf_putcf(void *ptr, char c)
{
	(void)ptr;
	usart_send_blocking(USART1, c);
}

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_48mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_DMA);

	systick_set_frequency(1000, 48000000);
	systick_counter_enable();
	systick_interrupt_enable();
}


static void gpio_setup(void)
{
	// Timer output (On GPIOB !)
	// We do this first, since the heater is ON while the GPIO is floating!
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_HEATER);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO_HEATER);
	gpio_set(GPIOB, GPIO_HEATER);


	// USART
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

	// I2C
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9 | GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);

	// ADC (V_TC and VDD)
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);

	// Encoder and idle switch
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_ENCODER_A | GPIO_ENCODER_B | GPIO_ENCODER_SW | GPIO_IDLE_SW);
}


static void usart_setup(void)
{
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable(USART1);
}


static void i2c_setup(void)
{
	rcc_set_i2c_clock_sysclk(I2C1);
	i2c_set_speed(I2C1, i2c_speed_fm_400k, 48);
	i2c_peripheral_enable(I2C1);
}


static void adc_setup(void)
{
	// The ADC will sample the TC voltage, VCC and the internal reference
	// TC conversion is last to maximize the time between turning off the heater and measuring the temp.
	// We configure the ADC in single-sequence mode tied to the haeter timer as trigger and the DMA in circular mode.
	// Then, a DMA_CPLT interrupt is generated after the full conversion is complete
	// To re-arm the ADC, we only need to set the start bit again.
	adc_calibrate(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_enable_vrefint();
	uint8_t ADC_SEQUENCE[] = {ADC_CHANNEL_VREF, 1, 0};
	adc_set_regular_sequence(ADC1, sizeof(ADC_SEQUENCE) / sizeof(ADC_SEQUENCE[0]), ADC_SEQUENCE);
	adc_power_on(ADC1);
	adc_enable_dma(ADC1);
	adc_enable_external_trigger_regular(ADC1, ADC_TRIGGER, ADC_CFGR1_EXTEN_RISING_EDGE);
	//adc_enable_dma_circular_mode(ADC1);

	// setup DMA
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_values);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, sizeof(ADC_SEQUENCE) / sizeof(ADC_SEQUENCE[0]));
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);
	dma_set_channel_request(DMA1, DMA_CHANNEL1, 0);

	dma_enable_channel(DMA1, DMA_CHANNEL1);

	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	// arm the adc trigger
	adc_start_conversion_regular(ADC1);
}


static void exti_setup(void)
{
	exti_select_source(EXTI_ENCODER_A | EXTI_ENCODER_SW | EXTI_IDLE_SW, GPIOA);
	exti_set_trigger(EXTI_ENCODER_A | EXTI_ENCODER_SW | EXTI_IDLE_SW, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI_ENCODER_A | EXTI_ENCODER_SW | EXTI_IDLE_SW);

	nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}


static void timer_setup(void)
{
	timer_set_mode(HEATER_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(HEATER_TIMER, HEATER_TIMER_PRESCALER);
	timer_one_shot_mode(HEATER_TIMER);

	// for now we set the period to fixed 10ms
	timer_set_period(HEATER_TIMER, 10000);

	timer_set_master_mode(HEATER_TIMER, TIM_CR2_MMS_UPDATE);

	// instead of the OC output of the timer, we do it by foot through an interrupt
	// It's impossible to configure the OC output in a way, that the output stays in the same state as in the second half of the cycle
	timer_set_oc_value(HEATER_TIMER, HEATER_TIMER_OC, 0);
	timer_enable_irq(HEATER_TIMER, HEATER_TIMER_OC_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
}


static void reset_stby_state(void)
{
	stby_state.timer = tickcounter;
	stby_state.idle_detect_timer = tickcounter;
	stby_state.idle_sw = input_idle_switch();
	stby_state.off = stby_state.manual_off;
	stby_state.stby = false;
}


static void update_stby_state(void)
{
	if (stby_state.manual_off)
	{
		stby_state.off = true;
		return;
	}

	if (!stby_state.off && config.idle_detect_threshold != 0)
	{
		// idle if long-term avg < current * (1 - threshold)
		if (current_duty_smoothed * (100 - config.idle_detect_threshold) > current_duty_lt_avg100)
		{
			// heater is doing things -> reset timer
			stby_state.idle_detect_timer = tickcounter;
		}
		else if (tickcounter - stby_state.idle_detect_timer > 60000UL * config.standby_time)
		{
			// idle detection timeout
			stby_state.off = true;
		}
	}

	if (input_idle_switch())
	{
		if (!stby_state.idle_sw)
		{
			stby_state.idle_sw = true;
			if (!stby_state.off)
			{
				// reset idle detect timer
				reset_stby_state();
			}
		}
		if (stby_state.off)
		{
			return;
		}

		if (stby_state.stby && tickcounter - stby_state.timer > 60000UL * config.standby_time)
		{
			// standby timeout -> turn off
			stby_state.off = true;
		}

		if (!stby_state.stby && tickcounter - stby_state.timer > 10000UL * config.standby_delay)
		{
			// not yet in standby and long enough in holder
			stby_state.stby = true;
			stby_state.timer = tickcounter;
		}
	}
	else
	{
		// NOT in holder
		if (stby_state.idle_sw)
		{
			// reset idle detect timer
			reset_stby_state();
		}
	}
}


static void update_config(void)
{
	timer_set_period(HEATER_TIMER, config.controller_period * 1000);
	heater_max_period = (((uint32_t)config.controller_max_pwm) * config.controller_period * 1000) / 100;

	if (temp_set < config.limit_min)
	{
		temp_set = config.limit_min;
	}
	if (temp_set > config.limit_max)
	{
		temp_set = config.limit_max;
	}

	config_save();
}


static bool outlier_detection(uint32_t value)
{
	// insert new value
	outlier_detection_sum -= outlier_detection_window[outlier_detection_index];
	outlier_detection_window[outlier_detection_index] = value;
	outlier_detection_index = (outlier_detection_index + 1) % OUTLIER_DETECTION_WND;
	outlier_detection_sum += value;

	uint32_t avg = outlier_detection_sum / OUTLIER_DETECTION_WND;

	uint32_t err_sum = 0;
	for (size_t i = 0; i < OUTLIER_DETECTION_WND; ++i)
	{
		if (outlier_detection_window[i] > avg)
		{
			err_sum += outlier_detection_window[i] - avg;
		}
		else
		{
			err_sum += avg - outlier_detection_window[i];
		}
	}

	uint32_t avg_err = err_sum / OUTLIER_DETECTION_WND;
	if (avg_err < OUTLIER_DETECTION_MIN_AVG_ERR)
	{
		avg_err = OUTLIER_DETECTION_MIN_AVG_ERR;
	}

	uint32_t val_err;
	if (value > avg)
	{
		val_err = value - avg;
	}
	else
	{
		val_err = avg - value;
	}

	return val_err * 100 < avg_err * OUTLIER_DETECTION_THRESHOLD;
}


static void screen_home(void)
{
	uint32_t sw_dn = 0;
	char disp_buf[32];
	uint32_t temp_changed_time;
	bool temp_changed = false;
	while (1)
	{
		uint16_t temp_round = (((temp_current1000 / 1000) + 2) / 5) * 5;
		//uint16_t temp_round = temp_current1000 / 1000;

		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 3);
		disp_buf[0] = ' ' + ((temp_round / 100) % 10);
		disp_buf[1] = ' ' + ((temp_round / 10 ) % 10);
		disp_buf[2] = ' ' + ((temp_round      ) % 10);
		disp_buf[3] = 0;

		ssd1306_WriteStringComp(disp_buf, CompFont_16x26_NUMC, White);
		ssd1306_DrawCircle(54, 10, 3, White);
		ssd1306_SetCursor(58,3);
		ssd1306_WriteStringComp("\x2a", CompFont_16x26_NUMC, White);

		ssd1306_SetCursor(90,3);
		snprintf(disp_buf, sizeof(disp_buf), "%3u", temp_set);
		ssd1306_WriteStringComp(disp_buf, CompFont_7x10, White);
		ssd1306_DrawCircle(117, 5, 1, White);
		ssd1306_SetCursor(121, 3);
		ssd1306_WriteStringComp("C", CompFont_7x10, White);

		if (kill_heater)
		{
			ssd1306_SetCursor(128 - 6 * 7,12);
			ssd1306_WriteStringComp("ERROR ", CompFont_7x10, Black);

			if (err_reason)
			{
				ssd1306_SetCursor(128 - 6 * 7,22);
				ssd1306_WriteStringComp(err_reason, CompFont_7x10, Black);
			}
		}
		else if (stby_state.off)
		{
			ssd1306_SetCursor(128 - 3 * 7,22);
			ssd1306_WriteStringComp("OFF", CompFont_7x10, White);
		}
		else if (stby_state.stby)
		{
			ssd1306_SetCursor(128 - 4 * 7,22);
			ssd1306_WriteStringComp("STBY", CompFont_7x10, White);
		}
		else
		{
			ssd1306_SetCursor(128 - 2 * 7,22);
			ssd1306_WriteStringComp("ON", CompFont_7x10, White);
		}

		uint16_t pwrline = current_duty_smoothed * 128 / heater_max_period;

		if (pwrline)
		{
			ssd1306_Line(0, 31, pwrline, 31, White);
		}

		ssd1306_UpdateScreen();


		printf("c=%lu, lt=%lu, idle=%lu\r\n", current_duty_smoothed, current_duty_lt_avg100, tickcounter - stby_state.idle_detect_timer);
		//printf("t_min=%i, tmax=%i, v33min=%i, v33max=%i\r\n", adc_tc_min, adc_tc_max, adc_v33_min, adc_v33_max);


		// update every 100 ms
		input_delay_or_evt(100);
		if (input_encoder_switch())
		{
			if (sw_dn == 0)
			{
				sw_dn = tickcounter;
			}
			else if (tickcounter - sw_dn > 1000)
			{
				// Long press -> Menu
				return;
			}
		}
		else if (sw_dn != 0)
		{
			sw_dn = 0;
			// Short press -> On/Off

			// We set the manual off state here to the inverse of the off state.
			// Otherwise (using the manual_state) pressing the switch while OFF from idle will turn the iron OFF first.
			stby_state.manual_off = !stby_state.off;
			reset_stby_state();
		}

		uint16_t old_temp = temp_set;
		temp_set = input_add_encoder_value_bound_step((temp_set * 5) / 5, config.limit_min, config.limit_max, 5);

		if (temp_set != old_temp)
		{
			temp_changed = true;
			temp_changed_time = tickcounter;
		}

		if (temp_changed && tickcounter - temp_changed_time > TEMP_SAVE_INTERVAL)
		{
			printf("Saving new temp: %u/r/n", temp_set);
			config_set_last_temperature(temp_set);
			temp_changed = false;
		}
	}
}


static void screen_raw(void)
{
	while (!input_encoder_switch())
	{
		uint32_t v33 = (3300 * ST_VREFINT_CAL) / adc_values[ADC_VALUE_VREF];
		uint32_t vdc = ((3300ULL * 11 * ST_VREFINT_CAL) * adc_values[ADC_VALUE_VDC]) / adc_values[ADC_VALUE_VREF] / 4096;
		uint32_t vtc = (((3300000ULL / 501) * ST_VREFINT_CAL) * adc_values[ADC_VALUE_TC]) / adc_values[ADC_VALUE_VREF] / 4096;
		printf("v33=%lumV vdc=%lumV tc=%luuV ~temp=%luC\r\n", v33, vdc, vtc, vtc/8 + 25);

		char buff[32];
		ssd1306_Fill(Black);

		ssd1306_SetCursor(0,0);
		snprintf(buff, sizeof(buff), "V33 %4lu mV", v33);
		ssd1306_WriteStringComp(buff, CompFont_7x10, White);
		ssd1306_SetCursor(0,11);
		snprintf(buff, sizeof(buff), "VDC %4lu mV", vdc);
		ssd1306_WriteStringComp(buff, CompFont_7x10, White);
		ssd1306_SetCursor(0,22);
		snprintf(buff, sizeof(buff), "TC  %4lu uV", vtc);
		ssd1306_WriteStringComp(buff, CompFont_7x10, White);


		uint16_t pwr = current_duty_smoothed * 100 / heater_max_period;
		ssd1306_SetCursor(86,0);
		snprintf(buff, sizeof(buff), "P %u%%", pwr);
		ssd1306_WriteStringComp(buff, CompFont_7x10, White);

		ssd1306_UpdateScreen();
	}
}




int main(void)
{
	iwdg_set_period_ms(150);
	iwdg_start();

	clock_setup();
	gpio_setup();
	i2c_setup();
	exti_setup();
	usart_setup();
	adc_setup();
	timer_setup();

	init_printf(NULL, &tpf_putcf);

	printf("==============================================================\r\n");

	iwdg_reset();
	ssd1306_Init();

	input_init();

	// Don't turn on automatically as soon as there is power.
	stby_state.manual_off = true;
	reset_stby_state();

	iwdg_reset();

	temp_diff_panic_timer = tickcounter;

	// initially start the ADC sample sequence by starting the timer
	timer_enable_counter(HEATER_TIMER);

	// load config
	if (!config_load())
	{
		menu_restore_default_config();
	}

	temp_set = config_load_last_temperature();

	update_config();


	menu_result_t sel = menu_result_home;

	while (1)
	{
		printf("Show main screen %i\r\n", sel);
		switch (sel)
		{
			case menu_result_show_raw:
				screen_raw();
				break;

			case menu_result_home:
			default:
				screen_home();
				break;
		}

		sel = menu_main();
		update_config();
	}
}

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
#include "ssd1306_fonts.h"
#include "eeprom_i2c.h"

// GPIOA
#define GPIO_ENCODER_A  GPIO4
#define GPIO_ENCODER_B  GPIO5
#define GPIO_ENCODER_SW GPIO6
#define GPIO_IDLE_SW    GPIO7

// GBIOB
#define GPIO_HEATER     GPIO1

#define EXTI_ENCODER_A  EXTI4
#define EXTI_ENCODER_SW EXTI6
#define EXTI_IDLE_SW    EXTI7

#define HEATER_TIMER TIM3
#define HEATER_TIMER_PRESCALER 48 // 1us per tick, so max period is 65ms
#define HEATER_TIMER_OC TIM_OC4
#define HEATER_TIMER_OC_IRQ TIM_DIER_CC4IE

#define ADC_TRIGGER (3 << ADC_CFGR1_EXTSEL_SHIFT) // Timer3

#define ENCODER_DEBOUNCE_TICKS 2
#define BUTTON_DEBOUNCE_TICKS 2
#define IDLE_SW_DEBOUNCE_TICKS 200

#define EEPROM_SIZE 1024
#define EEPROM_I2C_ADDR 0x50

#define EEPROM_POS_TEMP       0
#define EEPROM_POS_CONFIG_CS 30
#define EEPROM_POS_CONFIG    32

#define MENU_TIMEOUT       10000
#define TEMP_SAVE_INTERVAL 1000


// panic cutoff disables the heater until the mcu is reset
// this is at a relative temp of 430K assuming 12 uV/K and a VCC of 3.3V
#define PANIC_CUTOFF_ADC_VALUE ((430ULL * 12 * 501 * 4096) / 3300000) // ~430 C @ 3.3V

#define CFG_LIMIT_MIN 25
#define CFG_LIMIT_MAX 450

// 10s of seconds
#define CFG_STBY_DELAY_MIN  0
#define CFG_STBY_DELAY_MAX 30
#define CFG_STBY_TEMP_MIN   0
#define CFG_STBY_TEMP_MAX 350
// Minutes
#define CFG_STBY_TIME_MIN   0
#define CFG_STBY_TIME_MAX  30
// % of max power
#define CFG_STBY_IDLE_THRESHOLD_MIN   0
#define CFG_STBY_IDLE_THRESHOLD_MAX 100


#define CFG_CTRL_MAX_PWM_MIN      1
#define CFG_CTRL_MAX_PWM_MAX    100

// ms
#define CFG_CTRL_PWM_PERIOD_MIN    1
#define CFG_CTRL_PWM_PERIOD_MAX  100

#define CFG_CTRL_WND_SIZE_MIN     0
#define CFG_CTRL_WND_SIZE_MAX    30

//mK / Period
#define CFG_CTRL_INT_SPEED_MIN   1
#define CFG_CTRL_INT_SPEED_MAX 100

// K
#define CFG_CTRL_INT_LIMIT_MIN   1
#define CFG_CTRL_INT_LIMIT_MAX  10

// 1/10 uV
#define CFG_TC_UV_PER_DEG_MIN  85
#define CFG_TC_UV_PER_DEG_MAX 150

// Deg C
#define CFG_TC_COLDJUNCTION_MIN  0
#define CFG_TC_COLDJUNCTION_MAX 50


enum MenuSelection
{
	SCREEN_HOME,
	SCREEN_RAW,
	SCREEN_TEMP_LIMITS,
	SCREEN_STANDBY,
	SCREEN_CTRL_PARAM,
	SCREEN_TC_SETTINGS,
	SCREEN_RESTORE_DEFAULT
};
static const char *MAIN_MENU_TEXT[] = {"Home", "Raw values", "Temp limits", "Standby settings", "Heater controller", "Temp measurement", "Restore defaults"};

static void update_buttons(void);
static void update_stby_state(void);


/*
 * Basic mode of operation
 * HEATING    |    DEAD_TIME    |    ADC_SAMPLE    | ...
 *            ^                 ^                  ^
 *        Timer OC   Timer update (adc trg)   ADC cplt int
 */

volatile uint32_t tickcounter = 0;

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

config_t config;

const config_t default_config =
{
	.limit_max = 420,
	.limit_min = 100,
	.standby_temp = 150,
	.standby_delay = 0,
	.standby_time = 1,
	.idle_detect_threshold = 10,
	.controller_max_pwm = 70,
	.controller_period = 10,
	.controller_window = 10,
	.controller_int_limit = 8,
	.controller_int_speed = 10,
	.tc_v_per_deg = 110,
	.tc_offset = 30
};

typedef struct
{
	uint16_t *val_ptr;
	uint16_t bound_min;
	uint16_t bound_max;
	const char *entry_text_fmt;
	const char *detail_text;
	const char *detail_value_fmt;
} config_menu_entry_t;

#define ADC_VALUE_VREF 0
#define ADC_VALUE_VDC  1
#define ADC_VALUE_TC   2
volatile uint16_t adc_values[3];

// This is the panic shutdown
// Value is set to true, if the temperature exceeds the panic threshold
// It's never cleared, so device needs to be restart.
volatile bool kill_heater = false;


// derived from period and max_pwm
uint16_t heater_max_period;

uint16_t temp_set;
uint16_t temp_current;
uint16_t controller_offset1000 = 0;
uint32_t current_duty_smoothed = 0;
uint32_t current_duty_lt_avg100 = 0;

/*
int adc_tc_max = 0;
int adc_tc_min = 0;
int adc_v33_max = 0;
int adc_v33_min = 0;
*/

struct
{
	uint32_t idle_detect_timer;
	uint32_t timer;

	bool manual_off;
	bool off;
	bool stby;
	bool idle_sw;
} stby_state;

bool menu_timeout;


void sys_tick_handler(void)
{
	tickcounter++;
	update_buttons();
	update_stby_state();
}

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


void dma1_channel1_isr()
{
	// DMA interrupt from ADC DMA
	// only used for DMA1_TCIF
	dma_clear_interrupt_flags(DMA1, 1, DMA_TCIF);

	if (adc_values[ADC_VALUE_TC] > PANIC_CUTOFF_ADC_VALUE)
	{
		kill_heater = true;
	}

	/*
	if (adc_values[ADC_VALUE_TC] > adc_tc_max)
	{
		adc_tc_max = adc_values[ADC_VALUE_TC];
	}
	if (adc_values[ADC_VALUE_TC] < adc_tc_min)
	{
		adc_tc_min = adc_values[ADC_VALUE_TC];
	}
	if (adc_values[ADC_VALUE_VREF] > adc_v33_max)
	{
		adc_v33_max = adc_values[ADC_VALUE_VREF];
	}
	if (adc_values[ADC_VALUE_VREF] < adc_v33_min)
	{
		adc_v33_min = adc_values[ADC_VALUE_VREF];
	}

	adc_tc_max--;
	adc_tc_min++;
	adc_v33_max--;
	adc_v33_min++;
	*/

	// thermocouple voltahe in 0.1uV
	uint32_t vtc_uv10 = (((33000000ULL / 501) * ST_VREFINT_CAL) * adc_values[ADC_VALUE_TC]) / adc_values[ADC_VALUE_VREF] / 4096;

	// temperature in mK
	// Of course, our resolution isn't that high, but it makes our controller more stable if we use more precise values.
	uint32_t tc1000 = (1000 * vtc_uv10) / config.tc_v_per_deg + config.tc_offset * 1000;
	temp_current = tc1000 / 1000;

	uint16_t ts = temp_set;

	if (stby_state.off)
	{
		ts = 0;
	}
	else if (stby_state.stby && config.standby_temp < ts)
	{
		ts = config.standby_temp;
	}


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


	uint16_t new_period = 0;
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

void tim3_isr()
{
	// Only used for the OC interrupt
	// We should turn OFF the heater now
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


static void update_buttons(void)
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


static void reset_stby_state(void)
{
	stby_state.timer = tickcounter;
	stby_state.idle_detect_timer = tickcounter;
	stby_state.idle_sw = idle_sw.stable_state;
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

	if (idle_sw.stable_state)
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


static bool load_config(config_t *cfg)
{
	uint8_t *c = (uint8_t*)cfg;
	eeprom_read(EEPROM_POS_CONFIG, c, sizeof(*cfg));

	uint16_t cs;
	eeprom_read(EEPROM_POS_CONFIG_CS, (uint8_t*)&cs, sizeof(cs));

	printf("checksum in eeprom: 0x%04x\r\n", cs);

	uint16_t cs_gen;
	for (size_t i = 0; i < sizeof(*cfg); ++i)
	{
		cs_gen += c[i];
	}

	printf("calculated cs: 0x%04x\r\n", cs_gen);
	return cs == cs_gen;
}


static void update_config(void)
{
	printf("Check for config changes\r\n");
	config_t saved_config;

	// I don't care if the cs is OK or not
	load_config(&saved_config);

	if (saved_config.controller_period != config.controller_period)
	{
		printf("PWM period changed\r\n");
		// timer runs at 1MHz, so we set the period to the configured period * 1000
		timer_set_period(HEATER_TIMER, config.controller_period * 1000);
	}

	heater_max_period = (((uint32_t)config.controller_max_pwm) * config.controller_period * 1000) / 100;

	if (temp_set < config.limit_min)
	{
		temp_set = config.limit_min;
	}
	if (temp_set > config.limit_max)
	{
		temp_set = config.limit_max;
	}

	uint8_t *saved = (uint8_t*)&saved_config;
	uint8_t *current = (uint8_t*)&config;

	bool changed = false;

	for (uint16_t pos = 0; pos < sizeof(config); pos++)
	{
		uint16_t start = pos;
		while (saved[pos] != current[pos] && pos < sizeof(config))
		{
			pos++;
		}

		if (pos != start)
		{
			changed = true;
			printf("Update config in eeprom from %u to %u\r\n", start, pos);
			eeprom_write(EEPROM_POS_CONFIG + start, current + start, pos - start);
		}
	}

	if (changed)
	{
		// update cs
		uint16_t cs = 0;
		for (uint16_t pos = 0; pos < sizeof(config); pos++)
		{
			cs += current[pos];
		}
		printf("write new checksum: 0x%04x\r\n", cs);
		eeprom_write(EEPROM_POS_CONFIG_CS, (uint8_t*)&cs, sizeof(cs));
	}
}


static void delay_or_evt(uint32_t ticks)
{
	bool old_esw = encoder_sw.stable_state;
	uint32_t start = tickcounter;
	while (tickcounter - start > ticks)
	{
		if (encoder.ticks != 0 ||
			old_esw != encoder_sw.stable_state)
		{
			return;
		}
	}
}


static void menu_wait()
{
	uint32_t timeout = tickcounter;
	bool old_esw = encoder_sw.stable_state;
	while (encoder_sw.stable_state == old_esw && encoder.ticks == 0)
	{
		if (tickcounter - timeout > MENU_TIMEOUT)
		{
			menu_timeout = true;
			return;
		}
	}
}


static uint32_t add_encoder_value_bound(uint32_t val, uint32_t lower_bound, uint32_t upper_bound)
{
	int t = encoder.ticks;
	encoder.ticks = 0;

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


static uint8_t show_menu(char const* const* text, uint8_t cnt, uint8_t sel)
{
	if (menu_timeout)
	{
		return 0;
	}

	uint8_t entry = sel;
	encoder.ticks = 0;

	bool sw_dn = encoder_sw.stable_state;
	bool sw_evt = false;
	printf("show menu with %u entries\r\n", cnt);

	while (1)
	{
		uint8_t page = entry / 3; // 3 menu entries per page

		ssd1306_Fill(Black);

		for (uint8_t e = page * 3; e < cnt && e < (page + 1) * 3; ++e)
		{
			uint8_t ofs = (e - page * 3) * 11;
			ssd1306_SetCursor(1, ofs);
			if (entry == e)
			{
				// draw highlighted
				for (uint8_t y = ofs; y < ofs + 10; ++y)
				{
					for (uint8_t x = 0; x < 128; ++x)
					{
						ssd1306_DrawPixel(x, y, White);
					}
				}
				ssd1306_WriteStringComp(text[e], CompFont_7x10, Black);
			}
			else
			{
				// draw normal
				ssd1306_WriteStringComp(text[e], CompFont_7x10, White);
			}
		}

		ssd1306_UpdateScreen();

		// Wait for some event
		menu_wait();
		if (menu_timeout)
		{
			return 0;
		}

		if (encoder_sw.stable_state && !sw_dn)
		{
			sw_dn = true;
			sw_evt = true;
		}
		sw_dn = encoder_sw.stable_state;
		if (sw_evt && !sw_dn)
		{
			printf("show menu selection: %u\r\n", entry);
			return entry;
		}

		if (encoder.ticks != 0)
		{
			sw_evt = false;
		}

		entry = add_encoder_value_bound(entry, 0, cnt - 1);
	}
}


static void show_config_screen(const config_menu_entry_t *entry)
{
	char disp_buf[16];

	while (!encoder_sw.stable_state)
	{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(64 - (strlen(entry->detail_text) * 7) / 2, 0);
		ssd1306_WriteStringComp(entry->detail_text, CompFont_7x10, White);

		snprintf(disp_buf, sizeof(disp_buf), entry->detail_value_fmt, *entry->val_ptr);
		size_t l = strlen(disp_buf);
		ssd1306_SetCursor(64 - (l * 11) / 2, 32 - 18);
		ssd1306_WriteStringComp(disp_buf, CompFont_11x18, Black);
		ssd1306_UpdateScreen();

		delay_or_evt(0xffffffff);
		(*entry->val_ptr) = add_encoder_value_bound(*entry->val_ptr, entry->bound_min, entry->bound_max);
	}
}


static void show_config_menu(const config_menu_entry_t *entries, uint8_t num_of_entries)
{
	const char *menu_entries[6];
	char text_buffer[5][19];
	if (num_of_entries >= sizeof(menu_entries) / sizeof(menu_entries[0]))
	{
		return;
	}

	menu_entries[num_of_entries] = "<- Back";

	uint8_t sel = 0;
	while (1)
	{
		for (uint8_t e = 0; e < num_of_entries; e++)
		{
			snprintf(text_buffer[e], sizeof(text_buffer[e]), entries[e].entry_text_fmt, *entries[e].val_ptr);
			menu_entries[e] = text_buffer[e];
		}

		sel = show_menu(menu_entries, num_of_entries + 1, sel);

		if (menu_timeout || sel == num_of_entries)
		{
			// "Back"
			update_config();
			return;
		}
		show_config_screen(entries + sel);
	}
}


static void screen_home(void)
{
	uint32_t sw_dn = 0;
	char disp_buf[32];
	uint32_t temp_changed_time;
	bool temp_changed = false;
	while (1)
	{
		ssd1306_Fill(Black);

		itoa(temp_current, disp_buf, 10);
		for (uint8_t k = 0; k < 4; k++)
		{
			if (!disp_buf[k])
			{
				ssd1306_SetCursor((3 - k) * 16, 3);
				break;
			}
			disp_buf[k] -= '0' - ' ';
		}
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
			ssd1306_SetCursor(128 - 5 * 7,22);
			ssd1306_WriteStringComp("ERROR", CompFont_7x10, Black);
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
		delay_or_evt(100);
		if (encoder_sw.stable_state)
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
		temp_set = add_encoder_value_bound(temp_set, config.limit_min, config.limit_max);

		if (temp_set != old_temp)
		{
			temp_changed = true;
			temp_changed_time = tickcounter;
		}

		if (temp_changed && tickcounter - temp_changed_time > TEMP_SAVE_INTERVAL)
		{
			printf("Saving new temp: %u/r/n", temp_set);
			eeprom_write(EEPROM_POS_TEMP, (const uint8_t*)&temp_set, sizeof(temp_set));
			temp_changed = false;
		}
	}
}


static void screen_raw(void)
{
	while (!encoder_sw.stable_state)
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


static void screen_config_limits(void)
{
	static const config_menu_entry_t entries[] =
		{
			{.val_ptr = &config.limit_min, .bound_min = CFG_LIMIT_MIN, .bound_max = CFG_LIMIT_MAX, .entry_text_fmt = "Min temp %8uC", .detail_text = "Lower temp limit", .detail_value_fmt = "%u C"},
			{.val_ptr = &config.limit_max, .bound_min = CFG_LIMIT_MIN, .bound_max = CFG_LIMIT_MAX, .entry_text_fmt = "Max temp %8uC", .detail_text = "Upper temp limit", .detail_value_fmt = "%u C"}
		};

	show_config_menu(entries, sizeof(entries) / sizeof(entries[0]));
}


static void screen_config_standby(void)
{
	static const config_menu_entry_t entries[] =
		{
			{.val_ptr = &config.standby_temp, .bound_min = CFG_STBY_TEMP_MIN, .bound_max = CFG_LIMIT_MAX, .entry_text_fmt = "Stby temp %7uC", .detail_text = "Standby temp", .detail_value_fmt = "%u C"},
			{.val_ptr = &config.standby_delay, .bound_min = CFG_STBY_DELAY_MIN, .bound_max = CFG_STBY_DELAY_MAX, .entry_text_fmt = "Stby delay %5u0s", .detail_text = "Standby delay", .detail_value_fmt = "%u0 s"},
			{.val_ptr = &config.standby_time, .bound_min = CFG_STBY_TIME_MIN, .bound_max = CFG_STBY_TIME_MAX, .entry_text_fmt = "Pwrdn delay %3umin", .detail_text = "Powerdown delay", .detail_value_fmt = "%u min"},
			{.val_ptr = &config.idle_detect_threshold, .bound_min = CFG_STBY_IDLE_THRESHOLD_MIN, .bound_max = CFG_STBY_IDLE_THRESHOLD_MAX, .entry_text_fmt = "Idle thd %8u%%", .detail_text = "Idle threshold", .detail_value_fmt = "%u%%"}
		};

	show_config_menu(entries, sizeof(entries) / sizeof(entries[0]));
}


static void screen_config_ctrl_param(void)
{
	static const config_menu_entry_t entries[] =
		{
			{.val_ptr = &config.controller_max_pwm, .bound_min = CFG_CTRL_MAX_PWM_MIN, .bound_max = CFG_CTRL_MAX_PWM_MAX, .entry_text_fmt = "Max PWM duty %4u%%", .detail_text = "Max duty cycle", .detail_value_fmt = "%u %%"},
			{.val_ptr = &config.controller_period, .bound_min = CFG_CTRL_PWM_PERIOD_MIN, .bound_max = CFG_CTRL_PWM_PERIOD_MAX, .entry_text_fmt = "PWM period %5ums", .detail_text = "PWM period", .detail_value_fmt = "%ums"},
			{.val_ptr = &config.controller_window, .bound_min = CFG_CTRL_WND_SIZE_MIN, .bound_max = CFG_CTRL_WND_SIZE_MAX, .entry_text_fmt = "Reg wnd size %4uC", .detail_text = "Reg window size", .detail_value_fmt = "%u C"},
			{.val_ptr = &config.controller_int_limit, .bound_min = CFG_CTRL_INT_LIMIT_MIN, .bound_max = CFG_CTRL_INT_LIMIT_MAX, .entry_text_fmt = "Int. ofs limit %2uC", .detail_text = "Integral limit", .detail_value_fmt = "%u C"},
			{.val_ptr = &config.controller_int_speed, .bound_min = CFG_CTRL_INT_SPEED_MIN, .bound_max = CFG_CTRL_INT_SPEED_MAX, .entry_text_fmt = "Int. speed %3umK/p", .detail_text = "Integral speed", .detail_value_fmt = "%u mK/p"}
		};

	show_config_menu(entries, sizeof(entries) / sizeof(entries[0]));
}

static void screen_config_tc_settings(void)
{
	static const config_menu_entry_t entries[] =
		{
			{.val_ptr = &config.tc_v_per_deg, .bound_min = CFG_TC_UV_PER_DEG_MIN, .bound_max = CFG_TC_UV_PER_DEG_MAX, .entry_text_fmt = "uV/C %4u x 1/10uV", .detail_text = "TC sensitivity", .detail_value_fmt = "%ux1/10uV"},
			{.val_ptr = &config.tc_offset, .bound_min = CFG_TC_COLDJUNCTION_MIN, .bound_max = CFG_TC_COLDJUNCTION_MAX, .entry_text_fmt = "Cold junction %3uC", .detail_text = "Cold junction temp", .detail_value_fmt = "%u C"}
		};

	show_config_menu(entries, sizeof(entries) / sizeof(entries[0]));
}


static void load_default_config()
{
	config = default_config;
	update_config();

	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteStringComp("Default cfg loaded", CompFont_7x10, White);

	ssd1306_SetCursor(58, 32 - 18);
	ssd1306_WriteStringComp("OK", CompFont_11x18, Black);
	ssd1306_UpdateScreen();

	while(encoder_sw.stable_state);
	while(!encoder_sw.stable_state);
}


static void screen_restore_defaults(void)
{
	stby_state.off = true;
	bool yes = false;
	while (1)
	{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteStringComp("Load default cfg?", CompFont_7x10, White);


		ssd1306_SetCursor(20, 32 - 18);
		ssd1306_WriteStringComp("YES", CompFont_11x18, yes ? Black : White);

		ssd1306_SetCursor(128 - 20 - 11 * 2, 32 - 18);
		ssd1306_WriteStringComp("NO", CompFont_11x18, yes ? White : Black);

		ssd1306_UpdateScreen();

		menu_wait();
		if (menu_timeout)
		{
			return;
		}

		if (encoder.ticks > 0)
		{
			yes = false;
		}
		else if (encoder.ticks < 0)
		{
			yes = true;
		}
		encoder.ticks = 0;

		if (encoder_sw.stable_state)
		{
			while (1)
			{
				if (encoder.ticks != 0)
				{
					break;
				}
				if (!encoder_sw.stable_state)
				{
					if (yes)
					{
						// load default
						load_default_config();
					}
					return;
				}
			}
		}
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

	encoder.phase = (gpio_get(GPIOA, GPIO_ENCODER_A) != 0);
	encoder_sw.raw_state = !gpio_get(GPIOA, GPIO_ENCODER_SW);
	idle_sw.raw_state = !gpio_get(GPIOA, GPIO_IDLE_SW);

	// load config
	if (load_config(&config))
	{
		timer_set_period(HEATER_TIMER, config.controller_period * 1000);
		heater_max_period = (((uint32_t)config.controller_max_pwm) * config.controller_period * 1000) / 100;
	}
	else
	{
		load_default_config();
	}

	eeprom_read(EEPROM_POS_TEMP, (uint8_t*)&temp_set, sizeof(temp_set));
	if (temp_set < config.limit_min)
	{
		temp_set = config.limit_min;
	}
	if (temp_set > config.limit_max)
	{
		temp_set = config.limit_max;
	}

	// Don't turn on automatically as soon as there is power.
	stby_state.manual_off = true;
    reset_stby_state();

	iwdg_reset();

    // initially start the ADC sample sequence by starting the timer
	timer_enable_counter(HEATER_TIMER);


	enum MenuSelection sel = SCREEN_HOME;

	while (1)
	{
		if (menu_timeout)
		{
			sel = SCREEN_HOME;
		}

		menu_timeout = false;
		switch (sel)
		{
			case SCREEN_RAW:
				screen_raw();
				break;

			case SCREEN_TEMP_LIMITS:
				screen_config_limits();
				break;

			case SCREEN_STANDBY:
				screen_config_standby();
				break;

			case SCREEN_CTRL_PARAM:
				screen_config_ctrl_param();
				break;

			case SCREEN_TC_SETTINGS:
				screen_config_tc_settings();
				break;

			case SCREEN_RESTORE_DEFAULT:
				screen_restore_defaults();
				break;

			case SCREEN_HOME:
			default:
				screen_home();
				break;
		}

		sel = show_menu(MAIN_MENU_TEXT, sizeof(MAIN_MENU_TEXT) / sizeof(MAIN_MENU_TEXT[0]), sel);
	}
}

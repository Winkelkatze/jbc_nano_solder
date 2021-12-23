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


#include "config.h"

#include "tinyprintf.h"
#include "eeprom_i2c.h"

#define EEPROM_POS_TEMP       0
#define EEPROM_POS_CONFIG_CS 30
#define EEPROM_POS_CONFIG    32

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
	.controller_int_limit = 3,
	.controller_int_speed = 10,
	.tc_v_per_deg = 90,
	.tc_offset = 30
};

config_t config;

static bool load_config(config_t *cfg)
{
	uint8_t *c = (uint8_t*)cfg;
	eeprom_read(EEPROM_POS_CONFIG, c, sizeof(*cfg));

	uint16_t cs;
	eeprom_read(EEPROM_POS_CONFIG_CS, (uint8_t*)&cs, sizeof(cs));

	printf("checksum in eeprom: 0x%04x\r\n", cs);

	uint16_t cs_gen = 0;
	for (size_t i = 0; i < sizeof(*cfg); ++i)
	{
		cs_gen += c[i];
	}

	printf("calculated cs: 0x%04x\r\n", cs_gen);
	return cs == cs_gen;
}


bool config_load(void)
{
	return load_config(&config);
}

void config_load_default(void)
{
	config = default_config;
}

void config_save(void)
{
	printf("Check for config changes\r\n");
	config_t saved_config;

	// I don't care if the cs is OK or not
	load_config(&saved_config);

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
	printf("Config save done\r\n");
}


uint16_t config_load_last_temperature(void)
{
	uint16_t ret;
	eeprom_read(EEPROM_POS_TEMP, (uint8_t*)&ret, sizeof(ret));
	return ret;
}

void config_set_last_temperature(uint16_t temp)
{
	eeprom_write(EEPROM_POS_TEMP, (const uint8_t*)&temp, sizeof(temp));
}

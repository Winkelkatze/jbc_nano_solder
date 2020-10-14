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


#include "menu.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "tinyprintf.h"
#include "ssd1306.h"

#include "input.h"
#include "config.h"

#define MENU_TIMEOUT       10000

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

typedef struct
{
	uint16_t *val_ptr;
	uint16_t bound_min;
	uint16_t bound_max;
	const char *entry_text_fmt;
	const char *detail_text;
	const char *detail_value_fmt;
} config_menu_entry_t;

enum
{
	SCREEN_HOME,
	SCREEN_RAW,
	SCREEN_TEMP_LIMITS,
	SCREEN_STANDBY,
	SCREEN_CTRL_PARAM,
	SCREEN_TC_SETTINGS,
	SCREEN_RESTORE_DEFAULT,
	SCREEN_BACK // same as home
};
static const char *MAIN_MENU_TEXT[] = {"Home", "Raw values", "Temp limits", "Standby settings", "Heater controller", "Temp measurement", "Restore defaults", "<- Back"};

extern uint32_t tickcounter;


static bool menu_timeout;


static void menu_wait()
{
	uint32_t timeout = tickcounter;
	bool old_esw = input_encoder_switch();
	while (input_encoder_switch() == old_esw && !input_encoder_has_ticks())
	{
		if (tickcounter - timeout > MENU_TIMEOUT)
		{
			menu_timeout = true;
			return;
		}
	}
}

static uint8_t show_menu(char const* const* text, uint8_t cnt, uint8_t sel)
{
	if (menu_timeout)
	{
		return 0;
	}

	uint8_t entry = sel;
	input_encoder_ticks_reset();

	bool sw_dn = input_encoder_switch();
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

		if (input_encoder_switch() && !sw_dn)
		{
			sw_dn = true;
			sw_evt = true;
		}
		sw_dn = input_encoder_switch();
		if (sw_evt && !sw_dn)
		{
			printf("show menu selection: %u\r\n", entry);
			return entry;
		}

		if (input_encoder_has_ticks())
		{
			sw_evt = false;
		}

		entry = input_add_encoder_value_bound(entry, 0, cnt - 1);
	}
}


static void show_config_screen(const config_menu_entry_t *entry)
{
	char disp_buf[16];

	while (!input_encoder_switch())
	{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(64 - (strlen(entry->detail_text) * 7) / 2, 0);
		ssd1306_WriteStringComp(entry->detail_text, CompFont_7x10, White);

		snprintf(disp_buf, sizeof(disp_buf), entry->detail_value_fmt, *entry->val_ptr);
		size_t l = strlen(disp_buf);
		ssd1306_SetCursor(64 - (l * 11) / 2, 32 - 18);
		ssd1306_WriteStringComp(disp_buf, CompFont_11x18, Black);
		ssd1306_UpdateScreen();

		input_delay_or_evt(0xffffffff);
		(*entry->val_ptr) = input_add_encoder_value_bound(*entry->val_ptr, entry->bound_min, entry->bound_max);
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
			return;
		}
		show_config_screen(entries + sel);
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


static void screen_restore_defaults(void)
{
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

		int t = input_encoder_ticks_reset();
		if (t > 0)
		{
			yes = false;
		}
		else if (t < 0)
		{
			yes = true;
		}

		if (input_encoder_switch())
		{
			while (1)
			{
				if (input_encoder_has_ticks())
				{
					break;
				}
				if (!input_encoder_switch())
				{
					if (yes)
					{
						// load default
						menu_restore_default_config();
					}
					return;
				}
			}
		}
	}


}

menu_result_t menu_main(void)
{
	uint8_t sel = SCREEN_HOME;

	while (1)
	{
		menu_timeout = false;
		sel = show_menu(MAIN_MENU_TEXT, sizeof(MAIN_MENU_TEXT) / sizeof(MAIN_MENU_TEXT[0]), sel);
		if (menu_timeout)
		{
			return menu_result_home;
		}
		switch (sel)
		{
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

			case SCREEN_RAW:
				return menu_result_show_raw;
				break;

			case SCREEN_HOME:
			case SCREEN_BACK:
			default:
				return menu_result_home;
		}
		if (menu_timeout)
		{
			return menu_result_home;
		}
	}
}

void menu_restore_default_config(void)
{
	config_load_default();

	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteStringComp("Default cfg loaded", CompFont_7x10, White);

	ssd1306_SetCursor(58, 32 - 18);
	ssd1306_WriteStringComp("OK", CompFont_11x18, Black);
	ssd1306_UpdateScreen();

	while(input_encoder_switch());
	while(!input_encoder_switch());
}

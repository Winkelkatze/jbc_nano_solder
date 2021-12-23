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


#include "eeprom_i2c.h"

#include <string.h>
#include <libopencm3/stm32/i2c.h>

#define EEPROM_PAGE_SIZE 8

void eeprom_read(uint16_t ofs, uint8_t *data, uint16_t size)
{
	if (ofs + size > EEPROM_SIZE || ofs + size < ofs)
	{
		return;
	}

	while (size)
	{
		uint8_t ofs_in_page = ofs & 0xff;
		uint8_t page = ofs >> 8;

		uint8_t cur = size > 255 ? 255 : size;

		if (((uint16_t)ofs_in_page) + cur > 0xff)
		{
			cur = 0x100 - ofs_in_page;
		}

		//printf("eep read: addr p=%u ofs=%u cur=%u\r\n", page, ofs_in_page, cur);

		// set address
		i2c_transfer7(EEPROM_I2C_PORT, EEPROM_I2C_ADDR + page, &ofs_in_page, 1, NULL, 0);

		// read transfer
		i2c_transfer7(EEPROM_I2C_PORT, EEPROM_I2C_ADDR + page, NULL, 0, data, cur);

		size -= cur;
		data += cur;
		ofs += cur;
	}
}

void eeprom_write(uint16_t ofs, const uint8_t *data, uint16_t size)
{
	uint8_t pagebuf[EEPROM_PAGE_SIZE + 1];

	if (ofs + size > EEPROM_SIZE || ofs + size < ofs)
	{
		return;
	}

	while (size)
	{
		uint8_t ofs_in_page = ofs & 0xff;
		uint8_t page = ofs >> 8;

		uint8_t cur = size > EEPROM_PAGE_SIZE ? EEPROM_PAGE_SIZE : size;

		// Ensure aligned
		uint16_t line_end = ((ofs / EEPROM_PAGE_SIZE) + 1) * EEPROM_PAGE_SIZE;
		if (ofs_in_page + cur > line_end)
		{
			cur = line_end - ofs_in_page;
		}

		//printf("eep write: addr p=%u ofs=%u cur=%u\r\n", page, ofs_in_page, cur);

		pagebuf[0] = ofs_in_page;
		memcpy(pagebuf + 1, data, cur);

		// write data
		i2c_transfer7(EEPROM_I2C_PORT, EEPROM_I2C_ADDR + page, pagebuf, cur + 1, NULL, 0);

		size -= cur;
		data += cur;
		ofs += cur;
	}
}

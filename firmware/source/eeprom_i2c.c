#include "eeprom_i2c.h"

#include <string.h>
#include <libopencm3/stm32/i2c.h>

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
	uint8_t pagebuf[17];

	if (ofs + size > EEPROM_SIZE || ofs + size < ofs)
	{
		return;
	}

	while (size)
	{
		uint8_t ofs_in_page = ofs & 0xff;
		uint8_t page = ofs >> 8;

		uint8_t cur = size > 16 ? 16 : size;

		if (((uint16_t)ofs_in_page) + cur > 0xff)
		{
			cur = 0x100 - ofs_in_page;
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

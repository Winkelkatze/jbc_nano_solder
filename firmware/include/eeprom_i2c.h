#include <stdint.h>

#define EEPROM_SIZE 1024
#define EEPROM_I2C_ADDR 0x50
#define EEPROM_I2C_PORT I2C1

void eeprom_read(uint16_t ofs, uint8_t *data, uint16_t size);
void eeprom_write(uint16_t ofs, const uint8_t *data, uint16_t size);

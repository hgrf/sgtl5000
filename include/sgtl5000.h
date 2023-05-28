#ifndef SGTL5000_H
#define SGTL5000_H

#include "driver/i2c.h"

esp_err_t sgtl5000_init(i2c_port_t port, uint8_t i2c_address);
esp_err_t sgtl5000_set_volume(uint8_t volume);

#endif // SGTL5000_H

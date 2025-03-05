#ifndef ADXL_DRIVER_H
#define ADXL_DRIVER_H

#include <Arduino.h>

void    adxl_setup();
void    adxl_enable_motion_detection();
void    adxl_disable_motion_detection();
uint8_t i2c_read(uint8_t slave_addr, uint8_t reg);
void    i2c_write(uint8_t slave_addr, uint8_t reg, uint8_t data);
void    i2c_read_bytes(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint8_t len);

#endif  // ADXL_DRIVER_H


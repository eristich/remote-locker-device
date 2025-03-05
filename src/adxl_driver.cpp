#include <Arduino.h>
#include <Wire.h>
#include <locker_defines.hpp>
#include <adxl_driver.hpp>

uint8_t i2c_read(uint8_t slave_addr, uint8_t reg) {
    Wire.beginTransmission(slave_addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(slave_addr, (uint8_t)1);
    uint8_t data = Wire.read();

    return data;
}

void i2c_write(uint8_t slave_addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(slave_addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// read multiple i2c bytes with Wire.readBytes()
void i2c_read_bytes(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(slave_addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(slave_addr, len);
    Wire.readBytes(data, len);
}

void adxl_setup() {
    i2c_write(ADXL_ADDR, POWER_CTL, 0b00001000); // Set to measure mode
    // Configure le seuil d'activité (exemple : 1g = 16 LSB)
    i2c_write(ADXL_ADDR, THRESH_ACT, 70); // 50
    // Active la détection de mouvement sur X, Y, Z en mode DC
    i2c_write(ADXL_ADDR, ACT_INACT_CTL, 0x70);
}

void adxl_enable_motion_detection() {
    // Active l'interruption d'activité
    i2c_write(ADXL_ADDR, INT_ENABLE, 0x10);
    // Mappe l'interruption sur INT1 (0 = INT1, 1 = INT2)
    i2c_write(ADXL_ADDR, INT_MAP, 0x00);
    // Active l'interruption sur INT1
    // attachInterrupt(digitalPinToInterrupt(ESP32_PIN_INT1), adxl_motion_isr, RISING);
}

void adxl_disable_motion_detection() {
    // Désactive l'interruption d'activité
    i2c_write(ADXL_ADDR, INT_ENABLE, 0x00);
}
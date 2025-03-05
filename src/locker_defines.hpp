#ifndef LOCKER_DEFINES_HPP
#define LOCKER_DEFINES_HPP

#define TINY_GSM_MODEM_SIM7080

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon           Serial
#define SerialAT            Serial1
#define SerialGPS           Serial2
#define TINY_GSM_RX_BUFFER  650
#define TINY_GSM_DEBUG      SerialMon
#define GSM_AUTOBAUD_MIN    9600
#define GSM_AUTOBAUD_MAX    115200
#define TINY_GSM_USE_GPRS   true
#define TINY_GSM_USE_WIFI   false

#define ADXL_ADDR           0x53
#define DEVID               0x00
#define POWER_CTL           0x2D
#define DATAX0              0x32
#define DATAX1              0x33
#define DATAY0              0x34
#define DATAY1              0x35
#define DATAZ0              0x36
#define DATAZ1              0x37
#define THRESH_ACT          0x24
#define ACT_INACT_CTL       0x27
#define INT_ENABLE          0x2E
#define INT_MAP             0x2F
#define INT_SOURCE          0x30

#define ESP32_PIN_INT1      0x04
#define ESP32_PIN_LED       0x44
#define ESP32_PIN_BUTTON    0x07
#define ESP32_PIN_MOTOR     0x03

#define ESP32_GPS_RX        0x09
#define ESP32_GPS_TX        0x08
#define ESP32_GPS_BAUD      9600

#define ESP32_GSM_RX        0x01
#define ESP32_GSM_TX        0x02
#define ESP32_GSM_BAUD      115200

#define ESP32_MOTOR_PIN     0x03

#endif // LOCKER_DEFINES_HPP
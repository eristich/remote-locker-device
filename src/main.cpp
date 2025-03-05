#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <adxl_driver.hpp>

#include <locker_defines.hpp>

// Server details
const char server[]   =         "admin.eristich.dev";
const char resource[] =         "/webhook";
const int  port       =         443;
const char webhook_api_key[] =  "<API_KEY>";

const char apn[]      = "TM";

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
// HardwareSerial ss(2);
// SoftwareSerial ss(ESP32_GPS_RX, ESP32_GPS_TX);

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

TinyGsm             modem(SerialAT);
TinyGsmClientSecure client(modem);
HttpClient          http(client, server, port);

Servo motor;

bool is_track_position_mode_enabled = false;
bool adxl345_int_on = false;
bool is_motion_detected_by_interruption = false;
bool is_lock_mode_enabled = false;

// define interrupt service routine
void IRAM_ATTR adxl_motion_isr() {
  is_motion_detected_by_interruption = true;
    log_e("[INT1] Motion Interrupt from ADXL345");
}

void connectToNetwork() {
  SerialMon.println("Initialisation du modem...");
  if (!modem.restart()) {
    SerialMon.println("Échec de l'initialisation du modem");
      return;
  }

  if ("" && modem.getSimStatus() != 3) { modem.simUnlock(""); }
  modem.setNetworkMode(13);

  SerialMon.print("Waiting for SIM network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

  SerialMon.println("Connexion au réseau GPRS...");
  while (!modem.gprsConnect(apn, "", "")) {
    SerialMon.println("Échec de la connexion au réseau, nouvelle tentative...");
      delay(3000);
  }
  SerialMon.println("Connecté au réseau avec succès!");
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  SerialMon.begin(115200);
  delay(500);

  SerialMon.print("[INFO] Setup GPS Module...");
  SerialGPS.begin(ESP32_GPS_BAUD, SERIAL_8N1, ESP32_GPS_RX, ESP32_GPS_TX);
  SerialMon.println(" OK");

  delay(500);

  SerialMon.print("[INFO] Setup GSM Network Module...");
  SerialAT.begin(115200, SERIAL_8N1, ESP32_GSM_RX, ESP32_GSM_TX);
  SerialMon.println(" OK");

  delay(500);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("[INFO] Initializing GSM Modem connection...");
  connectToNetwork();

  delay(100);

  // ADXL345
  SerialMon.print("[INFO] Setup ADXL345 Module...");
  adxl_setup();
  SerialMon.println(" OK");

  SerialMon.print("[INFO] Setup LED pin...");
  pinMode(ESP32_PIN_LED, OUTPUT);
  SerialMon.println(" OK");

  SerialMon.print("[INFO] Setup INTERRUPT pin...");
  pinMode(ESP32_PIN_INT1, INPUT_PULLUP);
  SerialMon.println(" OK");

  delay(50);

  SerialMon.print("[INFO] Setup STARTUP BUTTON pin...");
  pinMode(ESP32_PIN_BUTTON, INPUT_PULLUP);
  SerialMon.println(" OK");

  SerialMon.print("[INFO] Setup Setup MOTOR...");
  motor.attach(ESP32_MOTOR_PIN);          // Attache le servo
  motor.write(0);                         // Position initiale du servo à 0°
  SerialMon.println(" OK");

  i2c_read(ADXL_ADDR, INT_SOURCE);
  delay(100);
}

void loop() {
  if (digitalRead(ESP32_PIN_BUTTON) == LOW) {
    is_lock_mode_enabled = !is_lock_mode_enabled;
    if (is_lock_mode_enabled) {
      SerialMon.println("Mode verrouillage activé");
      motor.write(180);
      adxl_enable_motion_detection();
      delay(250);
      i2c_read(ADXL_ADDR, INT_SOURCE);
      attachInterrupt(digitalPinToInterrupt(ESP32_PIN_INT1), adxl_motion_isr, RISING);
    } else {
      SerialMon.println("Mode verrouillage désactivé");
      motor.write(0);
      is_track_position_mode_enabled = false;
      adxl_disable_motion_detection();
      delay(250);
      i2c_read(ADXL_ADDR, INT_SOURCE);
      detachInterrupt(digitalPinToInterrupt(ESP32_PIN_INT1));
    }
    delay(400);
  }

  if (is_motion_detected_by_interruption) {
    is_motion_detected_by_interruption = false;

    SerialMon.println("Mouvement détecté !");
    digitalWrite(ESP32_PIN_LED, HIGH);
    delay(1000);
    digitalWrite(ESP32_PIN_LED, LOW);

    // Lire les données pour effacer l'interruption (accusé de reception)
    i2c_read(ADXL_ADDR, INT_SOURCE);
    detachInterrupt(digitalPinToInterrupt(ESP32_PIN_INT1));
    is_track_position_mode_enabled = true;
  }

  if (is_track_position_mode_enabled) {
    while (SerialGPS.available() > 0) {
      if (gps.encode(SerialGPS.read())) {
        if (gps.location.isValid()) {
          // encode latitude and longitude in JSON format
          delay(300);
          String payload = "{\"latitude\": \"" + String(gps.location.lat(), 6) + "\", \"longitude\": \"" + String(gps.location.lng(), 6) + "\"}";
          // String payload = "{\"latitude\": \"45.189863\", \"longitude\": \"5.708954\"}";

          if (!modem.isNetworkConnected() || !modem.isGprsConnected()) {
            SerialMon.println("Network status: " + String(modem.isNetworkConnected()));
            SerialMon.println("GPRS status: " + String(modem.isGprsConnected()));
            SerialMon.println("Perte de connexion, tentative de reconnexion...");
            connectToNetwork();
          }
          String httpRaw = "POST /webhook HTTP/1.1\r\nHost: weblocker-staging.eristich.dev\r\nX-Api-Key: " + String(webhook_api_key) + "\r\nContent-Type: application/json\r\nContent-Length: " + String(payload.length()) + "\r\n\r\n" + payload;
          SerialMon.println(httpRaw);

          // Données à envoyer
          if (client.connect(server, port)) {
            client.println(httpRaw);
            unsigned long timeout = millis();
            while (client.connected() && millis() - timeout < 10000L) {
              while (client.available()) {
                String line = client.readStringUntil('\n');
                SerialMon.println(line);
                // char c = client.read();
                // Serial.print(c);
                timeout = millis();
              }
            }
            client.stop();
          }
          delay(5500);
        } 
          else 
        {
          SerialMon.println("Location not valid");
        }
      }
    }
  }
}
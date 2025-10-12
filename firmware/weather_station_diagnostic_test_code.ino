/********************************************************************
  PogodaAI - Diagnostic Test Code
  Author: Kamil Gruszczyński
  Team: Krasiniak TechWorks
  Purpose:
  - Check if all modules respond correctly
  - Print "OK" or "ERROR" for each component
********************************************************************/

#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include "Zanshin_BME680.h"
#include "Adafruit_PM25AQI.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <HardwareSerial.h>
#include <Adafruit_INA219.h>

#define RAIN_PIN 33
#define LIGHT_PIN 32
#define ANEMOMETER_PIN 27
#define SD_CS 5
#define PMS_RX 17
#define PMS_TX 16
#define I2C_SDA 21
#define I2C_SCL 22

RTC_DS3231 rtc;
BME680_Class bme;
Adafruit_PM25AQI aqi;
Adafruit_INA219 ina219;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
HardwareSerial pmSerial(2);
LiquidCrystal_I2C lcd(0x27, 20, 4);

volatile uint16_t anemometerCount = 0;
void IRAM_ATTR anemometerISR() { anemometerCount++; }

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== PogodaAI Diagnostic Test ===");

  // --- I2C init ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C initialized.");

  // --- LCD ---
  lcd.init();
  lcd.backlight();
  lcd.print("Diagnostic...");
  Serial.print("LCD: ");
  lcd.setCursor(0, 1);
  lcd.print("LCD OK");
  Serial.println("OK");

  // --- RTC ---
  Serial.print("RTC DS3231: ");
  if (rtc.begin()) {
    Serial.println("OK");
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else Serial.println("ERROR");

  // --- BME680 ---
  Serial.print("BME680: ");
  if (bme.begin(I2C_STANDARD_MODE)) {
    Serial.println("OK");
    int32_t t, h, p, g;
    bme.getSensorData(t, h, p, g);
    Serial.printf("  T: %.2f*C, H: %.2f%%, P: %.2fhPa, G: %.2fKOhm\n",
                  t / 100.0, h / 1000.0, p / 100.0, g / 100.0);
  } else Serial.println("ERROR");

  // --- PMSA003 ---
  Serial.print("PMSA003: ");
  pmSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);
  delay(2000);
  if (aqi.begin_UART(&pmSerial)) {
    Serial.println("OK");
    PM25_AQI_Data data;
    if (aqi.read(&data))
      Serial.printf("  PM1:%d  PM2.5:%d  PM10:%d\n", data.pm10_standard, data.pm25_standard, data.pm100_standard);
    else Serial.println("  (No data yet)");
  } else Serial.println("ERROR");

  // --- INA219 ---
  Serial.print("INA219 (power): ");
  if (ina219.begin()) {
    Serial.println("OK");
    float busV = ina219.getBusVoltage_V();
    Serial.printf("  Voltage: %.2f V\n", busV);
  } else Serial.println("ERROR");

  // --- Magnetometer ---
  Serial.print("HMC5883L (magnetometer): ");
  if (mag.begin()) {
    Serial.println("OK");
    sensors_event_t event;
    mag.getEvent(&event);
    Serial.printf("  X: %.2f  Y: %.2f  Z: %.2f\n", event.magnetic.x, event.magnetic.y, event.magnetic.z);
  } else Serial.println("ERROR");

  // --- SD Card ---
  Serial.print("SD Card: ");
  if (SD.begin(SD_CS)) {
    Serial.println("OK");
    File f = SD.open("/test.txt", FILE_WRITE);
    if (f) {
      f.println("PogodaAI test log");
      f.close();
      Serial.println("  Write test: OK");
    } else Serial.println("  Write test: ERROR");
  } else Serial.println("ERROR");

  // --- Rain Sensor ---
  Serial.print("Rain Sensor: ");
  int rain = analogRead(RAIN_PIN);
  Serial.printf("OK  (Value: %d)\n", rain);

  // --- Light Sensor ---
  Serial.print("Light Sensor: ");
  int light = analogRead(LIGHT_PIN);
  Serial.printf("OK  (Value: %d)\n", light);

  // --- Anemometer ---
  Serial.print("Anemometer: ");
  pinMode(ANEMOMETER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, RISING);
  Serial.println("OK (waiting for pulses...)");

  Serial.println("\n=== Initialization complete ===");
  Serial.println("Rotate anemometer or blow on sensor to test pulses.");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    Serial.printf("[Test] Rain=%d  Light=%d  Pulses=%u\n",
                  analogRead(RAIN_PIN),
                  analogRead(LIGHT_PIN),
                  anemometerCount);
  }
}

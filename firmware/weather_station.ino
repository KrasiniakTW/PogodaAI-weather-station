/********************************************************************
  Stacja pogodowa ESP32 - główny kod
  Arduino IDE
  Funkcje:
  - BME680 (Temp, Wilgotność, Ciśnienie, Gas)
  - PMSA003 (PM1, PM2.5, PM10)
  - DS3231 RTC
  - SD logging
  - Anemometr
  - Czujniki deszczu i światła
  - 4x1 klawiatura (pomijamy przycisk 3)
  - I2C LCD 20x4
  - Menu + Wi-Fi hotspot dla pobrania plików
  - Deep sleep z wake on button
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
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiAP.h>
#include <Adafruit_INA219.h>

/************************** PINY ******************************/
#define KEY1 2
#define KEY2 15
#define KEY4 4

#define RAIN_PIN 33
#define LIGHT_PIN 32
#define ANEMOMETER_PIN 27

#define ANEMOMETER_RADIUS_M 0.08
#define WIND_CALIBRATION 1.0

#define SD_CS 5

#define I2C_SDA 21
#define I2C_SCL 22

#define PMS_RX 17
#define PMS_TX 16

/************************** STAŁE / ZMIENNE ******************************/
const uint32_t SERIAL_SPEED = 115200;
const uint32_t MEASURE_INTERVAL_SEC = 300;  // interwał deep sleep w sekundach
const uint32_t MENU_ACTIVE_SEC = 30;        // ile sekund stacja pozostaje aktywna po wybudzeniu
const uint8_t ANEMOMETER_HOLES = 12;        // liczba otworów w kole anemometru

RTC_DS3231 rtc;
BME680_Class BME680;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_INA219 ina219;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

HardwareSerial pmSerial(2);
Adafruit_PM25AQI aqi;

File logFile;

// zmienne do anemometru
volatile uint16_t anemometerCount = 0;

//zmienne do czujnika deszczu
int lastRainValue = 0;

// Wi-Fi hotspot
const char* AP_SSID = "StacjaPogodowa_ESP32";
const char* AP_PASSWORD = "12345678";
WebServer server(80);

// zmienne do menu
bool menuActive = false;
unsigned long menuStartMillis = 0;

// zmienna do karty SD
bool sdActive = true;

void IRAM_ATTR anemometerISR() {
  anemometerCount++;
}

int batteryPercentage(float voltage) {
  // zakres przybliżony: 4.20V (100%) → 3.20V (0%)
  if (voltage >= 4.20) return 100;
  if (voltage <= 3.20) return 0;
  return (int)((voltage - 3.20) * 100.0 / (4.20 - 3.20));
}

void openLogFile() {
  DateTime now = rtc.now();
  char filename[20];
  sprintf(filename, "/log_%04d%02d%02d.csv", now.year(), now.month(), now.day());
  
  if (SD.exists(filename)) {
    logFile = SD.open(filename, FILE_APPEND);
  } else {
    logFile = SD.open(filename, FILE_WRITE);
    logFile.println("Data,Czas,Temp,Hum,Press,Gas,PM1,PM2_5,PM10,Rain,Light,Wind_RPM,Wind_Speed");
  }

  if (!logFile) {
    Serial.println("Nie mozna otworzyc pliku log!");
    lcd.print("SD open fail!");
  }
}

void readBME680(float &temp, float &hum, float &press, float &gas) {
  int32_t t, h, p, g;
  BME680.getSensorData(t, h, p, g);
  temp = t / 100.0;
  hum  = h / 1000.0;
  press = p / 100.0;
  gas = g / 100.0;
}

bool readPMSA003(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10) {
  PM25_AQI_Data data;
  if (!aqi.read(&data)) return false;
  pm1 = data.pm10_standard;
  pm25 = data.pm25_standard;
  pm10 = data.pm100_standard;
  return true;
}

String headingToDirection(float headingDegrees) {
  String dirs[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                   "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int index = (int)((headingDegrees + 11.25) / 22.5);
  index = index % 16;
  return dirs[index];
}

float readRain() {
  int val = analogRead(RAIN_PIN);
  return (float)val; // opcjonalnie przeliczyć na mm
}

String getRainState(int rainValue, int maxValue = 4095) {
  rainValue = maxValue - rainValue;
  if (rainValue >= maxValue) {
    return "Full";  // zalany czujnik
  } else if (rainValue > lastRainValue) {
    return "Pada";
  } else {
    return "Nie pada";
  }
}

float readLight() {
  int val = analogRead(LIGHT_PIN);
  return (float)val; // opcjonalnie przeliczyć na lux
}

String getLightState(int lightValue, int threshold = 2000) {
  if (lightValue > threshold) {
    return "Dzien";
  } else {
    return "Noc";
  }
}

float calculateWindSpeed(float measurementSeconds, uint16_t &outRevs) {
  noInterrupts();
  uint16_t revs = anemometerCount;
  anemometerCount = 0;
  interrupts();

  outRevs = revs;
  if (measurementSeconds <= 0 || revs == 0) return 0.0;

  float revsPerSec = (float)revs / measurementSeconds;             // impulsy/s
  float rotationsPerSec = revsPerSec / (float)ANEMOMETER_HOLES;    // obr/s
  float circumference = 2.0 * PI * ANEMOMETER_RADIUS_M;            // m
  float windSpeed = rotationsPerSec * circumference * WIND_CALIBRATION; // m/s

  return windSpeed;
}

void logData() {
  if (!logFile) return;

  DateTime now = rtc.now();
  float temp, hum, press, gas, wind;
  uint16_t pm1, pm25, pm10, revs;
  String light, rain;

  readBME680(temp, hum, press, gas);
  readPMSA003(pm1, pm25, pm10);
  rain = getRainState(readRain());
  light = getLightState(readLight());
  lastRainValue = readRain();
  revs = 0;
  wind = calculateWindSpeed((float)MEASURE_INTERVAL_SEC, revs);
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22; // do korekty później
  heading += declinationAngle;
  if (heading < 0) heading += 2*PI;
  if (heading > 2*PI) heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI;
  String windDir = headingToDirection(headingDegrees);

  char line[200];
  sprintf(line, "%04d-%02d-%02d,%02d:%02d:%02d,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%s,%s,%lu,%s",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second(),
    temp, hum, press, gas,
    pm1, pm25, pm10,
    rain.c_str(), light.c_str(),
    (unsigned long)anemometerCount, windDir.c_str()
  );


  logFile.println(line);
  logFile.flush(); // zapis na SD
}

void checkButtons() {
  if (digitalRead(KEY1) || digitalRead(KEY2) || digitalRead(KEY4)) {
    menuActive = true;
    menuStartMillis = millis();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Menu:");
    lcd.setCursor(0,1);
    lcd.print("1-Status 2-SD 4-WiFi");
    delay(300); // debounce
  }
}

void handleMenu() {
  if (!menuActive) return;

  float busvoltage = ina219.getBusVoltage_V();
  float shuntvoltage = ina219.getShuntVoltage_mV() / 1000.0;
  float loadvoltage = busvoltage + shuntvoltage;
  int battPercent = batteryPercentage(loadvoltage);

  if (millis() - menuStartMillis > MENU_ACTIVE_SEC * 1000) {
    menuActive = false;
    lcd.clear();
    lcd.noBacklight();
    return;
  }

  if (digitalRead(KEY1)) { // status
    lcd.clear();
    float temp, hum, press, gas;
    uint16_t pm1, pm25, pm10;
    readBME680(temp, hum, press, gas);
    readPMSA003(pm1, pm25, pm10);
    lcd.setCursor(0,0);
    lcd.printf("T:%.1f H:%.1f", temp, hum);
    lcd.setCursor(0,1);
    lcd.printf("P:%.0f G:%.0f", press, gas);
    lcd.setCursor(0,2);
    lcd.printf("PM2.5:%d PM10:%d", pm25, pm10);
    lcd.setCursor(0,3);
    lcd.printf("Bat:%d", battPercent);
    delay(1000);
  }
  else if (digitalRead(KEY2)) { // safe SD remove / reinit
    lcd.clear();
    if (sdActive) {
      // Odmontowanie karty
      lcd.print("SD off");
      if (logFile) logFile.close();
      SD.end();
      sdActive = false;
    } else {
      // Ponowne zamontowanie karty
      lcd.print("SD init...");
      if (SD.begin(SD_CS)) {
        openLogFile();
        sdActive = true;
        lcd.setCursor(0,1);
        lcd.print("SD ready");
      } else {
        lcd.setCursor(0,1);
        lcd.print("SD fail");
      }
    }
    menuStartMillis = millis();   // reset timera menu
    delay(2000);
  }
  else if (digitalRead(KEY4)) { // Wi-Fi hotspot
    lcd.clear();
    lcd.print("WiFi AP:");
    lcd.setCursor(0,1);
    lcd.print(AP_SSID);
    lcd.setCursor(0,2);
    lcd.print("IP:");
    lcd.print(WiFi.softAPIP());
    delay(2000);
  }
}

void handleFileDownload() {
  DateTime now = rtc.now();
  char filename[20];
  sprintf(filename, "/log_%04d%02d%02d.csv", now.year(), now.month(), now.day());

  if (!SD.exists(filename)) {
    server.send(200, "text/plain", "Brak pliku log");
    return;
  }

  File file = SD.open(filename, FILE_READ);
  if (!file) {
    server.send(200, "text/plain", "Blad otwarcia pliku");
    return;
  }

  server.streamFile(file, "text/csv");
  file.close();
}

void setup() {
  Serial.begin(SERIAL_SPEED);
  delay(100);

  /********** Piny klawiatury **********/
  pinMode(KEY1, INPUT_PULLDOWN);
  pinMode(KEY2, INPUT_PULLDOWN);
  pinMode(KEY4, INPUT_PULLDOWN);

  /********** I2C **********/
  Wire.begin(I2C_SDA, I2C_SCL);

  /********** LCD **********/
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Stacja pogodowa");
  delay(1000);
  lcd.clear();

  /********** RTC **********/
  if (!rtc.begin()) {
    lcd.print("Nie wykryto DS3231!");
    Serial.println("Nie wykryto DS3231!");
    while(1) delay(10);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  /********** BME680 **********/
  while (!BME680.begin(I2C_STANDARD_MODE)) {
    Serial.println("BME680 nie znaleziony, retry za 5s...");
    delay(5000);
  }
  BME680.setOversampling(TemperatureSensor, Oversample16);
  BME680.setOversampling(HumiditySensor, Oversample16);
  BME680.setOversampling(PressureSensor, Oversample16);
  BME680.setIIRFilter(IIR4);
  BME680.setGas(320, 150);

  /********** PMSA003 **********/
  pmSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);
  delay(3000);
  if (!aqi.begin_UART(&pmSerial)) {
    Serial.println("PMSA003 nie znaleziony!");
    lcd.print("PMSA003 blad!");
    while(1) delay(10);
  }

  /********** INA219 **********/
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  /********** HMC5883 **********/
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  /********** SD **********/
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init fail!");
    lcd.print("SD init fail!");
    while(1) delay(10);
  }
  openLogFile(); // otwiera plik dzienny

  /********** Anemometr **********/
  pinMode(ANEMOMETER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, RISING);

  /********** Wi-Fi hotspot **********/
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  server.on("/", HTTP_GET, handleFileDownload);
  server.begin();

  /********** Powitalny komunikat **********/
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Stacja gotowa!");
  delay(1000);
  lcd.clear();
}

void loop() {
  checkButtons();   // wybudzenie / menu
  handleMenu();     // obsługa menu

  // Jeśli nie w menu, wykonujemy pomiar i logowanie
  if (!menuActive) {
    logData();
    // przygotowanie do deep sleep
    lcd.clear();
    lcd.noBacklight();
    logFile.close(); // zamknięcie pliku przed deep sleep
    esp_sleep_enable_timer_wakeup(MEASURE_INTERVAL_SEC * 1000000ULL);

    // wake on button
    esp_sleep_enable_ext1_wakeup(
      (1ULL << KEY1) | (1ULL << KEY2) | (1ULL << KEY4),
      ESP_EXT1_WAKEUP_ANY_HIGH
    );

    Serial.println("Wchodzę w deep sleep...");
    delay(100);
    esp_deep_sleep_start();
  }

  server.handleClient(); // obsługa HTTP servera dla hotspot
}

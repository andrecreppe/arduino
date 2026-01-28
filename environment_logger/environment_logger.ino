/*
===============================================================================
  Project: Environment Logger
  Function: Reads temperature and humidity every minute, logs to CSV (env_log.csv),
            displays live readings, total points logged, and error statuses.

  --------------------------
  Connections (Arduino Uno/Nano)
  --------------------------
  DHT22 Sensor:
    - VCC  -> 5V
    - GND  -> GND
    - DATA -> D2
    - (If your DHT22 is a bare sensor, add a 10k pull-up resistor from DATA to 5V)

  microSD Module (SPI):
    - VCC  -> 5V  (ensure the module has level shifting for 5V boards)
    - GND  -> GND
    - CS   -> D10
    - MOSI -> D11
    - MISO -> D12
    - SCK  -> D13

  128x64 I2C OLED (SSD1306, typical addr 0x3C):
    - VCC -> 5V (many OLED breakouts accept 3.3–5V; check your module)
    - GND -> GND
    - SDA -> A4 (I2C SDA on Uno/Nano)
    - SCL -> A5 (I2C SCL on Uno/Nano)

  Notes:
    - If using a different board, use its dedicated SPI pins for SD and SDA/SCL pins for I2C.
    - Format the microSD as FAT16/FAT32.
===============================================================================
*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

// ------------------------ Configuration ------------------------
#define DHTPIN 2
#define DHTTYPE DHT22

const int SD_CS_PIN = 10;
const unsigned long LOG_INTERVAL_MS = 60000UL; // 1 minute

// OLED configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C

// File name (CSV only)
const char* CSV_FILENAME = "env_log.csv";

// ------------------------ Globals ------------------------------
DHT dht(DHTPIN, DHTTYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool sdOK = false;
bool lastDHTOK = false;

unsigned long lastLog = 0;
unsigned long minuteCounter = 0;   // relative timing index, starts at 0, first log -> 1
unsigned long totalPoints = 0;     // number of successfully logged records

float lastTempC = NAN;
float lastHum = NAN;

// ------------------------ Helpers ------------------------------
void splash(const __FlashStringHelper* line1, const __FlashStringHelper* line2 = F(""), const __FlashStringHelper* line3 = F("")) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.println(line2);
  display.println(line3);
  display.display();
}

void renderDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Row 0-7
  display.setCursor(0, 0);
  display.print(F("T: "));
  if (isnan(lastTempC)) display.print(F("--"));
  else {
    display.print(lastTempC, 2);
    display.print(F(" C"));
  }

  // Row 8-15
  display.setCursor(0, 10);
  display.print(F("H: "));
  if (isnan(lastHum)) display.print(F("--"));
  else {
    display.print(lastHum, 2);
    display.print(F(" %"));
  }

  // Row 16-23
  display.setCursor(0, 20);
  display.print(F("Pts: "));
  display.print(totalPoints);

  // Row 26-33
  display.setCursor(0, 30);
  display.print(F("SD: "));
  display.println(sdOK ? F("OK") : F("ERROR"));

  // Row 36-43
  display.setCursor(0, 40);
  display.print(F("DHT: "));
  display.println(lastDHTOK ? F("OK") : F("ERROR"));

  // Row 46-53
  display.setCursor(0, 50);
  display.print(F("Next @ +"));
  // Show the minute counter + 1 as "next minute index"
  display.print(minuteCounter + 1);

  display.display();
}

bool openForAppend(const char* fname, File &f) {
  f = SD.open(fname, FILE_WRITE);
  return (bool)f;
}

void writeHeaderIfNew(const char* fname, const char* header) {
  if (!SD.exists(fname)) {
    File f = SD.open(fname, FILE_WRITE);
    if (f) {
      f.println(header);
      f.close();
    }
    return;
  }
  File f = SD.open(fname, FILE_WRITE);
  if (!f) return;
  if (f.size() == 0) {
    f.println(header);
  }
  f.close();
}

void tryInitSD() {
  sdOK = SD.begin(SD_CS_PIN);
  if (sdOK) {
    writeHeaderIfNew(CSV_FILENAME, "minute,temperature_c,humidity_pct");
  }
}

bool logCSV(unsigned long minuteIdx, float tC, float h) {
  if (!sdOK) return false;

  File f;
  if (!openForAppend(CSV_FILENAME, f)) {
    sdOK = false; // Mark SD as failed after this attempt
    return false;
  }

  // minute,temperature_c,humidity_pct
  f.print(minuteIdx);
  f.print(',');
  f.print(tC, 2);
  f.print(',');
  f.println(h, 2);
  f.close();
  return true;
}

// ------------------------ Setup & Loop -------------------------
void setup() {
  Serial.begin(9600);
  delay(200);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    // If OLED fails to initialize, we still proceed; just no display output
  } else {
    splash(F("Env Logger"), F("DHT22 + SD (CSV)"), F("Initializing..."));
  }

  dht.begin();
  tryInitSD();

  if (!sdOK) {
    Serial.println(F("[ERROR] SD init failed. Check wiring/power/CS pin and SD format."));
    splash(F("Env Logger"), F("SD: ERROR"), F("Check wiring/format"));
  } else {
    Serial.println(F("[OK] SD ready, logging to env_log.csv"));
  }

  lastLog = millis(); // start cadence
  renderDisplay();
}

void loop() {
  // Check if it's time to read/log (every 60s)
  if (millis() - lastLog >= LOG_INTERVAL_MS) {
    lastLog += LOG_INTERVAL_MS; // maintain steady cadence

    // Read sensor
    float h = dht.readHumidity();
    float t = dht.readTemperature(); // Celsius
    if (isnan(h) || isnan(t)) {
      lastDHTOK = false;
      Serial.println(F("[WARN] DHT read failed."));
      // Keep last good values on display but show error
      renderDisplay();
      return;
    }

    lastDHTOK = true;
    lastTempC = t;
    lastHum   = h;

    // Update relative minute counter (1-based in the file)
    minuteCounter++;
    bool writeOK = logCSV(minuteCounter, t, h);

    if (writeOK) {
      totalPoints++;
      Serial.print(F("[OK] Logged minute "));
      Serial.print(minuteCounter);
      Serial.print(F(": "));
      Serial.print(t, 2);
      Serial.print(F(" C, "));
      Serial.print(h, 2);
      Serial.println(F(" %"));
    } else {
      Serial.println(F("[ERROR] Failed to write to CSV. SD marked as ERROR."));
    }

    renderDisplay();
  }

  // You can do other non-blocking tasks here
}

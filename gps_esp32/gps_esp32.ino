// ESP32 + SSD1306 + TinyGPS++
// Robust detection of GPS loss with startup fix: start in search mode until real data arrives.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// --- OLED setup ---
// SDA = D21 -- SCL = D22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- GPS setup (ESP32 hardware serial) ---
#define GPS_RX 16   // ESP32 pin connected to GPS TX
#define GPS_TX 17   // ESP32 pin connected to GPS RX
HardwareSerial GPSserial(1);  // UART1
TinyGPSPlus gps;

// --- Globe animation vars ---
float angle = 0.0;
const float radius = 20.0;
const int centerX = SCREEN_WIDTH / 2;
const int centerY = (SCREEN_HEIGHT + 16) / 2; // push globe lower so top 16px yellow band is free

// --- Timeouts (milliseconds) ---
const unsigned long FIX_TIMEOUT   = 10UL * 1000UL; // if no new valid fix for 10s => lost
const unsigned long DATA_TIMEOUT  = 5UL * 1000UL;  // if no bytes received for 5s => lost

// --- State timestamps ---
unsigned long lastCharMillis     = 0; // when we last read a byte from GPS UART
unsigned long lastValidFixMillis = 0; // when we last parsed a new valid fix (location+time+date)
unsigned long searchStart        = 0;

// --- State flags ---
bool searching   = true;
bool everHadFix  = false; // prevent showing "0" fix on boot

void setup() {
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // OLED not found - halt
    for (;;);
  }
  display.clearDisplay();
  searchStart = millis();
}

void loop() {
  // --- Read bytes from GPS and feed TinyGPS++
  while (GPSserial.available() > 0) {
    char c = GPSserial.read();
    gps.encode(c);
    lastCharMillis = millis();
  }

  // --- Detect when TinyGPS++ parsed a NEW valid fix
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    if (gps.location.isUpdated() || gps.time.isUpdated() || gps.satellites.isUpdated()) {
      lastValidFixMillis = millis();
      everHadFix = true;   // only becomes true after first real update
    }
  }

  // --- Decide whether we have a fresh fix
  bool gotRecentChars = (millis() - lastCharMillis) <= DATA_TIMEOUT;
  bool gotRecentFix   = (millis() - lastValidFixMillis) <= FIX_TIMEOUT;
  bool hasFreshFix    = everHadFix && gotRecentChars && gotRecentFix;

  // Transition handling
  if (!hasFreshFix && !searching) {
    // just lost fix
    searching = true;
    searchStart = millis();
  } else if (hasFreshFix && searching) {
    // just acquired fix
    searching = false;
  }

  // --- Display
  display.clearDisplay();

  if (hasFreshFix) {
    // --- GPS FIX FOUND ---
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Yellow zone (top band)
    // Date
    display.setCursor(0, 0);
    display.printf("%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year());
    // Satellites count
    display.setCursor(85, 0);
    display.printf("Sat: %02d", gps.satellites.value());
    // Time
    display.setCursor(0, 8);
    display.printf("%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    display.print(" (UTC)");

    // Blue zone
    // Coords
    display.setCursor(0, 20);
    display.print("Lat: "); display.println(gps.location.lat(), 6);
    display.setCursor(0, 30);
    display.print("Lng: "); display.println(gps.location.lng(), 6);
    // Alt
    display.setCursor(0, 40);
    display.print("Alt: "); display.print(gps.altitude.meters()); display.println(" m");
    // Speed
    display.setCursor(0, 50);
    display.print("Spd: "); display.print(gps.speed.kmph()); display.println(" km/h");

  } else {
    // --- WAITING FOR FIX ---
    unsigned long elapsed = (millis() - searchStart) / 1000; // seconds
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Searching GPS...");

    display.setCursor(0, 8);
    display.print("Elapsed: ");
    display.print(elapsed);
    display.print("s");

    // Globe animation in lower part
    drawGlobe();
    angle += 5;
    if (angle >= 360) angle = 0;
  }

  display.display();
  delay(100);
}

// --- Functions for globe wireframe ---
void drawLatitude(int lat, float rotAngle) {
  for (int lon = 0; lon < 360; lon += 10) {
    float x1, y1, x2, y2;
    sphereToScreen(lat, lon, rotAngle, x1, y1);
    sphereToScreen(lat, lon + 10, rotAngle, x2, y2);
    display.drawLine((int)x1, (int)y1, (int)x2, (int)y2, SSD1306_WHITE);
  }
}

void drawLongitude(int lon, float rotAngle) {
  for (int lat = -90; lat < 90; lat += 10) {
    float x1, y1, x2, y2;
    sphereToScreen(lat, lon, rotAngle, x1, y1);
    sphereToScreen(lat + 10, lon, rotAngle, x2, y2);
    display.drawLine((int)x1, (int)y1, (int)x2, (int)y2, SSD1306_WHITE);
  }
}

void drawGlobe() {
  for (int lat = -60; lat <= 60; lat += 30) {
    drawLatitude(lat, angle);
  }
  for (int lon = 0; lon < 360; lon += 30) {
    drawLongitude(lon, angle);
  }
}

void sphereToScreen(float lat, float lon, float rotAngle, float &x, float &y) {
  float latRad = radians(lat);
  float lonRad = radians(lon + rotAngle);

  float X = radius * cos(latRad) * cos(lonRad);
  float Y = radius * sin(latRad);

  x = centerX + X;
  y = centerY + Y;
}

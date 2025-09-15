#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// --- OLED setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- GPS setup ---
static const int RXPin = 4, TXPin = 3;   // Change if needed
static const uint32_t GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

// --- Globe animation vars ---
float angle = 0.0;
const float radius = 20.0;
const int centerX = SCREEN_WIDTH / 2;
const int centerY = (SCREEN_HEIGHT + 16) / 2; // Push globe lower to keep yellow bar clear

// --- Timer for searching ---
unsigned long searchStart = 0;
bool searching = true;

// --- Setup ---
void setup() {
  gpsSerial.begin(GPSBaud);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }
  display.clearDisplay();
  searchStart = millis(); // mark time when search starts
}

// --- Main loop ---
void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  display.clearDisplay();

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    // --- GPS FIX FOUND ---
    searching = false;

    // Yellow band (top 16 px)
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(gps.date.month()); display.print('/');
    display.print(gps.date.day());   display.print('/');
    display.print(gps.date.year());
    display.setCursor(0, 8);
    display.print(gps.time.hour()); display.print(':');
    display.print(gps.time.minute()); display.print(':');
    display.print(gps.time.second());
    display.setCursor(90, 0);
    display.print("Sat:");
    display.print(gps.satellites.value());

    // Blue/white area (below 16 px)
    display.setCursor(0, 20);
    display.print("Lat: "); display.println(gps.location.lat(), 6);
    display.setCursor(0, 30);
    display.print("Lng: "); display.println(gps.location.lng(), 6);
    display.setCursor(0, 40);
    display.print("Alt: "); display.print(gps.altitude.meters()); display.println(" m");
    display.setCursor(0, 50);
    display.print("Spd: "); display.print(gps.speed.kmph()); display.println(" km/h");

  } else {
    // --- WAITING FOR FIX ---
    searching = true;

    // Yellow band text with elapsed time
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
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }
}

void drawLongitude(int lon, float rotAngle) {
  for (int lat = -90; lat < 90; lat += 10) {
    float x1, y1, x2, y2;
    sphereToScreen(lat, lon, rotAngle, x1, y1);
    sphereToScreen(lat + 10, lon, rotAngle, x2, y2);
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
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
  float Z = radius * cos(latRad) * sin(lonRad);

  x = centerX + X;
  y = centerY + Y;
}

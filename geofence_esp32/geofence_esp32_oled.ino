#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// -------------------- OLED Display setup --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------------------- GPS setup --------------------
TinyGPSPlus gps;
HardwareSerial GPSserial(1); // use UART1 on ESP32
#define RXD2 16
#define TXD2 17

// -------------------- LEDs & Buzzer --------------------
#define LED_GREEN 25
#define LED_YELLOW 26
#define LED_RED 27
#define BUZZER 14

// -------------------- Geofence rectangle --------------------

// Define four corners of the runway exclusion zone (lat, lon in degrees)
struct Point {
  double lat;
  double lon;
};

// Point runway[4] = {
//   // Embraer GPX <> Runway 02-20
//   { -21.751308, -48.405645 }, // NW
//   { -21.751315, -48.405206 }, // NE
//   { -21.796440, -48.404396 }, // SE
//   { -21.796451, -48.404831 }, // SW
// };
Point runway[4] = {
  // Embraer GPX <> Alpha Yard
  { -21.762005, -48.403078 }, // NW
  { -21.761996, -48.402081 }, // NE
  { -21.763241, -48.402057 }, // SE
  { -21.763264, -48.403050 }, // SW
};

// #define WARNING_RADIUS 150
#define WARNING_RADIUS 35


// -------------------- Timers --------------------
unsigned long lastValidFixMillis = 0;
unsigned long lastCharReceived = 0;
bool everHadFix = false;
const unsigned long GPS_FIX_TIMEOUT = 10000; // ms


// -------------------- Helpers: distance --------------------
// Compute min distance from current point to rectangle edges
// Approximate conversion of lat/lon to local XY (meters) relative to a reference point
void latLonToXY(double lat, double lon, double lat0, double lon0, double &x, double &y) {
  const double R = 6371000.0; // Earth radius in meters
  double dLat = radians(lat - lat0);
  double dLon = radians(lon - lon0);
  double meanLat = radians((lat + lat0) / 2.0);
  x = dLon * cos(meanLat) * R;
  y = dLat * R;
}

// Distance from point P to segment AB in XY plane
double pointToSegmentDist(double px, double py, double ax, double ay, double bx, double by) {
  double dx = bx - ax;
  double dy = by - ay;
  if (dx == 0 && dy == 0) {
    // A and B are the same point
    dx = px - ax;
    dy = py - ay;
    return sqrt(dx*dx + dy*dy);
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / (dx*dx + dy*dy);
  if (t < 0) {
    dx = px - ax;
    dy = py - ay;
  } else if (t > 1) {
    dx = px - bx;
    dy = py - by;
  } else {
    double projx = ax + t * dx;
    double projy = ay + t * dy;
    dx = px - projx;
    dy = py - projy;
  }
  return sqrt(dx*dx + dy*dy);
}

double distanceToRectangle(double lat, double lon) {
  // Reference point: first corner
  double lat0 = runway[0].lat;
  double lon0 = runway[0].lon;

  // Convert test point
  double px, py;
  latLonToXY(lat, lon, lat0, lon0, px, py);

  // Convert polygon
  double polyX[4], polyY[4];
  for (int i = 0; i < 4; i++) {
    latLonToXY(runway[i].lat, runway[i].lon, lat0, lon0, polyX[i], polyY[i]);
  }

  // Check if inside polygon (simple ray casting in XY)
  bool inside = false;
  for (int i = 0, j = 3; i < 4; j = i++) {
    if (((polyY[i] > py) != (polyY[j] > py)) &&
        (px < (polyX[j] - polyX[i]) * (py - polyY[i]) / (polyY[j] - polyY[i]) + polyX[i]))
      inside = !inside;
  }
  if (inside) return 0.0;

  // Compute min distance to edges
  double minDist = 1e9;
  for (int i = 0; i < 4; i++) {
    int j = (i+1) % 4;
    double d = pointToSegmentDist(px, py, polyX[i], polyY[i], polyX[j], polyY[j]);
    if (d < minDist) minDist = d;
  }
  return minDist;
}

// -------------------- Helpers: globe animation --------------------
// Globe animation vars
float angle = 0.0;
const float radius = 20.0;
const int centerX = SCREEN_WIDTH / 2;
const int centerY = (SCREEN_HEIGHT + 16) / 2; // push globe lower so top 16px yellow band is free

// Functions for globe wireframe
void drawLatitude(int lat, float rotAngle) {
  for (int lon = 0; lon < 360; lon += 10) {
    float x1, y1, x2, y2;
    sphereToScreen(lat, lon, rotAngle, x1, y1);
    sphereToScreen(lat, lon + 10, rotAngle, x2, y2);
    oled.drawLine((int)x1, (int)y1, (int)x2, (int)y2, SSD1306_WHITE);
  }
}

void drawLongitude(int lon, float rotAngle) {
  for (int lat = -90; lat < 90; lat += 10) {
    float x1, y1, x2, y2;
    sphereToScreen(lat, lon, rotAngle, x1, y1);
    sphereToScreen(lat + 10, lon, rotAngle, x2, y2);
    oled.drawLine((int)x1, (int)y1, (int)x2, (int)y2, SSD1306_WHITE);
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


// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // GPS
  GPSserial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Oled Display
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 OLED allocation failed");
    for (;;);
  }
  oled.clearDisplay();
  oled.display();

  // LEDs and buzzer
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER, LOW);
}

// -------------------- Loop --------------------
void loop() {
  while (GPSserial.available() > 0) {
    char c = GPSserial.read();
    if (gps.encode(c)) {
      if (gps.location.isValid() && gps.time.isValid()) {
        lastValidFixMillis = millis();
        everHadFix = true;
      }
    }
    lastCharReceived = millis();
  }

  bool gotRecentChars = (millis() - lastCharReceived < 2000);
  bool gotRecentFix = (millis() - lastValidFixMillis < GPS_FIX_TIMEOUT);
  bool hasFreshFix = everHadFix && gotRecentChars && gotRecentFix;

  oled.clearDisplay();

  if (hasFreshFix) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    double alt = gps.altitude.meters();
    double spd = gps.speed.kmph();
    int sats  = gps.satellites.value();

    // Calculate distance to rectangle
    double dist = distanceToRectangle(lat, lon);

    // Display info
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);

    // Yellow zone (top band)
    // Date
    oled.setCursor(0, 0);
    oled.printf("%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year());
    // Satellites count
    oled.setCursor(85, 0);
    oled.printf("Sat: %02d", gps.satellites.value());
    // Time
    oled.setCursor(0, 8);
    oled.printf("%02d:%02d:%02d (UTC)", gps.time.hour(), gps.time.minute(), gps.time.second());

    // Blue zone
    // Coords
    oled.setCursor(0, 20);
    oled.print("Lat: "); oled.println(gps.location.lat(), 6);
    oled.setCursor(0, 30);
    oled.print("Lng: "); oled.println(gps.location.lng(), 6);
    // Speed
    oled.setCursor(0, 40);
    oled.print("Spd: "); oled.print(gps.speed.kmph()); oled.println(" km/h");
    // Distance info
    oled.setCursor(0, 55);
    oled.printf(">> DIST: %.1f m", dist);

    // LED + Buzzer logic
    if (dist == 0) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(BUZZER, HIGH); // constant
    } else if (dist <= WARNING_RADIUS) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, HIGH);
      // Beeping buzzer
      if ((millis() / 500) % 2 == 0) {
        digitalWrite(BUZZER, HIGH);
      } else {
        digitalWrite(BUZZER, LOW);
      }
    } else {
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(BUZZER, LOW);
    }

  } else {
    // Searching for satellites
    unsigned long elapsed = millis() / 1000;

    // Display info
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    // Yellow zone (top band)
    oled.setCursor(0, 0);
    oled.print("Searching sats...");
    oled.setCursor(0, 8);
    oled.printf("Elapsed: %ds", elapsed);

    // Reset outputs
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER, LOW);

    // Globe animation in lower part
    drawGlobe();
    angle += 5;
    if (angle >= 360) angle = 0;
  }

  oled.display();
  delay(100);
}

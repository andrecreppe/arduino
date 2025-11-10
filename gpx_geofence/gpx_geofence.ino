#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#include "geofence_functions.h"


// -------------------- LCD Display setup --------------------
#define LCD_COLUMNS 20
#define LDC_ROWS 4
#define LCD_ADDRESS 0x27
// SDA = 21 ; SCL = 22
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LDC_ROWS);


// -------------------- GPS setup --------------------
TinyGPSPlus gps;
HardwareSerial GPSserial(1); // use UART1 on ESP32
#define RXD2 16
#define TXD2 17


// -------------------- LEDs, Buzzer and BUttons --------------------
#define LED_GREEN 25
#define LED_YELLOW 26
#define LED_RED 27
#define BUZZER 14
#define BTN_MUTE 34


// -------------------- Geofence rectangle --------------------
// Point runway[4] = {
//   // Embraer GPX <> Runway 02-20
//   { -21.750669, -48.406295 }, // NW
//   { -21.750669, -48.404549 }, // NE
//   { -21.796963, -48.403810 }, // SE
//   { -21.796989, -48.405407 }, // SW
// };

// #define WARNING_RADIUS 90

Point runway[4] = {
  // Embraer GPX <> Alpha Yard
  { -21.762005, -48.403078 }, // NW
  { -21.761996, -48.402081 }, // NE
  { -21.763241, -48.402057 }, // SE
  { -21.763264, -48.403050 }, // SW
};

#define WARNING_RADIUS 35


// -------------------- Variables: Timers --------------------
unsigned long lastValidFixMillis = 0;
unsigned long lastCharReceived = 0;
bool everHadFix = false;
const unsigned long GPS_FIX_TIMEOUT = 10000; // ms


// -------------------- Variables: Mute logic --------------------
bool isMuted = false;


// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // GPS
  GPSserial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // LCD Display
  lcd.init();
  lcd.backlight();

  // LEDs, Buzzer and Buttons
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_MUTE, INPUT);

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

  if (digitalRead(BTN_MUTE) == HIGH) {
    isMuted = true;
  }

  if (hasFreshFix) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    double alt = gps.altitude.meters();
    double spd = gps.speed.kmph();
    int sats  = gps.satellites.value();

    // Calculate distance to rectangle
    double dist = distanceToRectangle(lat, lon);

    // Display Text
    lcd.setCursor(0,1);
    lcd.print("DIST: ");
    lcd.print(dist, 1);
    lcd.print(" m      ");

    lcd.setCursor(0,0);
    lcd.print("GPS OK - ");
    
    // LED + Buzzer logic
    if (dist <= 0) {
      lcd.print("PISTA! ");
      // Green Light
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED, HIGH);
      // Buzzer
      if (!isMuted) {
        digitalWrite(BUZZER, ((millis() / 250) % 2 == 0) ? HIGH : LOW); // 4 Hz beep
      } else {
        digitalWrite(BUZZER, LOW);
      }
    } else if (dist <= WARNING_RADIUS) {
      lcd.print("ATENCAO!");
      // Yellow Light
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, HIGH);
      // Buzzer
      if (!isMuted) {
        digitalWrite(BUZZER, ((millis() / 500) % 2 == 0) ? HIGH : LOW); // 2 Hz beep
      } else {
        digitalWrite(BUZZER, LOW);
      }
    } else {
      lcd.print("LIVRE  ");
      // Red Light
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED, LOW);
      // No buzzer
      digitalWrite(BUZZER, LOW);
      isMuted = false; // Reset mute when free
    }

  } else {
    lcd.setCursor(0,0);
    lcd.print("<!< GPX RIAD >!>");

    // Searching for satellites
    int elapsed = (millis() / 1000) % 4;

    // Display info
    lcd.setCursor(0,1);
    if (elapsed == 0) lcd.print("STARTING GPS   ");
    else if (elapsed == 1) lcd.print("STARTING GPS.  ");
    else if (elapsed == 2) lcd.print("STARTING GPS.. ");
    else if (elapsed == 3) lcd.print("STARTING GPS...");

    // Reset outputs
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(BUZZER, LOW);
  }

  delay(100);
}
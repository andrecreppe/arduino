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

byte blockChar[] = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111};

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
#define POT 13


// -------------------- Geofence rectangle --------------------
#define WARNING_RADIUS 90


// -------------------- Variables: Mute logic --------------------
bool isMuted = false;


// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  // LCD Display
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, blockChar);

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

  // -----------------------------------

  for(int i=0; i<4; i++) {
    for(int j=0; j<20; j++) {
      lcd.setCursor(j,i);
      lcd.write(0);
    }
  }

  // Searching screen demo
  int introElapsed = 0;

  lcd.setCursor(0,0);
  lcd.print("<!< GPX RIAD >!>");

  while(introElapsed < 5) {
    // Searching for satellites
    int dots = introElapsed % 4;

    // Display info
    lcd.setCursor(0,1);
    if (dots == 0) lcd.print("STARTING GPS    ");
    else if (dots == 1) lcd.print("STARTING GPS.   ");
    else if (dots == 2) lcd.print("STARTING GPS..  ");
    else if (dots == 3) lcd.print("STARTING GPS... ");

    introElapsed += 1;
    delay(1000);
  }
}

// -------------------- Loop --------------------
void loop() {
  if (digitalRead(BTN_MUTE) == HIGH) {
    isMuted = true;
  }
  
  // Calculate distance
  int potValue = analogRead(POT);

  int dist = map(potValue, 0, 4095, 0, 180);

  // Display Text
  lcd.setCursor(0,1);
  lcd.print("DIST: ");
  lcd.print(dist, 1);
  lcd.print(" m     ");

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
    lcd.print("ATENCAO");
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

  delay(100);
}
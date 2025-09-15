#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHTPIN 4


LiquidCrystal_I2C lcd(0x27, 20, 4);

DHT dht(DHTPIN, DHT22);


void setup() {  
  lcd.init();
  lcd.backlight();
  lcd.clear();

  dht.begin();
}


void loop() {
  lcd.clear();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    lcd.setCursor(0,0);
    Serial.println("Failed to read!");
    return;
  }

  float hic = dht.computeHeatIndex(t, h, false);

  lcd.setCursor(0,0);
  lcd.print("Temp (oC): ");
  lcd.setCursor(11,0);
  lcd.print(t);

  lcd.setCursor(0,1);
  lcd.print("Humd (%): ");
  lcd.setCursor(10,1);
  lcd.print(h);

  lcd.setCursor(0,2);
  lcd.print("H.I. (oC): ");
  lcd.setCursor(11,2);
  lcd.print(hic);

  delay(5000);
}
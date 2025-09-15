#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>

// Define the RX and TX pins for Serial 2
#define RXD2 16
#define TXD2 17

#define GPS_BAUD 9600

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

// Chamada da funcação LiquidCrystal para ser usada com o I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);

// The TinyGPS++ object
TinyGPSPlus gps;

void setup(){
  // Serial Monitor
  //Serial.begin(115200);

  lcd.init();      // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  lcd.clear(); // Serve para limpar a tela do display
  
  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  //Serial.println("Serial 2 started at 9600 baud rate");

  lcd.setCursor(0,0); // Coloca o cursor do display na coluna 1 e linha 1
  lcd.print("GPS SERVICE STARTED"); // Comando de saída com a mensagem que deve aparecer na coluna 2 e linha 1.
}

int i = 0;

void loop(){
  // This sketch displays information every time a new sentence is correctly encoded.
  unsigned long start = millis();

  lcd.clear();
  if(i==0) {
    lcd.setCursor(0,2);
    lcd.print("Accquiring signal.");
    i++;
  } else if(i==1) {
    lcd.setCursor(0,2);
    lcd.print("Accquiring signal..");
    i++;
  } else if(i==2) {
    lcd.setCursor(0,2);
    lcd.print("Accquiring signal...");
    i=0;
  }
  

 while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      lcd.clear();

      lcd.setCursor(0,0);
      lcd.print("LAT: ");
      lcd.print(gps.location.lat());

      lcd.setCursor(1,0);
      lcd.print("LONG: "); 
      lcd.print(gps.location.lng());
      //Serial.print("SPEED (km/h) = "); 
      //Serial.println(gps.speed.kmph()); 
      //Serial.print("ALT (min)= "); 
      //Serial.println(gps.altitude.meters());
      //Serial.print("HDOP = "); 
      //Serial.println(gps.hdop.value() / 100.0); 
      lcd.setCursor(2,0);
      lcd.print("Sat: "); 
      lcd.print(gps.satellites.value()); 

      lcd.setCursor(3,0);
      lcd.print("UTC: ");
      lcd.print(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      //Serial.println("");
    }
  }
}
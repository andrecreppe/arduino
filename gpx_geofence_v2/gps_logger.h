#ifndef GPS_LOGGER_H
#define GPS_LOGGER_H

#include <Arduino.h>


// -------------------- SD Configuration --------------------
#define LOG_BUFFER_SIZE 512 // bytes


// -------------------- Internal State --------------------
String filename = "/gps_log.csv";
static File gpsLogFile;

static char logBuffer[LOG_BUFFER_SIZE];
static size_t bufferIndex = 0;
static int lastLoggedSecond = -1;


// -------------------- Helpers --------------------
inline String gpsTimestamp(const TinyGPSPlus &globalgps) {
  TinyGPSPlus gps = globalgps;

  char ts[25];
  snprintf(ts, sizeof(ts),
           "%04d-%02d-%02d %02d:%02d:%02d",
           gps.date.year(),
           gps.date.month(),
           gps.date.day(),
           gps.time.hour(),
           gps.time.minute(),
           gps.time.second());
  return String(ts);
}


// -------------------- Buffer Handling --------------------
inline void flushGPSBuffer() {
  if (gpsLogFile && bufferIndex > 0) {
    gpsLogFile.write((uint8_t *)logBuffer, bufferIndex);
    gpsLogFile.flush();
    bufferIndex = 0;
  }
}

inline void appendToGPSBuffer(const String &line) {
  if (bufferIndex + line.length() >= LOG_BUFFER_SIZE)
    flushGPSBuffer();

  memcpy(&logBuffer[bufferIndex], line.c_str(), line.length());
  bufferIndex += line.length();
}


// -------------------- Init (File Rotation Per Boot) --------------------
inline bool initGPSLogger(const int &sd_cs) {
  if (!SD.begin(sd_cs))
    return false;

  // File rotation using millis at boot (sufficient + deterministic)
  char filename[32];
  snprintf(filename, sizeof(filename), "/gps_%lu.csv", millis());

  gpsLogFile = SD.open(filename, FILE_WRITE);
  if (!gpsLogFile)
    return false;

  gpsLogFile.println("timestamp,lat,lon,alt_m,speed_kmph,sats,dist_m");
  gpsLogFile.flush();

  return true;
}

// -------------------- Public Logging Function --------------------
inline void logGPSFix(const TinyGPSPlus &globalgps,
                      double distToRectangle) {

  TinyGPSPlus gps = globalgps;

  if (!gps.location.isValid() || !gps.time.isValid() || !gps.date.isValid())
    return;

  int currentSecond = gps.time.second();

  // Enforce exactly one log per GPS second
  if (currentSecond == lastLoggedSecond)
    return;

  lastLoggedSecond = currentSecond;

  String line;
  line.reserve(96);

  line += gpsTimestamp(gps);
  line += ",";
  line += String(gps.location.lat(), 6);
  line += ",";
  line += String(gps.location.lng(), 6);
  line += ",";
  line += String(gps.altitude.meters(), 1);
  line += ",";
  line += String(gps.speed.kmph(), 1);
  line += ",";
  line += String(gps.satellites.value());
  line += ",";
  line += String(distToRectangle, 1);
  line += "\n";

  appendToGPSBuffer(line);
}

#endif // GPS_LOGGER_H
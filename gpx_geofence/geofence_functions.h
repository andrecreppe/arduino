#ifndef GEOFENCE_FUNCTIONS_H
#define GEOFENCE_FUNCTIONS_H

#include <Arduino.h>

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
extern Point runway[4];

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

#endif // GEOFENCE_FUNCTIONS_H
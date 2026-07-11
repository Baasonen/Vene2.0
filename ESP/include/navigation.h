#pragma once

#include <math.h>

const float degToRad = M_PI / 180.0;
const float radToDeg = 180.0 / M_PI;

#define EARTHRADIUS 6371000.0

#define LOOKAHEAD_DISTANCE 8.0f

float distanceToPoint(double lat1, double lon1, double lat2, double lon2);
float headingToPoint(double lat1, double lon1, double lat2, double lon2);
float lookaheadHeading(double lat, double lon, double prevLat, double prevLon,
                       double targetLat, double targetLon);

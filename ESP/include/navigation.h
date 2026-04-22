#pragma once

#include <math.h>

const float degToRad = M_PI / 180.0;
const float radToDeg = 180.0 / M_PI;

#define EARTHRADIUS 6371000.0

float distanceToPoint(double lat1, double lon1, double lat2, double lon2);
float headingToPoint(double lat1, double lon1, double lat2, double lon2);

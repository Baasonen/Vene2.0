#include "navigation.h"

float distanceToPoint(double lat1, double lon1, double lat2, double lon2)
{
    float dLat = (lat2 - lat1) * degToRad;
    float dLon = (lon2 - lon1) * degToRad;

    float latMean = (lat1 + lat2) * 0.5 * degToRad;

    // Maapallo on (kai) pyöreä, huomioi pituusasteiden välinen etäisyys
    dLon *= cos(latMean);

    float res = sqrt(dLon * dLon + dLat * dLat);

    return res * EARTHRADIUS;
}

float headingToPoint(double lat1, double lon1, double lat2, double lon2)
{
    float dLat = (lat2 - lat1) * degToRad;
    float dLon = (lon2 - lon1) * degToRad;

    float latMean = (lat1 + lat2) * 0.5 * degToRad;

    dLon *= cos(latMean);

    float heading = atan2(dLon, dLat) * radToDeg;
    if (heading < 0) {heading += 360.0;}

    return heading;
}


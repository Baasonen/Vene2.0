#include "navigation.h"

float distanceToPoint(double lat1, double lon1, double lat2, double lon2)
{
    float dLat = (lat2 - lat1) * degToRad;
    float dLon = (lon2 - lon1) * degToRad;

    float latMean = (lat1 + lat2) * 0.5 * degToRad;

    // Maapallo on (kai) pyöreä, huomioi pituusasteiden välinen etäisyys
    dLon *= cos(latMean);

    float res = sqrt(dLon * dLon + dLat * dLat);

    return res * EARTHRADIUS_M;
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

// Project a lat/lon point into local plane relative to a reference point
static void toLocalMeters(double lat, double lon, double refLat, double refLon, float& x, float& y)
{
    float dLat = (lat - refLat) * degToRad;
    float dLon = (lon - refLon) * degToRad;

    y = dLat * EARTHRADIUS_M;
    x = dLon * cos(refLat * degToRad) * EARTHRADIUS_M;
}

float lookaheadHeading(double lat, double lon, double prevLat, double prevLon,
                       double targetLat, double targetLon)
{
    float px;
    float py;

    float bx;
    float by;

    toLocalMeters(lat, lon, prevLat, prevLon, px, py);
    toLocalMeters(targetLat, targetLon, prevLat, prevLon, bx, by);

    float legLenSqr = bx * bx + by * by;

    // Prev and next almost identical, steer straight at target
    if (legLenSqr < 1.0f)
    {
        return headingToPoint(lat, lon, targetLat, targetLon);
    }

    float legLen = sqrt(legLenSqr);

    // Project pos to leg (0 ... 1)
    float t = (px * bx + py * by) / legLenSqr;
    if (t < 0.0f) {t = 0.0f;}
    if (t > 1.0f) {t = 1.0f;}

    float alongTrack = t * legLen;
    float targetS = alongTrack + LOOKAHEAD_DISTANCE;

    float carrotX;
    float carrotY;

    if (targetS >= legLen)
    {
        // Lookahead point at or beyond tgt point, steer straight to tgt
        carrotX = bx;
        carrotY = by;
    }
    else
    {
        float frac = targetS / legLen;

        carrotX = bx * frac;
        carrotY = by * frac;
    }

    float dx = carrotX - px;
    float dy = carrotY - py;

    float heading = atan2(dx, dy) * radToDeg;
    if (heading < 0) {heading += 360.0f;}

    return heading;
}
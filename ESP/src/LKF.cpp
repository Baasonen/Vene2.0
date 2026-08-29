#include "LKF.h"

#include <Arduino.h>
#include <math.h>

#include "tuning.h"

#define EARTHRADIUS_M 6371000.0

// 1D constan velocity KF
struct KF 
{
    float pos;
    float vel;

    float p00, p01, p11; // Covariance

    float posNIS;
    float velNIS;
    float posNISAvg = 1.0f;

    void reset(float pos0, float vel0, float posVar0, float velVar0)
    {
        pos = pos0;
        vel = vel0;
        p00 = posVar0;
        p01 = 0.0f;
        p11 = velVar0;

        posNIS = 0.0f;
        velNIS = 0.0f;
        posNISAvg = 0.0f;
    }

    void predict(float dt, float qPos, float qVel)
    {
        pos += vel * dt;

        float p00n = p00 + 2.0f * dt * p01 + dt * dt * p11 + qPos;
        float p01n = p01 + dt * p11;
        float p11n = p11 + qVel;

        p00 = p00n;
        p01 = p01n;
        p11 = p11n;
    }

    void updateVel(float z, float r)
    {
        float y = z - vel;
        float s = p11 + r;
        if (s <= 0.0f) {return;}

        velNIS = (y * y) / s;

        float k0 = p01 / s;
        float k1 = p11 / s;

        pos += k0 * y;
        vel += k1 * y;

        float p00n = p00 - k0 * p01;
        float p01n = p01 - k0 * p11;
        float p11n = (1.0f - k1) * p11;

        p00 = p00n;
        p01 = p01n;
        p11 = p11n;
    }

    void update(float z, float r)
    {
        float y = z - pos;
        float s = p00 + r;
        if (s <= 0.0f) {return;}

        posNIS = (y * y) / s;

        const float NIS_SAMPLE = 0.02f;
        posNISAvg = (1.0f - NIS_SAMPLE) * posNISAvg + NIS_SAMPLE * posNIS;

        float k0 = p00 / s;
        float k1 = p01 / s;

        pos += k0 * y;
        vel += k1 * y;

        float p00n = (1.0f - k0) * p00;
        float p01n = (1.0f - k0) * p01;
        float p11n = p11 - k1 * p01;

        p00 = p00n;
        p01 = p01n;
        p11 = p11n;
    }
};

static bool initialized = false;

static double lat0 = 0.0;
static double lon0 = 0.0;

static KF north;
static KF east;

static uint32_t lastUpdateMs = 0;
static uint32_t lastValidFixMs = 0;

static bool magLatest = false;
static float lastMagHeading = 0.0f;

static void toLocal(double lat, double lon, float &n, float &e)
{
    double dLat = (lat - lat0) * DEG_TO_RAD;
    double dLon = (lon - lon0) * DEG_TO_RAD;

    n = (float)(dLat * EARTHRADIUS_M);
    e = (float)(dLon * EARTHRADIUS_M * cos(lat0 * DEG_TO_RAD));
}

static void toLLA(float n, float e, double &lat, double &lon)
{
    lat = lat0 + (n / EARTHRADIUS_M) * RAD_TO_DEG;
    lon = lon0 + (e / (EARTHRADIUS_M * cos(lat0 * DEG_TO_RAD))) * RAD_TO_DEG;
}

static bool gpsReliable(const GPSData &gps)
{
    return gps.valid && !gps.accDegraded;
}

static float updateTurnRate(const MagData &mag, float dt)
{
    if (!mag.valid || mag.accuracy < MIN_MAG_ACCURACY)
    {
        magLatest = false;
        return 0.0f;
    }

    float rate = 0.0f;

    if (magLatest && dt > 0.0f)
    {
        float dHeading = mag.heading - lastMagHeading;

        while (dHeading > 180.0f) {dHeading -= 360.0f;}
        while (dHeading < -180.0f) {dHeading += 360.0f;}

        rate = fabsf(dHeading) / dt;
    }

    lastMagHeading = mag.heading;
    magLatest = true;

    return rate;
}

void LKFInit()
{
    initialized = false;
    lat0 = lon0 = 0.0;
    lastUpdateMs = millis();
    lastValidFixMs = 0;
    magLatest = false;
    north.reset(0.0f, 0.0f, 0.0f, 0.0f);
    east.reset(0.0f, 0.0f, 0.0f, 0.0f);
}

PosSol LKFUpdate(const GPSData &gps, const MagData &mag)
{
    PosSol out = {0};

    #if GPS_SKIP_LKF

    out.valid = gps.valid;
    out.lat = gps.lat;
    out.lon = gps.lon;

    out.posStdM = gps.hAccM;
    out.posNISAvg = 0.0f;
    out.velNIS = 0.0f;

    return out;
    #else

    uint32_t now = millis();

    float dt = (now - lastUpdateMs) / 1000.0f;
    if (dt < 0.0f) {dt = 0.0f;} // If millis() wraps around
    if (dt > 1.0f) {dt = 1.0f;}
    lastUpdateMs = now;

    if (gps.valid && gps.freshFix) {lastValidFixMs = now;}

    if (!initialized)
    {
        if (!gpsReliable(gps)) {return out;} // 0.0, 0.0 if no accurate start avail

        lat0 = gps.lat;
        lon0 = gps.lon;

        float posVar = gps.hAccM * gps.hAccM;
        float velVar = 4.0f;

        north.reset(0.0f, gps.velN, posVar, velVar);
        east.reset(0.0f, gps.velE, posVar, velVar);


        lastValidFixMs = now;
        magLatest = false;

        initialized = true;

        out.lat = lat0;
        out.lon = lon0;
        out.speedKMH = gps.speedKMH;
        out.gpsHeadingDeg = 0.0f;
        out.valid = true;

        return out;
    }

    // Too long w/o reliable gps fix
    if ((now - lastValidFixMs) > REAQUIRE_TIMEOUT_MS)
    {
        LKFInit();

        return out;
    }

    float turnRateDegS = updateTurnRate(mag, dt);
    bool turning = turnRateDegS > TURN_RATE_THRESHOLD_DEG;

    float qPos = Q_POS_RATE * dt;
    float qVel = (turning ? Q_VEL_RATE_TURN : Q_VEL_RATE_STRAIGHT) * dt;

    // Predict
    north.predict(dt, qPos, qVel);
    east.predict(dt, qPos, qVel);

    // Correct with GPS pos
    if (gps.valid && gps.freshFix)
    {
        float measN, measE;

        toLocal(gps.lat, gps.lon, measN, measE);

        float measStd = gps.hAccM;
        if (measStd < MIN_MEAS_STD_M) {measStd = MIN_MEAS_STD_M;}
        
        float r = measStd * measStd;

        north.update(measN, r);
        east.update(measE, r);

        float gpsSpeedMpS = sqrtf(gps.velN * gps.velN + gps.velE * gps.velE);

        float velStd = gps.velAccM;
        if (velStd < MIN_VEL_STD_MPS) {velStd = MIN_VEL_STD_MPS;}

        float velTrust = gpsSpeedMpS / VEL_FLOOR_SPEED_MPS;
        if (velTrust > 1.0f) {velTrust = 1.0f;}

        float inflate = 1.0f + (VEL_FLOOR_INFLATE - 1.0f) * (1.0f - velTrust);
        velStd *= inflate;

        float rVel = velStd * velStd;

        north.updateVel(gps.velN, rVel);
        east.updateVel(gps.velE, rVel);
    }

    out.posNIS = (north.posNIS + east.posNIS) / 2.0f;
    out.velNIS = (north.velNIS + east.velNIS) / 2.0f;
    out.posNISAvg = (north.posNISAvg + east.posNISAvg) / 2.0f;

    double lat, lon;
    toLLA(north.pos, east.pos, lat, lon);

    float speedMpS = sqrtf(north.vel * north.vel + east.vel * east.vel);

    float headingDeg = atan2f(east.vel, north.vel) * RAD_TO_DEG;
    if (headingDeg < 0.0f) {headingDeg += 360.0f;}

    out.posStdM = sqrtf(north.p00 + east.p00);
    out.velStdMpS = sqrtf(north.p11 + east.p11);

    out.accurate = (out.posStdM < POS_STD_DEGRADED_M);

    out.lat = lat;
    out.lon = lon;
    out.speedKMH = speedMpS * 3.6f;
    out.gpsHeadingDeg = headingDeg;
    out.valid = true;

    return out;

    #endif
}
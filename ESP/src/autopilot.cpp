#include "autopilot.h"

#include "navigation.h"
#include "control.h"

bool runAutopilot(SystemStatus& status, const SensorData& sensors, const Route& route)
{
    if (status.targetWaypoint == 0) {status.targetWaypoint = 1;}

    if (status.targetWaypoint > route.length)
    {
        turnRudder(0);
        setThrottle(0);
        
        return true;
    }

    const wp& target = route.waypoints[status.targetWaypoint];

    if (distanceToPoint(sensors.gps.lat, sensors.gps.lon, target.lat, target.lon) <= WP_TRESHOLD)
    {
        status.targetWaypoint++;

        if (status.targetWaypoint > route.length)
        {
            turnRudder(0);
            setThrottle(0);

            return true;
        }
    }

    const wp& legEnd = route.waypoints[status.targetWaypoint];

    setThrottle(AP_THROTTLE);

    if (status.targetWaypoint == 1)
    {
        // First wp, steer straight at target
        steerTo(headingToPoint(sensors.gps.lat, sensors.gps.lon, legEnd.lat, legEnd.lon));
    }
    else
    {
        const wp& legStart = route.waypoints[status.targetWaypoint - 1];
        steerTo(lookaheadHeading(sensors.gps.lat, sensors.gps.lon,
                                 legStart.lat, legStart.lon,
                                 legEnd.lat, legEnd.lon));
    }

    return false;
}
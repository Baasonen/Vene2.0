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

    if (distanceToPoint(sensors.kf.lat, sensors.kf.lon, target.lat, target.lon) <= WP_TRESHOLD_M)
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

    setThrottle(status.APThrottle);

    if (status.targetWaypoint == 1)
    {
        // First wp, steer straight at target
        steerTo(headingToPoint(sensors.kf.lat, sensors.kf.lon, legEnd.lat, legEnd.lon));
    }
    else
    {
        const wp& legStart = route.waypoints[status.targetWaypoint - 1];
        steerTo(lookaheadHeading(sensors.kf.lat, sensors.kf.lon,
                                 legStart.lat, legStart.lon,
                                 legEnd.lat, legEnd.lon));
    }

    return false;
}
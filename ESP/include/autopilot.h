#pragma once

#include "state.h"

#define AP_THROTTLE 50
#define WP_TRESHOLD 3


// DOES NOT MUTATE status.mode
// Returns true when final waypoint reached, caller should handle what happens after
bool runAutopilot(SystemStatus& status, const SensorData& sensors, const Route& route);
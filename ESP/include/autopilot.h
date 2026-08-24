#pragma once

#include "state.h"
#include "tuning.h"
#include "navigation.h"
#include "control.h"

// DOES NOT MUTATE status.mode
// Returns true when final waypoint reached, caller should handle what happens after
bool runAutopilot(SystemStatus& status, const SensorData& sensors, const Route& route);
#pragma once

// Control
#define RUDDER_U_LIM 80
#define RUDDER_L_LIM -80

#define RUDDER_KP 2.0f
#define RUDDER_KI 0.03f
#define RUDDER_I_LIM 40.0f

// Thr input -100 ... 0 ... 100
#define ESC_RANGE 2.0f // RANGE * Input
#define ESC_REVERSE_DIV 4 // (RANGE * 100) / REVERSE_DIV
#define ESC_NEUTRAL 1500 

// Autopilot
#define WP_TRESHOLD_M 2.5 // Dist. from where a wp is treated as reached
#define HOME_TRESHOLD_M 5
#define LOOKAHEAD_DISTANCE 6.0f // Lookahead dist. for route following

// LoRa
#define LORA_TIMEOUT_MS 20000

#define SLOW_TELE_PERIOD_MS 4700
#define FAST_TELE_PERIOD_MS 1900
#define BE_TELE_PERIOD_MS 15300

#define DUTY_CYCLE_WINDOW_MS 60000UL  // 60s
#define DUTY_CYCLE_LIMIT_PCT 10.0f

#define CAD_MAX_ATTEMPTS 3
#define CAD_BACKOFF_MIN_MS 2
#define CAD_BACKOFF_MAX_MS 10

// GPS
#define MIN_SAT_COUNT 4

#define GPS_HACC_GATE 10
#define GPS_HACC_ACCURATE 4

// LKF
#define Q_POS_RATE 0.02f
#define Q_VEL_RATE_STRAIGHT 0.15f
#define Q_VEL_RATE_TURN 4.0f

#define TURN_RATE_THRESHOLD_DEG 8.0f
#define MIN_MAG_ACCURACY 1

#define MIN_MEAS_STD_M 0.8f 

#define REAQUIRE_TIMEOUT_MS 10000UL

#define MIN_VEL_STD_MPS 0.1f

#define POS_STD_DEGRADED_M 4.0f
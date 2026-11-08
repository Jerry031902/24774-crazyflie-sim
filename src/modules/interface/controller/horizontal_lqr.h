// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef HORIZONTAL_LQR_H
#define HORIZONTAL_LQR_H
#include "stabilizer_types.h"

/* Called at POSITION_RATE (100 Hz). WORLD position/velocity in SI units;
 * legacy Crazyflie roll/pitch and yaw in degrees. Altitude is untouched.
 * Modes: 0 bypass; 1 position hold; 2 velocity tracking; 3 brake; 4 invalid.
 * Velocity mode deliberately discards position error, including old waypoints.
 */
void horizontalLqrSetEnabled(bool enabled);
bool horizontalLqrGetEnabled(void);
void horizontalLqrSetBrake(bool brake);
void horizontalLqrReset(void);
bool horizontalLqrUpdate(const setpoint_t *sp, const state_t *state, attitude_t *desired);
float horizontalLqrGetRoll(void);
float horizontalLqrGetPitch(void);
uint8_t horizontalLqrGetMode(void);
bool horizontalLqrGetSaturated(void);
uint32_t horizontalLqrGetUpdates(void);
#endif


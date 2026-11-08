// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include "horizontal_lqr.h"
#include "horizontal_lqr_gains.h"
#include "param.h"
#include "log.h"
#ifndef UNIT_TEST_MODE
#include "usec_time.h"
#endif

static uint8_t enabled = 0, brake = 0, mode = 0, saturated = 0;
static float rollDeg, pitchDeg;
static uint32_t updates, dtUs, execUs, maxExecUs;
#ifndef UNIT_TEST_MODE
static uint64_t lastUs;
#endif
#define RAD_TO_DEG 57.29577951308232f
#define DEG_TO_RAD 0.0174532925199433f
_Static_assert(POSITION_RATE == 100, "Regenerate LQR gains for a changed update rate");

void horizontalLqrSetEnabled(bool value) { enabled = value; }
bool horizontalLqrGetEnabled(void) { return enabled != 0; }
void horizontalLqrSetBrake(bool value) { brake = value; }
float horizontalLqrGetRoll(void) { return rollDeg; }
float horizontalLqrGetPitch(void) { return pitchDeg; }
uint8_t horizontalLqrGetMode(void) { return mode; }
bool horizontalLqrGetSaturated(void) { return saturated != 0; }
uint32_t horizontalLqrGetUpdates(void) { return updates; }
void horizontalLqrReset(void) {
  rollDeg = pitchDeg = 0;
  mode = saturated = 0;
  updates = dtUs = execUs = maxExecUs = 0;
#ifndef UNIT_TEST_MODE
  lastUs = 0;
#endif
}

bool horizontalLqrUpdate(const setpoint_t *sp, const state_t *s, attitude_t *desired) {
  mode = saturated = 0;
  if (!enabled) return false;
  // Preserve commander stop/manual attitude/rate modes and the existing supervisor.
  const bool position = sp->mode.x == modeAbs && sp->mode.y == modeAbs;
  const bool velocity = sp->mode.x == modeVelocity && sp->mode.y == modeVelocity;
  if ((!position && !velocity) || sp->mode.z == modeDisable ||
      sp->mode.roll == modeVelocity || sp->mode.pitch == modeVelocity ||
      sp->mode.quat == modeAbs) return false;

#ifndef UNIT_TEST_MODE
  const uint64_t start = usecTimestamp();
  dtUs = lastUs ? (uint32_t)(start - lastUs) : 0;
  lastUs = start;
#endif
  updates++;
  const float yaw = s->attitude.yaw * DEG_TO_RAD;
  const float c = cosf(yaw), sn = sinf(yaw);
  float ex = 0, ey = 0, rvx = 0, rvy = 0;
  if (!brake && position) {
    ex = sp->position.x - s->position.x;
    ey = sp->position.y - s->position.y;
    // modeAbs is position HOLD, just like stock PID; no implicit velocity FF.
  } else if (!brake && velocity) {
    rvx = sp->velocity.x;
    rvy = sp->velocity.y;
    if (sp->velocity_body) {
      const float worldX = c * rvx - sn * rvy;
      rvy = sn * rvx + c * rvy;
      rvx = worldX;
    }
  }
  // Isotropic world-frame drag and gains: design accelerations, then rotate.
  // Velocity tracking is a separate damping mode, not a second 4-state DARE.
  const float ax = LQR_G * (LQR_KP * ex + LQR_KV * (rvx - s->velocity.x))
                   + LQR_BETA * rvx;
  const float ay = LQR_G * (LQR_KP * ey + LQR_KV * (rvy - s->velocity.y))
                   + LQR_BETA * rvy;
  float pitch = -(c * ax + sn * ay) / LQR_G;
  float roll = -(-sn * ax + c * ay) / LQR_G;
  if (!isfinite(roll) || !isfinite(pitch)) {
    // Neutral horizontal request, still retain original vertical control.
    // This is NOT a replacement for the firmware estimator-validity supervisor.
    mode = 4;
    roll = pitch = 0;
  } else {
    mode = brake ? 3 : position ? 1 : 2;
    const float limit = LQR_ANGLE_LIMIT_DEG * DEG_TO_RAD;
    saturated = fabsf(roll) > limit || fabsf(pitch) > limit;
    roll = fmaxf(-limit, fminf(limit, roll));
    pitch = fmaxf(-limit, fminf(limit, pitch));
  }
  desired->roll = rollDeg = roll * RAD_TO_DEG;
  desired->pitch = pitchDeg = pitch * RAD_TO_DEG;
#ifndef UNIT_TEST_MODE
  execUs = (uint32_t)(usecTimestamp() - start);
  if (execUs > maxExecUs) maxExecUs = execUs;
#endif
  return true;
}

PARAM_GROUP_START(hlqr)
PARAM_ADD(PARAM_UINT8, enable, &enabled)
PARAM_ADD(PARAM_UINT8, brake, &brake)
PARAM_GROUP_STOP(hlqr)

LOG_GROUP_START(hlqr)
LOG_ADD(LOG_UINT8, enabled, &enabled)
LOG_ADD(LOG_UINT8, mode, &mode)
LOG_ADD(LOG_UINT8, saturated, &saturated)
LOG_ADD(LOG_FLOAT, rollRef, &rollDeg)
LOG_ADD(LOG_FLOAT, pitchRef, &pitchDeg)
LOG_ADD(LOG_UINT32, updates, &updates)
LOG_ADD(LOG_UINT32, dtUs, &dtUs)
LOG_ADD(LOG_UINT32, execUs, &execUs)
LOG_ADD(LOG_UINT32, maxExecUs, &maxExecUs)
LOG_GROUP_STOP(hlqr)


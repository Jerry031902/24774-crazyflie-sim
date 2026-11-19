/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_flow.h"
#include "log.h"
#include "platform_defaults.h"
#include "param.h"
#include <math.h>

#define FLOW_RESOLUTION 0.10f //We get the measurements in 10x the motion pixels (experimentally measured)

// TODO remove the temporary test variables (used for logging)
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;
static float flowNisX = NAN;
static float flowNisY = NAN;
static float flowInnovationVarianceX = NAN;
static float flowInnovationVarianceY = NAN;
static uint8_t flowGateEnabled = 0;
static float flowGateThreshold = 9.0f;
static uint16_t flowGateMaxReject = FLOW_GATE_DEFAULT_MAX_REJECT;
static uint16_t flowGateRecoverK = FLOW_GATE_DEFAULT_RECOVER_K;
static kalmanScalarGateState_t flowGateLogX;
static kalmanScalarGateState_t flowGateLogY;

bool kalmanCoreSetFlowGateConfig(uint8_t enabled, float threshold, uint16_t maxReject)
{
  if (enabled > 2 || !isfinite(threshold) || threshold <= 0 || maxReject == 0) { return false; }
  flowGateThreshold = threshold;
  flowGateMaxReject = maxReject;
  flowGateEnabled = enabled;
  return true;
}

uint8_t kalmanCoreGetFlowGateEnabled(void) { return flowGateEnabled; }
float kalmanCoreGetFlowGateThreshold(void) { return flowGateThreshold; }
uint16_t kalmanCoreGetFlowGateMaxReject(void) { return flowGateMaxReject; }
uint16_t kalmanCoreGetFlowGateDefaultMaxReject(void) { return FLOW_GATE_DEFAULT_MAX_REJECT; }
bool kalmanCoreSetFlowGateRecoverK(uint16_t recoverK)
{
  if (recoverK == 0) { return false; }
  flowGateRecoverK = recoverK;
  return true;
}
uint16_t kalmanCoreGetFlowGateRecoverK(void) { return flowGateRecoverK; }
uint16_t kalmanCoreGetFlowGateDefaultRecoverK(void) { return FLOW_GATE_DEFAULT_RECOVER_K; }
kalmanScalarGateState_t kalmanCoreGetFlowGateX(const kalmanCoreData_t *core) { return core->flowGateX; }
kalmanScalarGateState_t kalmanCoreGetFlowGateY(const kalmanCoreData_t *core) { return core->flowGateY; }

float kalmanCoreGetFlowNisX(void) { return flowNisX; }
float kalmanCoreGetFlowNisY(void) { return flowNisY; }
float kalmanCoreGetFlowInnovationVarianceX(void) { return flowInnovationVarianceX; }
float kalmanCoreGetFlowInnovationVarianceY(void) { return flowInnovationVarianceY; }
float kalmanCoreGetFlowPredictionX(void) { return predictedNX; }
float kalmanCoreGetFlowPredictionY(void) { return predictedNY; }

static Axis3f flowdeckPos = { .axis = { FLOWDECK_POS_X, FLOWDECK_POS_Y, FLOWDECK_POS_Z } }; // In body coordinate system

void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates
  // Snapshot runtime configuration once for both axes. Do not retain a reject
  // streak across parameter changes; totals still count all attempted updates.
  const uint8_t enabled = flowGateEnabled;
  const float threshold = flowGateThreshold;
  const uint16_t maxReject = flowGateMaxReject;
  const uint16_t recoverK = flowGateRecoverK;
  if (this->flowGateEnabled != enabled || this->flowGateThreshold != threshold || this->flowGateMaxReject != maxReject ||
      (enabled == 2 && this->flowGateRecoverK != recoverK)) {
    this->flowGateX.consecutive = this->flowGateY.consecutive = 0;
    this->flowGateX.recoveryConsecutive = this->flowGateY.recoveryConsecutive = 0;
    this->flowGateX.state = this->flowGateY.state = KC_FLOW_ARMED;
    this->flowGateEnabled = enabled;
    this->flowGateThreshold = threshold;
    this->flowGateMaxReject = maxReject;
    this->flowGateRecoverK = recoverK;
  }

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 35.0;         // [pixels] (same in x and y)
  float thetapix = 0.71674f; // [rad] 2*sin(42/2); 42 degrees is the angle of aperture, here we computed the corresponding ground length

  //~~~ Extract states ~~~
  // Body rates
  float omegax_b = gyro->x * DEG_TO_RAD;
  float omegay_b = gyro->y * DEG_TO_RAD;
  float omegaz_b = gyro->z * DEG_TO_RAD;

  // Velocities in body frame
  float dx_b = this->S[KC_STATE_PX];
  float dy_b = this->S[KC_STATE_PY];

  // Height above origin
  float z_g = 0.0;

  // Saturate height in prediction and correction to avoid singularities
  if ( this->S[KC_STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = this->S[KC_STATE_Z];
  }

  // Lever-arm induced translational velocity at camera
  // omega x r
  float v_cam_bx_add =  omegay_b * flowdeckPos.z - omegaz_b * flowdeckPos.y;
  float v_cam_by_add =  omegaz_b * flowdeckPos.x - omegax_b * flowdeckPos.z;
  
  // Effective camera point velocities in body frame
  float v_cam_bx = dx_b + v_cam_bx_add;
  float v_cam_by = dy_b + v_cam_by_add;

  // X velocity prediction and update
  // predicts the number of accumulated pixels in the x-direction
  float hx[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, KC_STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix) * ((v_cam_bx * this->R[2][2] / z_g) - omegay_b);
  measuredNX = flow->dpixelx*FLOW_RESOLUTION;

  // derive measurement equation with respect to dx (and z?)
  hx[KC_STATE_Z]  = (Npix * flow->dt / thetapix) * ((this->R[2][2] * v_cam_bx) / (-z_g * z_g));
  hx[KC_STATE_PX] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  //First update
  if (enabled == 2) {
    flowInnovationVarianceX = kalmanCoreScalarUpdateWithRecoveryGate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION,
      &flowNisX, &this->flowGateX, threshold, maxReject, recoverK);
  } else {
    flowInnovationVarianceX = kalmanCoreScalarUpdateWithGate(this, &Hx, (measuredNX-predictedNX), flow->stdDevX*FLOW_RESOLUTION,
      &flowNisX, &this->flowGateX, enabled, threshold, maxReject);
  }

  // Y velocity prediction and update
  float hy[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, KC_STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((v_cam_by * this->R[2][2] / z_g) + omegax_b);
  measuredNY = flow->dpixely*FLOW_RESOLUTION;

  // derive measurement equation with respect to dy (and z?)
  hy[KC_STATE_Z]  = (Npix * flow->dt / thetapix) * ((this->R[2][2] * v_cam_by) / (-z_g * z_g));
  hy[KC_STATE_PY] = (Npix * flow->dt / thetapix) * (this->R[2][2] / z_g);

  // Second update
  if (enabled == 2) {
    flowInnovationVarianceY = kalmanCoreScalarUpdateWithRecoveryGate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION,
      &flowNisY, &this->flowGateY, threshold, maxReject, recoverK);
  } else {
    flowInnovationVarianceY = kalmanCoreScalarUpdateWithGate(this, &Hy, (measuredNY-predictedNY), flow->stdDevY*FLOW_RESOLUTION,
      &flowNisY, &this->flowGateY, enabled, threshold, maxReject);
  }
  flowGateLogX = this->flowGateX;
  flowGateLogY = this->flowGateY;
}

/**
 * Predicted and measured values of the X and Y direction of the flowdeck
 */
LOG_GROUP_START(kalman_pred)

/**
 * @brief Flow sensor predicted dx  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowX?
 */
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
/**
 * @brief Flow sensor predicted dy  [pixels/frame]
 * 
 *  note: rename to kalmanMM.flowY?
 */
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
/**
 * @brief Flow sensor measured dx  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaX, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
/**
 * @brief Flow sensor measured dy  [pixels/frame]
 * 
 *  note: This is the same as motion.deltaY, so perhaps remove this?
 */
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
/**
 * @brief Flow X scalar NIS before its accept/reject decision (dimensionless)
 */
  LOG_ADD(LOG_FLOAT, nisX, &flowNisX)
/**
 * @brief Flow Y scalar NIS using covariance after the X decision (dimensionless)
 */
  LOG_ADD(LOG_FLOAT, nisY, &flowNisY)
LOG_GROUP_STOP(kalman_pred)

// Latest flow call, like kalman_pred. Totals are per-core reset epoch.
LOG_GROUP_START(flowGate)
  LOG_ADD(LOG_UINT8, decX, &flowGateLogX.decision)
  LOG_ADD(LOG_UINT8, decY, &flowGateLogY.decision)
  LOG_ADD(LOG_UINT16, seqX, &flowGateLogX.consecutive)
  LOG_ADD(LOG_UINT16, seqY, &flowGateLogY.consecutive)
  LOG_ADD(LOG_UINT16, runX, &flowGateLogX.runAtDecision)
  LOG_ADD(LOG_UINT16, runY, &flowGateLogY.runAtDecision)
  LOG_ADD(LOG_UINT32, accX, &flowGateLogX.accepted)
  LOG_ADD(LOG_UINT32, accY, &flowGateLogY.accepted)
  LOG_ADD(LOG_UINT32, rejX, &flowGateLogX.rejected)
  LOG_ADD(LOG_UINT32, rejY, &flowGateLogY.rejected)
  LOG_ADD(LOG_UINT32, forceX, &flowGateLogX.forced)
  LOG_ADD(LOG_UINT32, forceY, &flowGateLogY.forced)
  LOG_ADD(LOG_UINT8, stateX, &flowGateLogX.state)
  LOG_ADD(LOG_UINT8, stateY, &flowGateLogY.state)
  LOG_ADD(LOG_UINT8, usedX, &flowGateLogX.stateAtUpdate)
  LOG_ADD(LOG_UINT8, usedY, &flowGateLogY.stateAtUpdate)
  LOG_ADD(LOG_UINT16, goodX, &flowGateLogX.recoveryConsecutive)
  LOG_ADD(LOG_UINT16, goodY, &flowGateLogY.recoveryConsecutive)
  LOG_ADD(LOG_UINT16, goodRunX, &flowGateLogX.recoveryRunAtDecision)
  LOG_ADD(LOG_UINT16, goodRunY, &flowGateLogY.recoveryRunAtDecision)
  LOG_ADD(LOG_UINT32, entryX, &flowGateLogX.recoveryEntries)
  LOG_ADD(LOG_UINT32, entryY, &flowGateLogY.recoveryEntries)
  LOG_ADD(LOG_UINT32, recSmplX, &flowGateLogX.samplesInRecovery)
  LOG_ADD(LOG_UINT32, recSmplY, &flowGateLogY.samplesInRecovery)
LOG_GROUP_STOP(flowGate)

PARAM_GROUP_START(flowGate)
  PARAM_ADD(PARAM_UINT8, enable, &flowGateEnabled)
  PARAM_ADD(PARAM_FLOAT, threshold, &flowGateThreshold)
  PARAM_ADD(PARAM_UINT16, maxReject, &flowGateMaxReject)
  PARAM_ADD(PARAM_UINT16, recoverK, &flowGateRecoverK)
PARAM_GROUP_STOP(flowGate)

/**
 * Flowdeck properties
 */
PARAM_GROUP_START(flowdeck)
  /**
   * @brief Flow deck position X (in meters, body frame)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_x, &flowdeckPos.x)
  /**
   * @brief Flow deck position Y (in meters, body frame)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_y, &flowdeckPos.y)
  /**
   * @brief Flow deck position Z (in meters, body frame)
   */
  PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, flowdeckPos_z, &flowdeckPos.z)
PARAM_GROUP_STOP(flowdeck)

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

#pragma once

#include "kalman_core.h"

// Measurements of flow (dnx, dny)
void kalmanCoreUpdateWithFlow(kalmanCoreData_t* this, const flowMeasurement_t *flow, const Axis3f *gyro);

// Read-only diagnostics from the most recent flow call (shared, like kalman_pred).
// Read immediately after updating the intended core. NAN before the first call;
// held until another flow update, including across core resets. Dimensionless.
float kalmanCoreGetFlowNisX(void);
float kalmanCoreGetFlowNisY(void);

// Exact scalar HPHR [pixels^2] and cached prediction [pixels]. Read immediately
// after the intended flow call; held across resets. Variance starts as NAN;
// prediction retains the existing kalman_pred startup value (zero).
float kalmanCoreGetFlowInnovationVarianceX(void);
float kalmanCoreGetFlowInnovationVarianceY(void);
float kalmanCoreGetFlowPredictionX(void);
float kalmanCoreGetFlowPredictionY(void);

// Runtime parameter defaults. MaxReject is the high-NIS run length at which
// the current sample is forced: N-1 skipped, Nth accepted, then streak = 0.
#define FLOW_GATE_DEFAULT_MAX_REJECT 10
bool kalmanCoreSetFlowGateConfig(uint8_t enabled, float threshold, uint16_t maxReject);
uint8_t kalmanCoreGetFlowGateEnabled(void);
float kalmanCoreGetFlowGateThreshold(void);
uint16_t kalmanCoreGetFlowGateMaxReject(void);
uint16_t kalmanCoreGetFlowGateDefaultMaxReject(void);
kalmanScalarGateState_t kalmanCoreGetFlowGateX(const kalmanCoreData_t *core);
kalmanScalarGateState_t kalmanCoreGetFlowGateY(const kalmanCoreData_t *core);

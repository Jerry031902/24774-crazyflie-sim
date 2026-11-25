/**
 * Host-test stand-ins for the firmware APIs wedgebug_app.c uses: FreeRTOS
 * ticks, commander, supervisor, CRTP link, estimator type, battery, debug
 * print, and the log/param tables. Implemented by test_app.c.
 *
 * LOG_ and PARAM_ macros register each entry (with its file and line, so
 * test_app.c can recover the group) instead of placing it in a linker section.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"

// FreeRTOS, 1 kHz tick
typedef uint32_t TickType_t;
#define M2T(X) ((TickType_t)(X))
#define T2M(X) ((uint32_t)(X))
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t* previousWakeTime, TickType_t increment);

// app
void appMain(void);

// commander
#define COMMANDER_PRIORITY_DISABLE 0
#define COMMANDER_PRIORITY_LOWEST 1
#define COMMANDER_PRIORITY_HIGHLEVEL 1
#define COMMANDER_PRIORITY_CRTP 2
#define COMMANDER_PRIORITY_EXTRX 3
void commanderSetSetpoint(setpoint_t* setpoint, int priority);
void commanderRelaxPriority(void);

// supervisor, link, battery
bool supervisorRequestArming(const bool doArm);
bool supervisorCanFly(void);
bool supervisorIsTumbled(void);
bool supervisorIsCrashed(void);
bool supervisorIsLocked(void);
bool crtpIsConnected(void);
bool pmIsBatteryLow(void);

// estimator
typedef enum {
  StateEstimatorTypeAutoSelect = 0,
  StateEstimatorTypeComplementary,
  StateEstimatorTypeKalman,
  StateEstimatorType_COUNT,
} StateEstimatorType;
StateEstimatorType stateEstimatorGetType(void);
static inline void estimatorEnqueueTOF(const tofMeasurement_t* tof) { (void)tof; }

// debug
void testDebugPrint(const char* module, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
#define DEBUG_PRINT(fmt, ...) testDebugPrint(DEBUG_MODULE, fmt, ##__VA_ARGS__)

// log / param lookup
typedef uint16_t logVarId_t;
logVarId_t logGetVarId(const char* group, const char* name);
float logGetFloat(logVarId_t varid);

typedef struct {
  uint16_t id;
  uint16_t index;
} paramVarId_t;
#define PARAM_VARID_IS_VALID(varId) ((varId).id != 0xffffu)
paramVarId_t paramGetVarId(const char* group, const char* name);
unsigned int paramGetUint(paramVarId_t varid);
void paramSetInt(paramVarId_t varid, int valuei);

// log / param tables
enum { TEST_PARAM_GROUP, TEST_PARAM, TEST_LOG_GROUP, TEST_LOG };
enum { LOG_UINT8 = 1, LOG_UINT16, LOG_UINT32, LOG_INT8, LOG_INT16, LOG_INT32, LOG_FLOAT };
enum { PARAM_UINT8 = 1, PARAM_UINT16, PARAM_UINT32, PARAM_INT8, PARAM_INT16, PARAM_INT32, PARAM_FLOAT };
void testRegister(int kind, const char* file, int line, const char* name, int type, void* address);

#define TEST_CAT2(a, b) a##b
#define TEST_CAT(a, b) TEST_CAT2(a, b)
#define TEST_REGISTER(kind, name, type, address)                                     \
  static void __attribute__((constructor)) TEST_CAT(testRegister_, __LINE__)(void) { \
    testRegister(kind, __FILE__, __LINE__, name, type, (void*)(address));           \
  }
#define PARAM_GROUP_START(NAME) TEST_REGISTER(TEST_PARAM_GROUP, #NAME, 0, 0)
#define PARAM_GROUP_STOP(NAME)
#define PARAM_ADD(TYPE, NAME, ADDRESS) TEST_REGISTER(TEST_PARAM, #NAME, TYPE, ADDRESS)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) PARAM_ADD(TYPE, NAME, ADDRESS)
#define LOG_GROUP_START(NAME) TEST_REGISTER(TEST_LOG_GROUP, #NAME, 0, 0)
#define LOG_GROUP_STOP(NAME)
#define LOG_ADD(TYPE, NAME, ADDRESS) TEST_REGISTER(TEST_LOG, #NAME, TYPE, ADDRESS)
#define LOG_ADD_CORE(TYPE, NAME, ADDRESS) LOG_ADD(TYPE, NAME, ADDRESS)

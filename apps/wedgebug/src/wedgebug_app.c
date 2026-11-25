/**
 * WedgeBug mission app: flies the navigator in wedgebug.c on a Crazyflie 2.x
 * with a Flow deck v2 (state estimate) and a Multi-ranger deck (obstacles).
 *
 * wedgebug.c is the same C code the Isaac Sim missions fly through the
 * cffirmware bindings; this file does on the drone what
 * sim/mission_flight.py does around it in the sim:
 *
 *   IDLE      motors off, waiting for wedgebug.start = 1
 *   PREFLIGHT reset the EKF, check the decks, attitude, height and rangers, arm
 *   TAKEOFF   climb at 0.2 m/s to wedgebug.height over the start, holding x/y/yaw
 *   SETTLE    hover there until still for 1 s
 *   NAVIGATE  wedgebugStep() at 50 Hz, output sent as a hover setpoint:
 *             body-frame vx/vy, yaw rate, absolute height
 *   ARRIVE    every target done: brake
 *   HOLD      hold position for wedgebug.holdTime
 *   LAND      descend at 0.2 m/s where it is, then stop the motors
 *
 * Setting wedgebug.start = 0 at any time lands where the drone is. So does any
 * guard (wedgebug.reason says which): mission timeout, fence box, radio link
 * lost, Multi-ranger frames stopped, a hand held over the up sensor, low
 * battery, non-finite navigator output. The supervisor (tumble, crash,
 * emergency stop) cuts the motors itself; the app then stops too. None of this
 * replaces an operator ready to send an emergency stop.
 *
 * Frames. Targets, startX/startY and psi0 are in the mission (room) frame,
 * as in sim/navigation/missions.py. The EKF is reset just before takeoff, so
 * its own frame starts at the drone with x along the nose: place the drone at
 * (startX, startY) with its nose pointing psi0 (counter-clockwise from the
 * room's +x). The app maps between the two frames. Flow-deck position drifts;
 * expect centimetres to decimetres of error by the end of a mission.
 *
 * Multi-ranger readings reach the navigator as the sim's held 10 Hz values:
 * metres, INFINITY for no return. The stock driver writes 32767 mm for any
 * reading that is not RANGE_VALID, so on the drone "invalid" and "nothing in
 * range" both arrive as INFINITY (the sim can tell them apart; NAN there).
 * Each new frame is detected with rangeGetUpdateCount() and paired with the
 * yaw the drone had wedgebug.latency seconds earlier, which is what the sim's
 * timestamped frames carry (safety/transport.py).
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "commander.h"
#include "crtp.h"
#include "debug.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "pm.h"
#include "range.h"
#include "stabilizer_types.h"
#include "supervisor.h"

#include "wedgebug.h"

#define DEBUG_MODULE "WEDGEBUG"

#define PI_F 3.14159265358979323846f
#define RAD_TO_DEG_F (180.0f / PI_F)
#define DEG_TO_RAD_F (PI_F / 180.0f)

#define TICK_MS 20                   // navigator and setpoint period: 50 Hz, as in the sim
#define MAX_TARGETS 8                // waypoints + goal settable through parameters
#define SETPOINT_PRIORITY COMMANDER_PRIORITY_EXTRX  // above radio (CRTP) setpoints while flying
#define RANGER_INVALID_MM 32767.0f   // multiranger.c: reading not RANGE_VALID
#define VERTICAL_SPEED 0.2f          // takeoff and landing [m/s]
#define HEIGHT_MIN 0.2f              // allowed wedgebug.height [m]
#define HEIGHT_MAX 1.0f
#define SETTLE_POS_TOL 0.10f         // SETTLE: within this of the start point [m]...
#define SETTLE_SPEED_TOL 0.10f       // ...and slower than this [m/s]...
#define SETTLE_HOLD_MS 1000          // ...for this long, as the sim's 1 s
#define SETTLE_TIMEOUT_MS 10000
#define ARRIVE_SPEED 0.05f           // the sim's goal_vtol [m/s]
#define ARRIVE_TIMEOUT_MS 3000
#define EKF_SETTLE_MS 2000           // after the EKF reset, before checking the estimate
#define ARM_TIMEOUT_MS 2000
#define PREFLIGHT_MAX_TILT_DEG 15.0f
#define PREFLIGHT_MAX_Z 0.2f         // must start on the floor [m]
#define PREFLIGHT_MAX_XY 0.1f        // EKF x/y right after its reset [m]
#define PREFLIGHT_MAX_YAW (10.0f * DEG_TO_RAD_F)  // EKF yaw right after its reset
#define RANGER_MIN_FRAMES 2          // frames seen in PREFLIGHT before takeoff
#define RANGER_STALE_MS 500          // no frame for this long while navigating: land
#define UP_STOP_FRAMES 3             // consecutive up readings under upStop: land (0.3 s)
#define LAND_TAIL_MS 500             // hold z = 0 this long before stopping the motors
#define YAW_HISTORY 64               // 1.28 s of yaw at 50 Hz
#define V_LIMIT 1.0f                 // cap on commanded horizontal speed [m/s]
#define YAW_RATE_LIMIT PI_F          // cap on commanded yaw rate [rad/s]

typedef enum {
  STATE_IDLE = 0,
  STATE_PREFLIGHT = 1,
  STATE_TAKEOFF = 2,
  STATE_SETTLE = 3,
  STATE_NAVIGATE = 4,
  STATE_ARRIVE = 5,
  STATE_HOLD = 6,
  STATE_LAND = 7,
} appState_t;

static const char* const STATE_NAMES[] = {
  "IDLE", "PREFLIGHT", "TAKEOFF", "SETTLE", "NAVIGATE", "ARRIVE", "HOLD", "LAND",
};

// Why the last mission ended, or why it is landing (log wedgebug.reason).
typedef enum {
  REASON_NONE = 0,           // idle since boot, or flying normally
  REASON_DONE = 1,           // every target handled, landed
  REASON_ABORT = 2,          // wedgebug.start set to 0
  REASON_BAD_MISSION = 3,    // target, height or fence parameters invalid
  REASON_NO_DECK = 4,        // Flow deck v2 or Multi-ranger missing, or EKF not active
  REASON_PREFLIGHT = 5,      // not upright, not on the floor, or EKF not reset
  REASON_NO_RANGER = 6,      // no Multi-ranger frames before takeoff
  REASON_NOT_ARMED = 7,      // the supervisor did not allow flight
  REASON_SETTLE = 8,         // did not settle over the start point
  REASON_TIMEOUT = 9,        // navigation took longer than wedgebug.maxTime
  REASON_FENCE = 10,         // left the fence box, or above zMax
  REASON_LINK = 11,          // radio link lost for wedgebug.linkLoss seconds
  REASON_RANGER_STALE = 12,  // Multi-ranger frames stopped arriving
  REASON_UP_STOP = 13,       // something (a hand) held over the up sensor
  REASON_SUPERVISOR = 14,    // tumbled, crashed, locked or emergency stop: motors already cut
  REASON_NONFINITE = 15,     // state estimate or navigator output not finite
  REASON_BATTERY = 16,       // battery low
} appReason_t;

// ----- parameters ---------------------------------------------------------------

// Mission (group wedgebug). Targets and fence in the mission frame [m].
static uint8_t start = 0;
static uint8_t nTargets = 1;
static float targetX[MAX_TARGETS] = {1.0f};  // default: 1 m straight ahead
static float targetY[MAX_TARGETS] = {0.0f};
static float startX = 0.0f, startY = 0.0f;
static float psi0 = 0.0f;                    // takeoff heading [rad]
static float height = 0.5f;                  // flight height [m]
static float maxTime = 120.0f;               // navigation time limit [s]
static float holdTime = 2.0f;                // hover at the goal before landing [s]
static float fenceXMin = -1.0f, fenceXMax = 3.0f, fenceYMin = -2.0f, fenceYMax = 2.0f;
static float zMax = 1.5f;                    // [m]
static float linkLoss = 1.0f;                // land after this long without radio [s]; 0 = never
static float upStop = 0.2f;                  // up reading under this lands [m]; 0 = off
static float latency = 0.1f;                 // ranger measurement to delivery [s], the sim's ranger.latency
static uint8_t tsRanges = 1;                 // 1: timestamped frames, as the sim missions; 0: baseline

// Navigator settings (group wbnav), copied into the navigator at each start.
// The same values as wedgebugDefaultParams() (checked at boot): a static
// initializer, because the firmware startup runs no constructors and a host
// may write parameters before appMain() starts.
#define WB_DEG(d) ((d) * DEG_TO_RAD_F)
#define NAV_DEFAULTS {                                                              \
    .yawRateMax = WB_DEG(180.0f), .rangerFov = WB_DEG(27.0f), .rangerRangeMax = 4.0f, \
    .vCruise = 0.5f, .vFollow = 0.3f, .kStop = 2.0f, .kYaw = 4.0f,                   \
    .yawTol = WB_DEG(10.0f), .yawFreeRadius = 0.2f, .dSweep = 1.0f,                  \
    .sweepHalf = WB_DEG(45.0f), .sweepRate = WB_DEG(60.0f), .edgeJump = 0.5f,        \
    .passMargin = 0.4f, .tangentClearance = 0.5f, .passMax = 1.5f, .maxSweeps = 12,  \
    .maxWinding = 2.5f * PI_F, .dSide = 0.25f, .dDetect = 0.6f, .dWall = 0.3f,       \
    .dOpen = 0.6f, .dClose = 0.22f, .cornerMargin = 0.3f, .vTurn = 0.05f,            \
    .wpTol = 0.10f, .leaveMargin = 0.10f, .loopTol = 0.15f, .loopMinTravel = 1.0f,   \
    .hysteresis = 0.15f, .wallFitTravel = 0.10f, .wallFitMax = WB_DEG(30.0f),        \
    .wallFitGain = 0.5f}
static wedgebugParams_t navParams = NAV_DEFAULTS;

// ----- runtime ------------------------------------------------------------------

typedef struct {
  float ex, ey, ez, evx, evy, evz, roll, pitch, eyaw;  // EKF frame; deg for roll/pitch, rad for yaw
  float x, y, vx, vy, yaw;                             // mission frame
} navEstimate_t;

typedef struct {
  uint32_t counts[4];   // rangeGetUpdateCount() at the last frame, navigator order
  uint32_t upCount;
  uint32_t seq;         // frames since the mission started; 0 = none yet
  TickType_t tick;      // when the last frame was noticed
  float r[4];           // [front, left, back, right] [m], INFINITY = no valid return, NAN = none yet
  uint8_t upNear;       // consecutive up readings under upStop
} ranger_t;

static const rangeDirection_t NAV_DIRS[4] = {rangeFront, rangeLeft, rangeBack, rangeRight};

static wedgebug_t nav;  // ~5 kB: static, never on the task stack
static setpoint_t setpoint;
static ranger_t rg;
static struct {
  TickType_t tick[YAW_HISTORY];
  float yaw[YAW_HISTORY];
  int head, count;
} yawHistory;

// The mission frame, latched at start so parameter edits in flight cannot move it.
static struct {
  float x0, y0, psi0, height;
} frame;

static appState_t state = STATE_IDLE;
static uint8_t reason = REASON_NONE;
static TickType_t stateTick, navStartTick, settledSince, linkLostSince;
static bool settled, linkLost, armRequested;
static float zCmd;                       // last commanded height [m]
static float holdX, holdY, holdYaw;      // HOLD / LAND point, mission frame
static float landZ0;

static logVarId_t idX, idY, idZ, idVx, idVy, idVz, idRoll, idPitch, idYaw;
static paramVarId_t idReset, idFlow2, idRanger;

// Log group wedgebug
static uint8_t logState, logMode, logPhase, logWp;
static float logX, logY, logYaw;
static float cmdVx, cmdVy, cmdYawRate;
static uint16_t logSeq, logSweeps;
static uint8_t hits, reached, unreachable;
static uint8_t missionCount;  // missions started since boot: tells a host its start was seen

// ----- helpers -------------------------------------------------------------------

static float wrapPi(float a) {
  a = fmodf(a + PI_F, 2.0f * PI_F);
  if (a < 0.0f) {
    a += 2.0f * PI_F;
  }
  return a - PI_F;
}

static uint32_t msSince(TickType_t since, TickType_t now) { return T2M(now - since); }

static void setState(appState_t s, TickType_t now) {
  DEBUG_PRINT("%s -> %s\n", STATE_NAMES[state], STATE_NAMES[s]);
  state = s;
  stateTick = now;
}

static void readEstimate(navEstimate_t* e) {
  e->ex = logGetFloat(idX);
  e->ey = logGetFloat(idY);
  e->ez = logGetFloat(idZ);
  e->evx = logGetFloat(idVx);
  e->evy = logGetFloat(idVy);
  e->evz = logGetFloat(idVz);
  e->roll = logGetFloat(idRoll);
  e->pitch = logGetFloat(idPitch);
  e->eyaw = logGetFloat(idYaw) * DEG_TO_RAD_F;
  const float c = cosf(frame.psi0), s = sinf(frame.psi0);
  e->x = frame.x0 + c * e->ex - s * e->ey;
  e->y = frame.y0 + s * e->ex + c * e->ey;
  e->vx = c * e->evx - s * e->evy;
  e->vy = s * e->evx + c * e->evy;
  e->yaw = wrapPi(e->eyaw + frame.psi0);
}

static bool estimateFinite(const navEstimate_t* e) {
  return isfinite(e->ex) && isfinite(e->ey) && isfinite(e->ez) && isfinite(e->evx) && isfinite(e->evy) &&
         isfinite(e->evz) && isfinite(e->roll) && isfinite(e->pitch) && isfinite(e->eyaw);
}

// Position, height and yaw in the mission frame, sent in the EKF frame.
static void sendPosition(float x, float y, float z, float yaw) {
  const float dx = x - frame.x0, dy = y - frame.y0;
  const float c = cosf(frame.psi0), s = sinf(frame.psi0);
  memset(&setpoint, 0, sizeof(setpoint));
  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;
  setpoint.position.x = c * dx + s * dy;
  setpoint.position.y = -s * dx + c * dy;
  setpoint.position.z = z;
  setpoint.mode.yaw = modeAbs;
  setpoint.attitude.yaw = wrapPi(yaw - frame.psi0) * RAD_TO_DEG_F;
  commanderSetSetpoint(&setpoint, SETPOINT_PRIORITY);
  zCmd = z;
}

// The navigator's output: body-frame velocity, absolute height, yaw rate
// (counter-clockwise, rad/s). Same fields as the sim's hover target
// (controllers/firmware_pid.py) and the CRTP hover packet.
static void sendHover(float vxBody, float vyBody, float z, float yawRate) {
  memset(&setpoint, 0, sizeof(setpoint));
  setpoint.mode.x = modeVelocity;
  setpoint.mode.y = modeVelocity;
  setpoint.velocity.x = vxBody;
  setpoint.velocity.y = vyBody;
  setpoint.velocity_body = true;
  setpoint.mode.z = modeAbs;
  setpoint.position.z = z;
  setpoint.mode.yaw = modeVelocity;
  setpoint.attitudeRate.yaw = yawRate * RAD_TO_DEG_F;
  commanderSetSetpoint(&setpoint, SETPOINT_PRIORITY);
  zCmd = z;
}

static void stopMotors(void) {
  memset(&setpoint, 0, sizeof(setpoint));
  commanderSetSetpoint(&setpoint, SETPOINT_PRIORITY);
}

// ----- Multi-ranger ----------------------------------------------------------------

static float toMeters(float mm) {
  if (mm >= RANGER_INVALID_MM) {
    return INFINITY;  // not RANGE_VALID: on the drone, mostly nothing within range
  }
  const float m = mm / 1000.0f;
  return m > nav.params.rangerRangeMax ? INFINITY : m;
}

static void rangerReset(TickType_t now) {
  for (int i = 0; i < 4; i++) {
    rg.counts[i] = rangeGetUpdateCount(NAV_DIRS[i]);
    rg.r[i] = NAN;
  }
  rg.upCount = rangeGetUpdateCount(rangeUp);
  rg.seq = 0;
  rg.tick = now;
  rg.upNear = 0;
}

// A frame is new once all four horizontal sensors have been rewritten since
// the last one (the driver writes them one after another every 100 ms).
static void rangerPoll(TickType_t now) {
  const uint32_t up = rangeGetUpdateCount(rangeUp);
  if (up != rg.upCount) {
    rg.upCount = up;
    const bool near = upStop > 0.0f && rangeGet(rangeUp) < upStop * 1000.0f;
    rg.upNear = near ? (rg.upNear < 255 ? rg.upNear + 1 : 255) : 0;
  }
  for (int i = 0; i < 4; i++) {
    if (rangeGetUpdateCount(NAV_DIRS[i]) == rg.counts[i]) {
      return;
    }
  }
  for (int i = 0; i < 4; i++) {
    rg.counts[i] = rangeGetUpdateCount(NAV_DIRS[i]);
    rg.r[i] = toMeters(rangeGet(NAV_DIRS[i]));
  }
  rg.seq++;
  rg.tick = now;
}

static void yawPush(TickType_t now, float yaw) {
  yawHistory.head = (yawHistory.head + 1) % YAW_HISTORY;
  yawHistory.tick[yawHistory.head] = now;
  yawHistory.yaw[yawHistory.head] = yaw;
  if (yawHistory.count < YAW_HISTORY) {
    yawHistory.count++;
  }
}

// Yaw at the newest history entry not after t (the oldest if t predates them all).
static float yawAt(TickType_t t) {
  float yaw = yawHistory.yaw[yawHistory.head];
  for (int k = 0; k < yawHistory.count; k++) {
    const int i = (yawHistory.head - k + YAW_HISTORY) % YAW_HISTORY;
    yaw = yawHistory.yaw[i];
    if ((int32_t)(t - yawHistory.tick[i]) >= 0) {
      break;
    }
  }
  return yaw;
}

// ----- mission start / end -----------------------------------------------------------

static bool insideFence(float x, float y) {
  return x >= fenceXMin && x <= fenceXMax && y >= fenceYMin && y <= fenceYMax;
}

static bool missionValid(void) {
  if (nTargets < 1 || nTargets > MAX_TARGETS) {
    DEBUG_PRINT("nTargets must be 1..%d\n", MAX_TARGETS);
    return false;
  }
  if (!(height >= HEIGHT_MIN && height <= HEIGHT_MAX && height < zMax)) {
    DEBUG_PRINT("height must be %.1f..%.1f m and below zMax\n", (double)HEIGHT_MIN, (double)HEIGHT_MAX);
    return false;
  }
  if (!(isfinite(startX) && isfinite(startY) && isfinite(psi0) && maxTime > 0.0f && holdTime >= 0.0f &&
        fenceXMin < fenceXMax && fenceYMin < fenceYMax && insideFence(startX, startY))) {
    DEBUG_PRINT("start, psi0, maxTime, holdTime or fence invalid (start must be inside the fence)\n");
    return false;
  }
  for (int i = 0; i < nTargets; i++) {
    if (!(isfinite(targetX[i]) && isfinite(targetY[i]) && insideFence(targetX[i], targetY[i]))) {
      DEBUG_PRINT("target %d is not finite or outside the fence\n", i);
      return false;
    }
  }
  if (navParams.maxSweeps < 1 || navParams.maxSweeps > WEDGEBUG_MAX_SWEEP_SPOTS) {
    DEBUG_PRINT("wbnav.maxSweeps must be 1..%d\n", WEDGEBUG_MAX_SWEEP_SPOTS);
    return false;
  }
  return true;
}

static void finish(uint8_t why, TickType_t now) {
  if (state >= STATE_TAKEOFF) {
    stopMotors();
    commanderRelaxPriority();
  }
  if (armRequested) {
    supervisorRequestArming(false);  // CF2 auto-arms again by itself; brushless needs a new arm
    armRequested = false;
  }
  reason = why;
  start = 0;
  DEBUG_PRINT("mission ended, reason %d\n", why);
  setState(STATE_IDLE, now);
}

static void beginLand(uint8_t why, const navEstimate_t* e, TickType_t now) {
  reason = why;
  holdX = e->x;
  holdY = e->y;
  holdYaw = e->yaw;
  landZ0 = zCmd;
  DEBUG_PRINT("landing, reason %d\n", why);
  setState(STATE_LAND, now);
}

static void beginMission(TickType_t now) {
  // Reason first: a log sample that sees the new count never sees the old
  // reason. Every ending sets a nonzero reason, so a host reads "state 0,
  // reason 0, new count" as starting, not ended.
  reason = REASON_NONE;
  missionCount++;
  hits = reached = unreachable = 0;
  logSweeps = 0;
  if (!missionValid()) {
    finish(REASON_BAD_MISSION, now);
    return;
  }
  if (!PARAM_VARID_IS_VALID(idFlow2) || !PARAM_VARID_IS_VALID(idRanger) || !paramGetUint(idFlow2) ||
      !paramGetUint(idRanger) || stateEstimatorGetType() != StateEstimatorTypeKalman) {
    DEBUG_PRINT("needs Flow deck v2, Multi-ranger and the Kalman estimator\n");
    finish(REASON_NO_DECK, now);
    return;
  }
  if (supervisorIsLocked() || supervisorIsCrashed() || supervisorIsTumbled()) {
    DEBUG_PRINT("supervisor locked, crashed or tumbled\n");
    finish(REASON_SUPERVISOR, now);
    return;
  }

  frame.x0 = startX;
  frame.y0 = startY;
  frame.psi0 = psi0;
  frame.height = height;
  wedgebugInit(&nav, &navParams, startX, startY);
  for (int i = 0; i < nTargets; i++) {
    wedgebugAddTarget(&nav, targetX[i], targetY[i]);
  }
  zCmd = 0.0f;
  armRequested = false;
  linkLost = false;
  paramSetInt(idReset, 1);  // the Kalman task resets on its next cycle
  rangerReset(now);
  DEBUG_PRINT("start (%.2f, %.2f) heading %.0f deg, %d target(s), height %.2f m\n", (double)startX,
              (double)startY, (double)(psi0 * RAD_TO_DEG_F), nTargets, (double)height);
  setState(STATE_PREFLIGHT, now);
}

// ----- navigation ------------------------------------------------------------------------

static void navStep(const navEstimate_t* e, TickType_t now, wedgebugOutput_t* out) {
  wedgebugInput_t in;
  memset(&in, 0, sizeof(in));
  in.t = (float)msSince(navStartTick, now) / 1000.0f;
  in.x = e->x;
  in.y = e->y;
  in.vx = e->vx;
  in.vy = e->vy;
  in.yaw = e->yaw;
  in.rangeFront = rg.r[0];
  in.rangeLeft = rg.r[1];
  in.rangeBack = rg.r[2];
  in.rangeRight = rg.r[3];
  in.timestampedRanges = tsRanges != 0;
  if (in.timestampedRanges) {
    const TickType_t sampleTick = rg.tick - M2T((uint32_t)(latency * 1000.0f));
    in.rangerSequence = rg.seq;
    in.rangerSampleTime = (float)(int32_t)(sampleTick - navStartTick) / 1000.0f;
    in.rangerSampleYaw = yawAt(sampleTick);
    in.rawFront = rg.r[0];
    in.rawLeft = rg.r[1];
    in.rawRight = rg.r[3];
  }

  const int sweepsBefore = nav.sweepCount;
  wedgebugStep(&nav, &in, out);

  for (int k = 0; k < nav.stepEventCount; k++) {
    wedgebugEvent_t ev;
    wedgebugGetStepEvent(&nav, k, &ev);
    switch (ev.type) {
      case WEDGEBUG_EVENT_HIT:
        hits++;
        DEBUG_PRINT("hit obstacle at (%.2f, %.2f), target %d\n", (double)ev.x, (double)ev.y, ev.wp);
        break;
      case WEDGEBUG_EVENT_LEAVE:
        DEBUG_PRINT("left wall at (%.2f, %.2f)\n", (double)ev.x, (double)ev.y);
        break;
      case WEDGEBUG_EVENT_REACHED:
        reached++;
        DEBUG_PRINT("reached target %d at (%.2f, %.2f), t %.1f s\n", ev.wp, (double)ev.x, (double)ev.y,
                    (double)ev.t);
        break;
      case WEDGEBUG_EVENT_UNREACHABLE:
        unreachable++;
        DEBUG_PRINT("target %d unreachable, gave up at (%.2f, %.2f)\n", ev.wp, (double)ev.x, (double)ev.y);
        break;
      default:
        break;
    }
  }
  if (nav.sweepCount != sweepsBefore) {
    const wedgebugSweep_t* s = &nav.lastSweep;
    DEBUG_PRINT("sweep %d: round the %s\n", nav.sweepCount, s->side > 0 ? "left" : "right");
  }
}

// ----- state machine -----------------------------------------------------------------------

// Guards while the motors run (TAKEOFF..HOLD): the first that trips starts a landing.
static void guards(const navEstimate_t* e, TickType_t now) {
  if (linkLoss > 0.0f && !crtpIsConnected()) {
    if (!linkLost) {
      linkLost = true;
      linkLostSince = now;
    }
  } else {
    linkLost = false;
  }
  const bool navigating = state == STATE_NAVIGATE || state == STATE_ARRIVE;
  uint8_t why = REASON_NONE;
  if (start == 0) {
    why = REASON_ABORT;
  } else if (linkLost && msSince(linkLostSince, now) > (uint32_t)(linkLoss * 1000.0f)) {
    why = REASON_LINK;
  } else if (!insideFence(e->x, e->y) || e->ez > zMax) {
    why = REASON_FENCE;
  } else if (rg.upNear >= UP_STOP_FRAMES) {
    why = REASON_UP_STOP;
  } else if (pmIsBatteryLow()) {
    why = REASON_BATTERY;
  } else if (navigating && msSince(navStartTick, now) > (uint32_t)(maxTime * 1000.0f)) {
    why = REASON_TIMEOUT;
  } else if (navigating && msSince(rg.tick, now) > RANGER_STALE_MS) {
    why = REASON_RANGER_STALE;
  }
  if (why != REASON_NONE) {
    beginLand(why, e, now);
  }
}

static void step(TickType_t now) {
  navEstimate_t e;
  readEstimate(&e);
  yawPush(now, e.yaw);
  rangerPoll(now);

  if (state >= STATE_TAKEOFF) {
    // The supervisor has already cut the motors; stop sending setpoints.
    if (supervisorIsTumbled() || supervisorIsCrashed() || supervisorIsLocked()) {
      finish(REASON_SUPERVISOR, now);
      return;
    }
    if (!estimateFinite(&e)) {
      finish(REASON_NONFINITE, now);
      return;
    }
  }
  if (state >= STATE_TAKEOFF && state <= STATE_HOLD) {
    guards(&e, now);  // a landing it starts is flown below, this tick
  }

  switch (state) {
    case STATE_IDLE:
      if (start) {
        beginMission(now);
      }
      break;

    case STATE_PREFLIGHT: {
      if (start == 0) {
        finish(REASON_ABORT, now);
        break;
      }
      const uint32_t t = msSince(stateTick, now);
      if (t < EKF_SETTLE_MS) {
        break;
      }
      if (!armRequested) {  // checks once, then wait for the supervisor
        // A reset EKF sits at x = y = 0, yaw = 0 (kalman.initial* must be 0).
        if (!estimateFinite(&e) || fabsf(e.roll) > PREFLIGHT_MAX_TILT_DEG ||
            fabsf(e.pitch) > PREFLIGHT_MAX_TILT_DEG || e.ez > PREFLIGHT_MAX_Z ||
            fabsf(e.ex) > PREFLIGHT_MAX_XY || fabsf(e.ey) > PREFLIGHT_MAX_XY ||
            fabsf(e.eyaw) > PREFLIGHT_MAX_YAW) {
          DEBUG_PRINT("preflight failed: tilt %.0f/%.0f deg, z %.2f m, xy %.2f/%.2f m, yaw %.0f deg\n",
                      (double)e.roll, (double)e.pitch, (double)e.ez, (double)e.ex, (double)e.ey,
                      (double)(e.eyaw * RAD_TO_DEG_F));
          finish(REASON_PREFLIGHT, now);
          break;
        }
        if (rg.seq < RANGER_MIN_FRAMES) {
          DEBUG_PRINT("no Multi-ranger frames\n");
          finish(REASON_NO_RANGER, now);
          break;
        }
        armRequested = true;
        supervisorRequestArming(true);
      }
      if (supervisorCanFly()) {
        settled = false;
        setState(STATE_TAKEOFF, now);
        // fall into TAKEOFF on the next tick
      } else if (t > EKF_SETTLE_MS + ARM_TIMEOUT_MS) {
        DEBUG_PRINT("supervisor did not allow flight\n");
        finish(REASON_NOT_ARMED, now);
      }
      break;
    }

    case STATE_TAKEOFF: {
      const float z = fminf(frame.height, VERTICAL_SPEED * (float)msSince(stateTick, now) / 1000.0f);
      sendPosition(frame.x0, frame.y0, z, frame.psi0);
      if (z >= frame.height) {
        setState(STATE_SETTLE, now);
      }
      break;
    }

    case STATE_SETTLE: {
      sendPosition(frame.x0, frame.y0, frame.height, frame.psi0);
      const float err = sqrtf((e.x - frame.x0) * (e.x - frame.x0) + (e.y - frame.y0) * (e.y - frame.y0) +
                              (e.ez - frame.height) * (e.ez - frame.height));
      const float speed = sqrtf(e.evx * e.evx + e.evy * e.evy + e.evz * e.evz);
      if (err < SETTLE_POS_TOL && speed < SETTLE_SPEED_TOL) {
        if (!settled) {
          settled = true;
          settledSince = now;
        } else if (msSince(settledSince, now) >= SETTLE_HOLD_MS) {
          navStartTick = now;
          setState(STATE_NAVIGATE, now);
        }
      } else {
        settled = false;
        if (msSince(stateTick, now) > SETTLE_TIMEOUT_MS) {
          beginLand(REASON_SETTLE, &e, now);
        }
      }
      break;
    }

    case STATE_NAVIGATE: {
      wedgebugOutput_t out;
      navStep(&e, now, &out);
      if (!(isfinite(out.vxBody) && isfinite(out.vyBody) && isfinite(out.yawRate))) {
        beginLand(REASON_NONFINITE, &e, now);
        break;
      }
      const float speed = sqrtf(out.vxBody * out.vxBody + out.vyBody * out.vyBody);
      const float scale = speed > V_LIMIT ? V_LIMIT / speed : 1.0f;
      cmdVx = out.vxBody * scale;
      cmdVy = out.vyBody * scale;
      cmdYawRate = fmaxf(-YAW_RATE_LIMIT, fminf(YAW_RATE_LIMIT, out.yawRate));
      sendHover(cmdVx, cmdVy, frame.height, cmdYawRate);
      if (nav.mode == WEDGEBUG_DONE) {
        DEBUG_PRINT("navigation done in %.1f s: %d reached, %d unreachable\n",
                    (double)((float)msSince(navStartTick, now) / 1000.0f), reached, unreachable);
        setState(STATE_ARRIVE, now);
      }
      break;
    }

    case STATE_ARRIVE:
      cmdVx = cmdVy = cmdYawRate = 0.0f;
      sendHover(0.0f, 0.0f, frame.height, 0.0f);
      if (sqrtf(e.vx * e.vx + e.vy * e.vy) < ARRIVE_SPEED || msSince(stateTick, now) > ARRIVE_TIMEOUT_MS) {
        holdX = e.x;
        holdY = e.y;
        holdYaw = e.yaw;
        setState(STATE_HOLD, now);
      }
      break;

    case STATE_HOLD:
      sendPosition(holdX, holdY, frame.height, holdYaw);
      if (msSince(stateTick, now) >= (uint32_t)(holdTime * 1000.0f)) {
        beginLand(REASON_DONE, &e, now);
      }
      break;

    case STATE_LAND: {
      const uint32_t t = msSince(stateTick, now);
      const float z = fmaxf(0.0f, landZ0 - VERTICAL_SPEED * (float)t / 1000.0f);
      sendPosition(holdX, holdY, z, holdYaw);
      if (z <= 0.0f && t >= (uint32_t)(landZ0 / VERTICAL_SPEED * 1000.0f) + LAND_TAIL_MS) {
        finish(reason, now);
      }
      break;
    }
  }

  logState = (uint8_t)state;
  logMode = state == STATE_IDLE ? 0 : (uint8_t)nav.mode;
  logPhase = (uint8_t)nav.phase;
  logWp = (uint8_t)nav.wp;
  logX = e.x;
  logY = e.y;
  logYaw = e.yaw;
  logSeq = (uint16_t)rg.seq;
  logSweeps = (uint16_t)nav.sweepCount;
  if (state != STATE_NAVIGATE) {
    cmdVx = cmdVy = cmdYawRate = 0.0f;
  }
}

void appMain(void) {
  idX = logGetVarId("stateEstimate", "x");
  idY = logGetVarId("stateEstimate", "y");
  idZ = logGetVarId("stateEstimate", "z");
  idVx = logGetVarId("stateEstimate", "vx");
  idVy = logGetVarId("stateEstimate", "vy");
  idVz = logGetVarId("stateEstimate", "vz");
  idRoll = logGetVarId("stateEstimate", "roll");
  idPitch = logGetVarId("stateEstimate", "pitch");
  idYaw = logGetVarId("stateEstimate", "yaw");
  idReset = paramGetVarId("kalman", "resetEstimation");
  idFlow2 = paramGetVarId("deck", "bcFlow2");
  idRanger = paramGetVarId("deck", "bcMultiranger");
  wedgebugParams_t core;
  const wedgebugParams_t mine = NAV_DEFAULTS;
  wedgebugDefaultParams(&core);
  if (memcmp(&core, &mine, sizeof(core)) != 0) {
    DEBUG_PRINT("WARNING: NAV_DEFAULTS differ from wedgebugDefaultParams(), update wedgebug_app.c\n");
  }
  frame.x0 = startX;
  frame.y0 = startY;
  frame.psi0 = psi0;
  frame.height = height;
  wedgebugInit(&nav, &navParams, startX, startY);  // so the logs read sensibly before the first mission
  rangerReset(xTaskGetTickCount());
  DEBUG_PRINT("WedgeBug app ready: set wedgebug.start = 1 to fly\n");

  TickType_t lastWake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&lastWake, M2T(TICK_MS));
    step(xTaskGetTickCount());
  }
}

/**
 * WedgeBug mission: targets, frame, flight height and guards.
 * Takes effect at the next start (fence and guards: immediately).
 */
PARAM_GROUP_START(wedgebug)
/** @brief 1 = take off and fly the mission; 0 = land now. Reset to 0 when a mission ends */
PARAM_ADD(PARAM_UINT8, start, &start)
/** @brief Number of targets (waypoints then goal), 1..8 */
PARAM_ADD(PARAM_UINT8, nTargets, &nTargets)
PARAM_ADD(PARAM_FLOAT, x0, &targetX[0])
PARAM_ADD(PARAM_FLOAT, y0, &targetY[0])
PARAM_ADD(PARAM_FLOAT, x1, &targetX[1])
PARAM_ADD(PARAM_FLOAT, y1, &targetY[1])
PARAM_ADD(PARAM_FLOAT, x2, &targetX[2])
PARAM_ADD(PARAM_FLOAT, y2, &targetY[2])
PARAM_ADD(PARAM_FLOAT, x3, &targetX[3])
PARAM_ADD(PARAM_FLOAT, y3, &targetY[3])
PARAM_ADD(PARAM_FLOAT, x4, &targetX[4])
PARAM_ADD(PARAM_FLOAT, y4, &targetY[4])
PARAM_ADD(PARAM_FLOAT, x5, &targetX[5])
PARAM_ADD(PARAM_FLOAT, y5, &targetY[5])
PARAM_ADD(PARAM_FLOAT, x6, &targetX[6])
PARAM_ADD(PARAM_FLOAT, y6, &targetY[6])
PARAM_ADD(PARAM_FLOAT, x7, &targetX[7])
PARAM_ADD(PARAM_FLOAT, y7, &targetY[7])
/** @brief Takeoff point in the mission frame [m] */
PARAM_ADD(PARAM_FLOAT, startX, &startX)
PARAM_ADD(PARAM_FLOAT, startY, &startY)
/** @brief Takeoff heading, counter-clockwise from the mission +x [rad] */
PARAM_ADD(PARAM_FLOAT, psi0, &psi0)
/** @brief Flight height, 0.2..1.0 [m] */
PARAM_ADD(PARAM_FLOAT, height, &height)
/** @brief Navigation time limit [s] */
PARAM_ADD(PARAM_FLOAT, maxTime, &maxTime)
/** @brief Hover at the goal this long before landing [s] */
PARAM_ADD(PARAM_FLOAT, holdTime, &holdTime)
/** @brief Fence box in the mission frame [m]: leaving it lands */
PARAM_ADD(PARAM_FLOAT, fenceXMin, &fenceXMin)
PARAM_ADD(PARAM_FLOAT, fenceXMax, &fenceXMax)
PARAM_ADD(PARAM_FLOAT, fenceYMin, &fenceYMin)
PARAM_ADD(PARAM_FLOAT, fenceYMax, &fenceYMax)
/** @brief Estimated height above this lands [m] */
PARAM_ADD(PARAM_FLOAT, zMax, &zMax)
/** @brief Land after the radio link is lost this long [s]; 0 = keep flying */
PARAM_ADD(PARAM_FLOAT, linkLoss, &linkLoss)
/** @brief An up-sensor reading under this for 0.3 s lands (hand over the drone) [m]; 0 = off */
PARAM_ADD(PARAM_FLOAT, upStop, &upStop)
/** @brief Multi-ranger measurement-to-delivery delay used to pair frames with yaw [s] */
PARAM_ADD(PARAM_FLOAT, latency, &latency)
/** @brief 1 = timestamped ranger frames (as the sim missions), 0 = baseline navigator */
PARAM_ADD(PARAM_UINT8, tsRanges, &tsRanges)
PARAM_GROUP_STOP(wedgebug)

/**
 * WedgeBug navigator settings, same names as wedgebugParams_t and
 * sim/navigation/params.py (p.nav). Copied into the navigator at each start.
 */
PARAM_GROUP_START(wbnav)
PARAM_ADD(PARAM_FLOAT, yawRateMax, &navParams.yawRateMax)
PARAM_ADD(PARAM_FLOAT, rangerFov, &navParams.rangerFov)
PARAM_ADD(PARAM_FLOAT, rangerRangeMax, &navParams.rangerRangeMax)
PARAM_ADD(PARAM_FLOAT, vCruise, &navParams.vCruise)
PARAM_ADD(PARAM_FLOAT, vFollow, &navParams.vFollow)
PARAM_ADD(PARAM_FLOAT, kStop, &navParams.kStop)
PARAM_ADD(PARAM_FLOAT, kYaw, &navParams.kYaw)
PARAM_ADD(PARAM_FLOAT, yawTol, &navParams.yawTol)
PARAM_ADD(PARAM_FLOAT, yawFreeRadius, &navParams.yawFreeRadius)
PARAM_ADD(PARAM_FLOAT, dSweep, &navParams.dSweep)
PARAM_ADD(PARAM_FLOAT, sweepHalf, &navParams.sweepHalf)
PARAM_ADD(PARAM_FLOAT, sweepRate, &navParams.sweepRate)
PARAM_ADD(PARAM_FLOAT, edgeJump, &navParams.edgeJump)
PARAM_ADD(PARAM_FLOAT, passMargin, &navParams.passMargin)
PARAM_ADD(PARAM_FLOAT, tangentClearance, &navParams.tangentClearance)
PARAM_ADD(PARAM_FLOAT, passMax, &navParams.passMax)
PARAM_ADD(PARAM_INT32, maxSweeps, &navParams.maxSweeps)
PARAM_ADD(PARAM_FLOAT, maxWinding, &navParams.maxWinding)
PARAM_ADD(PARAM_FLOAT, dSide, &navParams.dSide)
PARAM_ADD(PARAM_FLOAT, dDetect, &navParams.dDetect)
PARAM_ADD(PARAM_FLOAT, dWall, &navParams.dWall)
PARAM_ADD(PARAM_FLOAT, dOpen, &navParams.dOpen)
PARAM_ADD(PARAM_FLOAT, dClose, &navParams.dClose)
PARAM_ADD(PARAM_FLOAT, cornerMargin, &navParams.cornerMargin)
PARAM_ADD(PARAM_FLOAT, vTurn, &navParams.vTurn)
PARAM_ADD(PARAM_FLOAT, wpTol, &navParams.wpTol)
PARAM_ADD(PARAM_FLOAT, leaveMargin, &navParams.leaveMargin)
PARAM_ADD(PARAM_FLOAT, loopTol, &navParams.loopTol)
PARAM_ADD(PARAM_FLOAT, loopMinTravel, &navParams.loopMinTravel)
PARAM_ADD(PARAM_FLOAT, hysteresis, &navParams.hysteresis)
PARAM_ADD(PARAM_FLOAT, wallFitTravel, &navParams.wallFitTravel)
PARAM_ADD(PARAM_FLOAT, wallFitMax, &navParams.wallFitMax)
PARAM_ADD(PARAM_FLOAT, wallFitGain, &navParams.wallFitGain)
PARAM_GROUP_STOP(wbnav)

/**
 * WedgeBug mission state. Positions and yaw in the mission frame.
 */
LOG_GROUP_START(wedgebug)
/** @brief 0 IDLE, 1 PREFLIGHT, 2 TAKEOFF, 3 SETTLE, 4 NAVIGATE, 5 ARRIVE, 6 HOLD, 7 LAND */
LOG_ADD(LOG_UINT8, state, &logState)
/** @brief Why the last mission ended or is landing, see appReason_t */
LOG_ADD(LOG_UINT8, reason, &reason)
/** @brief Navigator mode: 1 GO_TO_GOAL, 2 AVOID, 3 DONE (0 idle) */
LOG_ADD(LOG_UINT8, mode, &logMode)
/** @brief Navigator phase: 0 none, 1 sweep, 2 pass, 3 align, 4 follow, 5 overshoot */
LOG_ADD(LOG_UINT8, phase, &logPhase)
/** @brief Current target index, 0-based */
LOG_ADD(LOG_UINT8, wp, &logWp)
LOG_ADD(LOG_FLOAT, x, &logX)
LOG_ADD(LOG_FLOAT, y, &logY)
/** @brief [rad] */
LOG_ADD(LOG_FLOAT, yaw, &logYaw)
/** @brief Commanded body-frame velocity [m/s] and yaw rate [rad/s] */
LOG_ADD(LOG_FLOAT, cmdVx, &cmdVx)
LOG_ADD(LOG_FLOAT, cmdVy, &cmdVy)
LOG_ADD(LOG_FLOAT, cmdYawRate, &cmdYawRate)
/** @brief Navigator's ranges [m], front/left/back/right: inf = no valid return, nan = none yet */
LOG_ADD(LOG_FLOAT, rF, &rg.r[0])
LOG_ADD(LOG_FLOAT, rL, &rg.r[1])
LOG_ADD(LOG_FLOAT, rB, &rg.r[2])
LOG_ADD(LOG_FLOAT, rR, &rg.r[3])
/** @brief Multi-ranger frames seen this mission (wraps) */
LOG_ADD(LOG_UINT16, seq, &logSeq)
LOG_ADD(LOG_UINT16, sweeps, &logSweeps)
LOG_ADD(LOG_UINT8, hits, &hits)
LOG_ADD(LOG_UINT8, reached, &reached)
LOG_ADD(LOG_UINT8, unreachable, &unreachable)
/** @brief Missions started since boot (wraps): changes when a start is taken up */
LOG_ADD(LOG_UINT8, count, &missionCount)
LOG_GROUP_STOP(wedgebug)

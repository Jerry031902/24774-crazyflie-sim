/**
 * WedgeBug reactive navigation for a Crazyflie with a Multi-ranger deck.
 *
 * Port of matlab-trajectory-sim/src/nav/navigator_step.m. The drone flies
 * straight at each target; when an obstacle blocks the way it stops,
 * sweeps its rangers +/-45 deg to find where the obstacle ends, and goes
 * round the shorter side (wall-following when the edge is out of view).
 *
 * This file is the platform-independent core: no FreeRTOS, no drivers, no
 * dynamic memory, single-precision math. Call wedgebugStep() at a fixed
 * rate (50 Hz in the reference design) with the state estimate and the four
 * horizontal range readings, and send the output as a hover setpoint:
 * body-frame vx/vy, a yaw rate, and an absolute height held by the caller.
 *
 * Units: m, m/s, rad, rad/s. World frame Z-up; yaw counter-clockwise from +x.
 * A range of INFINITY means "no return within rangerRangeMax".
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WEDGEBUG_MAX_TARGETS 16
#define WEDGEBUG_MAX_SCAN 600         // sweep readings: 3 per tick, 200 ticks
#define WEDGEBUG_MAX_SWEEP_SPOTS 16   // upper bound for maxSweeps
#define WEDGEBUG_MAX_STEP_EVENTS 4

typedef enum {
  WEDGEBUG_GO_TO_GOAL = 1,
  WEDGEBUG_AVOID = 2,
  WEDGEBUG_DONE = 3,
} wedgebugMode_t;

typedef enum {
  WEDGEBUG_PHASE_NONE = 0,
  WEDGEBUG_PHASE_SWEEP = 1,
  WEDGEBUG_PHASE_PASS = 2,
  WEDGEBUG_PHASE_ALIGN = 3,
  WEDGEBUG_PHASE_FOLLOW = 4,
  WEDGEBUG_PHASE_OVERSHOOT = 5,
} wedgebugPhase_t;

typedef enum {
  WEDGEBUG_SWEEP_STOP = 0,
  WEDGEBUG_SWEEP_TO_START = 1,
  WEDGEBUG_SWEEP_SCAN = 2,
} wedgebugSweepStage_t;

typedef enum {
  WEDGEBUG_EVENT_HIT = 1,
  WEDGEBUG_EVENT_LEAVE = 2,
  WEDGEBUG_EVENT_REACHED = 3,
  WEDGEBUG_EVENT_UNREACHABLE = 4,
} wedgebugEventType_t;

// Same names and defaults as matlab-trajectory-sim/src/config/default_params.m.
typedef struct {
  float yawRateMax;        // [rad/s]
  float rangerFov;         // ranger cone width [rad]
  float rangerRangeMax;    // [m]
  float vCruise;           // GO_TO_GOAL speed [m/s]
  float vFollow;           // wall-following speed [m/s]
  float kStop;             // speed = kStop * distance to the next stop point [1/s]
  float kYaw;              // yaw rate = kYaw * heading error [1/s]
  float yawTol;            // heading error above which the drone stops and turns [rad]
  float yawFreeRadius;     // within this of a target, slide onto it without turning [m]
  float dSweep;            // stop this far from an obstacle ahead and sweep [m]
  float sweepHalf;         // sweep +/- this about the blocked heading [rad]
  float sweepRate;         // yaw rate while sweeping [rad/s]
  float edgeJump;          // a range jump this big marks the end of an outline [m]
  float passMargin;        // fly at least this far outside a seen edge [m]
  float tangentClearance;  // the next leg keeps this from the obstacle [m]
  float passMax;           // never more than this far outside the edge [m]
  int maxSweeps;           // sweeps on one obstacle before giving up (<= WEDGEBUG_MAX_SWEEP_SPOTS)
  float maxWinding;        // circling the target this far: unreachable [rad]
  float dSide;             // side reading that switches GO_TO_GOAL to wall-following [m]
  float dDetect;           // wall-following: a way counts as clear beyond this [m]
  float dWall;             // stop this far from whatever is ahead [m]
  float dOpen;             // wall-side reading above this: the wall has ended [m]
  float dClose;            // wall-side reading below this: too close, turn away [m]
  float cornerMargin;      // extra travel past a wall's end before turning back [m]
  float vTurn;             // brake below this speed before changing direction [m/s]
  float wpTol;             // a target counts as reached within this [m]
  float leaveMargin;       // leave the wall this much closer than the hit point [m]
  float loopTol;           // back within this of where wall-following began... [m]
  float loopMinTravel;     // ...after following this far = full loop [m]
} wedgebugParams_t;

typedef struct {
  float t;                 // time [s]
  float x, y;              // position estimate, world [m]
  float vx, vy;            // velocity estimate, world [m/s]
  float yaw;               // [rad]
  float rangeFront, rangeLeft, rangeBack, rangeRight;  // [m], INFINITY = no return
} wedgebugInput_t;

typedef struct {
  float vxBody;            // forward [m/s]
  float vyBody;            // left [m/s]
  float yawRate;           // counter-clockwise [rad/s]
} wedgebugOutput_t;

typedef struct {
  int type;                // wedgebugEventType_t
  int wp;                  // target index (0-based) at the time of the event
  float t, x, y;
} wedgebugEvent_t;

// Summary of the most recent completed sweep (left = round the left side).
typedef struct {
  float x, y, center;
  float edgeLeftX, edgeLeftY, edgeRightX, edgeRightY;
  bool seenLeft, seenRight;
  float costLeft, costRight;
  int side;                // +1 round the left, -1 round the right
  float passX, passY;      // NAN when following the wall instead
} wedgebugSweep_t;

typedef struct {
  wedgebugParams_t params;

  float targets[WEDGEBUG_MAX_TARGETS][2];
  int targetCount;
  int wp;                  // index of the current target
  int mode;                // wedgebugMode_t
  int phase;               // wedgebugPhase_t

  // Motion. Directions: 1 forward, 2 left, 3 back, 4 right; 0 straight at the point.
  bool facing;
  int execDir;
  float lastPos[2];

  // Avoidance episode
  bool episode;
  float hitPoint[2];
  float hitDist;
  int side;
  float epTravel;
  float sweepSpots[WEDGEBUG_MAX_SWEEP_SPOTS][3];   // x, y, epTravel
  int sweepSpotCount;
  float winding;
  float lastBearing;

  // Sweep. The scan is sorted by angle in place once the sweep ends.
  int sweepStage;          // wedgebugSweepStage_t
  float sweepCenter;
  float scanAng[WEDGEBUG_MAX_SCAN];
  float scanRng[WEDGEBUG_MAX_SCAN];
  int scanCount;
  float subgoal[2];

  // Wall-following
  int hand;                // +1 wall on the left, -1 wall on the right
  int travelDir;
  float psiHold;
  bool following;
  float loopPoint[2];
  float travel;
  float sinceTurn;
  bool wallSeen;
  float overshootLeft;
  float dFollowed;

  // Diagnostics: events raised by the last step, and the last sweep.
  wedgebugEvent_t stepEvents[WEDGEBUG_MAX_STEP_EVENTS];
  int stepEventCount;
  wedgebugSweep_t lastSweep;
  int sweepCount;
} wedgebug_t;

void wedgebugDefaultParams(wedgebugParams_t* params);
void wedgebugInit(wedgebug_t* nav, const wedgebugParams_t* params, float startX, float startY);
// Append a target (waypoints in order, then the goal). False when full.
bool wedgebugAddTarget(wedgebug_t* nav, float x, float y);
void wedgebugStep(wedgebug_t* nav, const wedgebugInput_t* in, wedgebugOutput_t* out);
// Copy event i (0 <= i < stepEventCount) of the last step. False if out of range.
bool wedgebugGetStepEvent(const wedgebug_t* nav, int i, wedgebugEvent_t* out);

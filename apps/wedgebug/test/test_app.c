/**
 * Host test of the WedgeBug flight app before it goes on the drone.
 *
 * src/wedgebug_app.c, src/wedgebug.c and the firmware's src/modules/src/range.c
 * are compiled unchanged against stubs/ and flown in a 2-D world:
 *
 *   - a kinematic drone that follows the setpoints the app sends (position or
 *     body velocity, height, absolute yaw or yaw rate) through first-order lags,
 *     standing in for the PID; the state estimate is the true state, no drift
 *   - an EKF frame that restarts at the drone's pose when the app resets it
 *   - a Multi-ranger: 27 deg cones of 9 rays, 4 m, whole millimetres plus 2 mm
 *     noise, 32767 for nothing in range, measured every 100 ms and written
 *     100 ms later one direction after another, in multiranger.c's order
 *
 * Each scenario runs in its own process (the app keeps static state) and
 * checks how the mission ended. This exercises the app's state machine, frame
 * mapping, ranger-frame detection and guards. It says nothing about the real
 * PID, Flow-deck drift or the real sensor.
 *
 *   make -C apps/wedgebug/test          build and run every scenario
 *   apps/wedgebug/test/build/test_app -v obstacle   one scenario, with the app's console
 */
#define _GNU_SOURCE
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "range.h"
#include "stubs.h"

#define PI_F 3.14159265358979f
#define DT 0.001f
#define RADIUS 0.07f            // body radius incl. propellers, sim/navigation/params.py [m]
#define TIME_LIMIT_MS 600000

// Reasons, as in wedgebug_app.c
enum {
  R_NONE, R_DONE, R_ABORT, R_BAD_MISSION, R_NO_DECK, R_PREFLIGHT, R_NO_RANGER, R_NOT_ARMED, R_SETTLE,
  R_TIMEOUT, R_FENCE, R_LINK, R_RANGER_STALE, R_UP_STOP, R_SUPERVISOR, R_NONFINITE, R_BATTERY,
};

static bool verbose;

// ----- log / param registry ---------------------------------------------------------

typedef struct {
  int kind;
  const char* file;
  int line;
  const char* name;
  int type;
  void* address;
  const char* group;
} entry_t;

static entry_t entries[256];
static int entryCount;
static bool resolved;

void testRegister(int kind, const char* file, int line, const char* name, int type, void* address) {
  if (entryCount < 256) {
    entries[entryCount++] = (entry_t){kind, file, line, name, type, address, NULL};
  }
}

static int byFileLine(const void* a, const void* b) {
  const entry_t *x = a, *y = b;
  const int c = strcmp(x->file, y->file);
  return c ? c : x->line - y->line;
}

static entry_t* find(int kind, const char* group, const char* name) {
  if (!resolved) {  // each entry belongs to the group started above it in its file
    qsort(entries, entryCount, sizeof(entry_t), byFileLine);
    const char *file = "", *paramGroup = NULL, *logGroup = NULL;
    for (int i = 0; i < entryCount; i++) {
      entry_t* e = &entries[i];
      if (strcmp(e->file, file) != 0) {
        file = e->file;
        paramGroup = logGroup = NULL;
      }
      if (e->kind == TEST_PARAM_GROUP) {
        paramGroup = e->name;
      } else if (e->kind == TEST_LOG_GROUP) {
        logGroup = e->name;
      } else {
        e->group = e->kind == TEST_PARAM ? paramGroup : logGroup;
      }
    }
    resolved = true;
  }
  for (int i = 0; i < entryCount; i++) {
    entry_t* e = &entries[i];
    if (e->kind == kind && e->group && !strcmp(e->group, group) && !strcmp(e->name, name)) {
      return e;
    }
  }
  fprintf(stderr, "no %s %s.%s\n", kind == TEST_PARAM ? "param" : "log", group, name);
  exit(2);
}

static void setParam(const char* group, const char* name, double v) {
  entry_t* e = find(TEST_PARAM, group, name);
  switch (e->type) {
    case PARAM_UINT8: *(uint8_t*)e->address = (uint8_t)v; break;
    case PARAM_INT32: *(int32_t*)e->address = (int32_t)v; break;
    case PARAM_FLOAT: *(float*)e->address = (float)v; break;
    default: fprintf(stderr, "param type %d\n", e->type); exit(2);
  }
}

static double getLog(const char* group, const char* name) {
  entry_t* e = find(TEST_LOG, group, name);
  switch (e->type) {
    case LOG_UINT8: return *(uint8_t*)e->address;
    case LOG_UINT16: return *(uint16_t*)e->address;
    case LOG_FLOAT: return *(float*)e->address;
    default: fprintf(stderr, "log type %d\n", e->type); exit(2);
  }
}

// ----- world ---------------------------------------------------------------------------

typedef struct {
  float ax, ay, bx, by;
} seg_t;

static seg_t segs[256];
static int segCount;

static void addPolygon(const float (*v)[2], int n) {
  for (int i = 0; i < n; i++) {
    const int j = (i + 1) % n;
    segs[segCount++] = (seg_t){v[i][0], v[i][1], v[j][0], v[j][1]};
  }
}

static void addBox(float cx, float cy, float w, float h) {
  const float v[4][2] = {{cx - w / 2, cy - h / 2}, {cx + w / 2, cy - h / 2}, {cx + w / 2, cy + h / 2},
                         {cx - w / 2, cy + h / 2}};
  addPolygon(v, 4);
}

static void addCircle(float cx, float cy, float r) {
  float v[24][2];
  const float rr = r / cosf(PI_F / 24);  // circumscribing, as geometry.obstacle_circle
  for (int i = 0; i < 24; i++) {
    v[i][0] = cx + rr * cosf(2 * PI_F * i / 24);
    v[i][1] = cy + rr * sinf(2 * PI_F * i / 24);
  }
  addPolygon(v, 24);
}

static float clearance(float x, float y) {
  float best = INFINITY;
  for (int i = 0; i < segCount; i++) {
    const seg_t* s = &segs[i];
    const float ex = s->bx - s->ax, ey = s->by - s->ay;
    float u = ((x - s->ax) * ex + (y - s->ay) * ey) / (ex * ex + ey * ey);
    u = fmaxf(0.0f, fminf(1.0f, u));
    best = fminf(best, hypotf(x - (s->ax + u * ex), y - (s->ay + u * ey)));
  }
  return best;
}

static float castRay(float ox, float oy, float ang) {
  const float dx = cosf(ang), dy = sinf(ang);
  float best = INFINITY;
  for (int i = 0; i < segCount; i++) {
    const seg_t* s = &segs[i];
    const float ex = s->bx - s->ax, ey = s->by - s->ay;
    const float den = ex * dy - ey * dx;
    if (fabsf(den) < 1e-9f) {
      continue;
    }
    // o + t d = a + u e, with w = a - o
    const float wx = s->ax - ox, wy = s->ay - oy;
    const float t = (ex * wy - ey * wx) / den;
    const float u = (dx * wy - dy * wx) / den;
    if (t >= 0 && u >= 0 && u <= 1) {
      best = fminf(best, t);
    }
  }
  return best;
}

// ----- simulated drone and firmware --------------------------------------------------------

static struct {
  float x, y, z, vx, vy, vz, yaw, yawRate;
} drone;
static struct {
  float x, y, yaw;
} ekf;  // world pose of the EKF frame's origin
static TickType_t now;
static setpoint_t sp;
static int spPriority;
static bool spValid;
static TickType_t spTick;
static bool armed, linkUp = true, batteryLow, rangerOn = true, flowDeck = true, rangerDeck = true;
static float upOverrideMm;  // 0: nothing above
static float minClearance = INFINITY, maxZ;
static bool everFlew;
static jmp_buf finished;

static float wrap(float a) {
  a = fmodf(a + PI_F, 2 * PI_F);
  return (a < 0 ? a + 2 * PI_F : a) - PI_F;
}
static float clampf(float v, float lo, float hi) { return fmaxf(lo, fminf(hi, v)); }

TickType_t xTaskGetTickCount(void) { return now; }

void commanderSetSetpoint(setpoint_t* setpoint, int priority) {
  if (priority >= spPriority) {
    sp = *setpoint;
    spPriority = priority;
    spValid = true;
    spTick = now;
  }
}
void commanderRelaxPriority(void) { spPriority = COMMANDER_PRIORITY_LOWEST; }

bool supervisorRequestArming(const bool doArm) {
  armed = doArm;
  return true;
}
bool supervisorCanFly(void) { return armed; }
bool supervisorIsTumbled(void) { return false; }
bool supervisorIsCrashed(void) { return false; }
bool supervisorIsLocked(void) { return false; }
bool crtpIsConnected(void) { return linkUp; }
bool pmIsBatteryLow(void) { return batteryLow; }
StateEstimatorType stateEstimatorGetType(void) { return StateEstimatorTypeKalman; }

void testDebugPrint(const char* module, const char* fmt, ...) {
  if (!verbose) {
    return;
  }
  va_list args;
  va_start(args, fmt);
  printf("  [%7.2f s] %s: ", now / 1000.0, module);
  vprintf(fmt, args);
  va_end(args);
}

// stateEstimate.* in the EKF frame (yaw, roll, pitch in degrees)
enum { L_X = 1, L_Y, L_Z, L_VX, L_VY, L_VZ, L_ROLL, L_PITCH, L_YAW };
logVarId_t logGetVarId(const char* group, const char* name) {
  static const char* names[] = {"", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"};
  if (strcmp(group, "stateEstimate") == 0) {
    for (int i = 1; i <= L_YAW; i++) {
      if (strcmp(name, names[i]) == 0) {
        return (logVarId_t)i;
      }
    }
  }
  return 0xffff;
}
float logGetFloat(logVarId_t id) {
  const float c = cosf(ekf.yaw), s = sinf(ekf.yaw);
  const float dx = drone.x - ekf.x, dy = drone.y - ekf.y;
  switch (id) {
    case L_X: return c * dx + s * dy;
    case L_Y: return -s * dx + c * dy;
    case L_Z: return drone.z;
    case L_VX: return c * drone.vx + s * drone.vy;
    case L_VY: return -s * drone.vx + c * drone.vy;
    case L_VZ: return drone.vz;
    case L_ROLL:
    case L_PITCH: return 0.0f;
    case L_YAW: return wrap(drone.yaw - ekf.yaw) * 180.0f / PI_F;
    default: return NAN;
  }
}

enum { P_RESET = 1, P_FLOW2, P_RANGER };
paramVarId_t paramGetVarId(const char* group, const char* name) {
  paramVarId_t id = {0xffff, 0};
  if (!strcmp(group, "kalman") && !strcmp(name, "resetEstimation")) id.id = P_RESET;
  if (!strcmp(group, "deck") && !strcmp(name, "bcFlow2")) id.id = P_FLOW2;
  if (!strcmp(group, "deck") && !strcmp(name, "bcMultiranger")) id.id = P_RANGER;
  return id;
}
unsigned int paramGetUint(paramVarId_t id) {
  return id.id == P_FLOW2 ? flowDeck : id.id == P_RANGER ? rangerDeck : 0;
}
void paramSetInt(paramVarId_t id, int value) {
  if (id.id == P_RESET && value) {  // the EKF restarts at the drone, x along its nose
    ekf.x = drone.x;
    ekf.y = drone.y;
    ekf.yaw = drone.yaw;
  }
}

static bool motorsRunning(void) {
  if (!armed || !spValid || now - spTick > 2000) {  // commander watchdog shuts down after 2 s
    return false;
  }
  return !(sp.mode.x == modeDisable && sp.mode.y == modeDisable && sp.mode.z == modeDisable && sp.thrust == 0);
}

// The PID as first-order lags: velocity tau 0.25 s (sim's p.tau), yaw rate 0.05 s.
static void physics(void) {
  if (!motorsRunning()) {
    drone.vx = drone.vy = drone.vz = drone.yawRate = 0;
    drone.z = fmaxf(0, drone.z - 2.0f * DT);
    return;
  }
  const float c = cosf(ekf.yaw), s = sinf(ekf.yaw);
  float vzc = sp.mode.z == modeAbs ? clampf(2.0f * (sp.position.z - drone.z), -1, 1) : 0;
  float vxc = 0, vyc = 0;
  if (sp.mode.x == modeAbs) {
    const float wx = ekf.x + c * sp.position.x - s * sp.position.y;
    const float wy = ekf.y + s * sp.position.x + c * sp.position.y;
    vxc = 1.5f * (wx - drone.x);
    vyc = 1.5f * (wy - drone.y);
  } else if (sp.mode.x == modeVelocity) {
    const float r = sp.velocity_body ? drone.yaw : ekf.yaw;
    vxc = cosf(r) * sp.velocity.x - sinf(r) * sp.velocity.y;
    vyc = sinf(r) * sp.velocity.x + cosf(r) * sp.velocity.y;
  }
  const float v = hypotf(vxc, vyc);
  if (v > 1.0f) {
    vxc /= v;
    vyc /= v;
  }
  if (drone.z < 0.02f) {
    vxc = vyc = 0;  // on the floor
  }
  float rc = 0;
  if (sp.mode.yaw == modeAbs) {
    rc = clampf(6.0f * wrap(sp.attitude.yaw * PI_F / 180.0f + ekf.yaw - drone.yaw), -PI_F, PI_F);
  } else if (sp.mode.yaw == modeVelocity) {
    rc = sp.attitudeRate.yaw * PI_F / 180.0f;
  }
  drone.vx += (vxc - drone.vx) * DT / 0.25f;
  drone.vy += (vyc - drone.vy) * DT / 0.25f;
  drone.vz += (vzc - drone.vz) * DT / 0.15f;
  drone.yawRate += (rc - drone.yawRate) * DT / 0.05f;
  drone.x += drone.vx * DT;
  drone.y += drone.vy * DT;
  drone.z = fmaxf(0, drone.z + drone.vz * DT);
  drone.yaw = wrap(drone.yaw + drone.yawRate * DT);
  if (drone.z > 0.05f) {
    everFlew = true;
    minClearance = fminf(minClearance, clearance(drone.x, drone.y) - RADIUS);
  }
  maxZ = fmaxf(maxZ, drone.z);
}

// Multi-ranger: measured every 100 ms, written 100 ms later in driver order
// front, back, up, left, right, 2 ms apart (sequential I2C reads).
static const rangeDirection_t DRIVER_ORDER[5] = {rangeFront, rangeBack, rangeUp, rangeLeft, rangeRight};
static float measured[2][RANGE_T_END];  // two frames in flight: one measured, one being written
static TickType_t measuredAt[2];
static unsigned seed = 1;

static float gauss(void) {
  float u1 = (rand_r(&seed) + 1.0f) / (RAND_MAX + 2.0f), u2 = (rand_r(&seed) + 1.0f) / (RAND_MAX + 2.0f);
  return sqrtf(-2 * logf(u1)) * cosf(2 * PI_F * u2);
}

static void ranger(void) {
  if (!rangerOn) {
    return;
  }
  for (int s = 0; s < 2; s++) {
    for (int k = 0; k < 5; k++) {
      if (measuredAt[s] > 0 && now == measuredAt[s] + 100 + 2 * k) {
        rangeSet(DRIVER_ORDER[k], measured[s][DRIVER_ORDER[k]] / 1000.0f);
      }
    }
  }
  if (now % 100 == 0) {
    const int s = (now / 100) % 2;
    const float axis[RANGE_T_END] = {[rangeFront] = 0, [rangeBack] = PI_F, [rangeLeft] = PI_F / 2,
                                     [rangeRight] = -PI_F / 2};
    for (int d = 0; d <= rangeRight; d++) {
      const float a = drone.yaw + axis[d];
      const float ox = drone.x + 0.0163f * cosf(a), oy = drone.y + 0.0163f * sinf(a);
      float r = INFINITY;
      for (int k = 0; k < 9; k++) {
        r = fminf(r, castRay(ox, oy, a + (k - 4) * (27.0f * PI_F / 180.0f) / 8));
      }
      measured[s][d] = r > 4.0f ? 32767.0f : fmaxf(0, roundf(r * 1000.0f + 2.0f * gauss()));
    }
    measured[s][rangeUp] = upOverrideMm > 0 ? upOverrideMm : 32767.0f;
    measuredAt[s] = now;
  }
}

// ----- scenarios ---------------------------------------------------------------------------

typedef struct {
  const char* name;
  void (*setup)(void);
  void (*event)(void);  // every ms
  int reason;           // expected end reason
  int reached;          // expected targets reached, -1 = any
  bool flies;           // expected to take off
  float goalX, goalY;   // R_DONE: should end within 0.25 m of this
} scenario_t;

static const scenario_t* current;

static void mission(float sx, float sy, float psi0, int n, const float (*targets)[2]) {
  drone.x = sx;
  drone.y = sy;
  drone.yaw = psi0;
  setParam("wedgebug", "startX", sx);
  setParam("wedgebug", "startY", sy);
  setParam("wedgebug", "psi0", psi0);
  setParam("wedgebug", "nTargets", n);
  char name[8];
  for (int i = 0; i < n; i++) {
    snprintf(name, sizeof(name), "x%d", i);
    setParam("wedgebug", name, targets[i][0]);
    snprintf(name, sizeof(name), "y%d", i);
    setParam("wedgebug", name, targets[i][1]);
  }
}

static void fence(float x0, float x1, float y0, float y1) {
  setParam("wedgebug", "fenceXMin", x0);
  setParam("wedgebug", "fenceXMax", x1);
  setParam("wedgebug", "fenceYMin", y0);
  setParam("wedgebug", "fenceYMax", y1);
}

static void room(float x0, float x1, float y0, float y1) {
  const float v[4][2] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  addPolygon(v, 4);
  fence(x0 + 0.1f, x1 - 0.1f, y0 + 0.1f, y1 - 0.1f);
}

// One box straight in the way, 4 m flight along +x.
static void setupObstacle(void) {
  room(0, 6, -2, 2);
  addBox(2.5f, 0.0f, 1.0f, 1.0f);
  const float t[1][2] = {{4.5f, 0.0f}};
  mission(0.5f, 0.0f, 0.0f, 1, t);
}

// Started away from the origin facing +y: the mission/EKF frame mapping.
static void setupFrame(void) {
  room(0, 4, 0, 6);
  addBox(1.2f, 3.0f, 0.8f, 0.8f);
  const float t[1][2] = {{1.0f, 5.0f}};
  mission(1.0f, 1.0f, PI_F / 2, 1, t);
}

// matlab_main from sim/navigation/missions.py: 3 waypoints, 4 obstacles.
static void setupMatlabMain(void) {
  room(0, 10, 0, 8);
  addBox(2.5f, 3.0f, 1.0f, 1.0f);
  addBox(5.5f, 4.0f, 0.2f, 3.4f);
  addCircle(8.0f, 2.9f, 0.3f);
  addBox(7.6f, 5.3f, 1.6f, 0.5f);
  const float t[4][2] = {{3.4f, 4.2f}, {6.6f, 2.0f}, {9.4f, 3.8f}, {6.4f, 6.4f}};
  mission(0.8f, 0.8f, 0.0f, 4, t);
  setParam("wedgebug", "maxTime", 400);
}

static void setupBaseline(void) {
  setupObstacle();
  setParam("wedgebug", "tsRanges", 0);
}

static void setupNoDeck(void) {
  setupObstacle();
  rangerDeck = false;
}

static void setupBadMission(void) {
  setupObstacle();
  setParam("wedgebug", "x0", 7.0f);  // outside the fence
}

static void setupTightFence(void) {
  setupObstacle();
  setParam("wedgebug", "fenceYMax", 0.4f);  // going round the box needs y > 0.4 or < -0.4
  setParam("wedgebug", "fenceYMin", -0.4f);
}

static void setupWall(void) {  // a wall right across the room: goal unreachable
  room(0, 6, -2, 2);
  addBox(2.5f, 0.0f, 0.2f, 4.0f);
  const float t[1][2] = {{4.5f, 0.0f}};
  mission(0.5f, 0.0f, 0.0f, 1, t);
  setParam("wedgebug", "maxTime", 300);
}

static void noEvent(void) {}
static void abortAt8(void) {
  if (now == 9000) setParam("wedgebug", "start", 0);
}
static void rangerStopsAt12(void) {
  if (now == 12000) rangerOn = false;
}
static void handOverAt12(void) {
  if (now == 12000) upOverrideMm = 100;
}
static void linkLostAt12(void) {
  if (now == 12000) linkUp = false;
}
static void batteryAt12(void) {
  if (now == 12000) batteryLow = true;
}

static const scenario_t SCENARIOS[] = {
  {"obstacle", setupObstacle, noEvent, R_DONE, 1, true, 4.5f, 0.0f},
  {"frame", setupFrame, noEvent, R_DONE, 1, true, 1.0f, 5.0f},
  {"matlab_main", setupMatlabMain, noEvent, R_DONE, 4, true, 6.4f, 6.4f},
  {"baseline", setupBaseline, noEvent, R_DONE, 1, true, 4.5f, 0.0f},
  {"unreachable", setupWall, noEvent, R_DONE, 0, true, NAN, NAN},
  {"abort", setupObstacle, abortAt8, R_ABORT, -1, true, NAN, NAN},
  {"ranger_stale", setupObstacle, rangerStopsAt12, R_RANGER_STALE, -1, true, NAN, NAN},
  {"up_stop", setupObstacle, handOverAt12, R_UP_STOP, -1, true, NAN, NAN},
  {"link_loss", setupObstacle, linkLostAt12, R_LINK, -1, true, NAN, NAN},
  {"battery", setupObstacle, batteryAt12, R_BATTERY, -1, true, NAN, NAN},
  {"fence", setupTightFence, noEvent, R_FENCE, -1, true, NAN, NAN},
  {"no_deck", setupNoDeck, noEvent, R_NO_DECK, -1, false, NAN, NAN},
  {"bad_mission", setupBadMission, noEvent, R_BAD_MISSION, -1, false, NAN, NAN},
};

void vTaskDelayUntil(TickType_t* previousWakeTime, TickType_t increment) {
  const TickType_t wake = *previousWakeTime + increment;
  while (now < wake) {
    now++;
    physics();
    ranger();
    current->event();
    if (now == 1000) {
      setParam("wedgebug", "start", 1);  // the host starts the mission 1 s after boot
    }
  }
  *previousWakeTime = wake;
  const bool ended = now > 1000 && getLog("wedgebug", "state") == 0 && getLog("wedgebug", "reason") != R_NONE;
  if (ended || now >= TIME_LIMIT_MS) {
    longjmp(finished, 1);
  }
}

static int run(const scenario_t* sc) {
  current = sc;
  sc->setup();
  if (setjmp(finished) == 0) {
    appMain();
  }
  const int reason = (int)getLog("wedgebug", "reason");
  const int reached = (int)getLog("wedgebug", "reached");
  const int unreachable = (int)getLog("wedgebug", "unreachable");
  const float goalErr = hypotf(drone.x - sc->goalX, drone.y - sc->goalY);
  char why[160] = "";
  if (reason != sc->reason) {
    snprintf(why, sizeof(why), "reason %d, expected %d", reason, sc->reason);
  } else if (sc->reached >= 0 && reached != sc->reached) {
    snprintf(why, sizeof(why), "reached %d targets, expected %d", reached, sc->reached);
  } else if (everFlew != sc->flies) {
    snprintf(why, sizeof(why), sc->flies ? "never took off" : "took off");
  } else if (everFlew && minClearance < 0) {
    snprintf(why, sizeof(why), "collided (clearance %.3f m)", minClearance);
  } else if (drone.z > 0.01f || motorsRunning()) {
    snprintf(why, sizeof(why), "did not end on the floor with motors off (z %.2f m)", drone.z);
  } else if (sc->reason == R_DONE && isfinite(sc->goalX) && goalErr > 0.25f) {
    snprintf(why, sizeof(why), "landed %.2f m from the goal", goalErr);
  } else if (maxZ > 0.6f) {
    snprintf(why, sizeof(why), "climbed to %.2f m", maxZ);
  }
  printf("%-4s %-13s t %6.1f s  reason %2d  reached %d  unreachable %d  min clearance %6.3f m  end (%5.2f, %5.2f)%s%s\n",
         why[0] ? "FAIL" : "ok", sc->name, now / 1000.0, reason, reached, unreachable,
         everFlew ? minClearance : NAN, drone.x, drone.y, why[0] ? "  <- " : "", why);
  return why[0] ? 1 : 0;
}

int main(int argc, char** argv) {
  const char* only = NULL;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-v")) {
      verbose = true;
    } else {
      only = argv[i];
    }
  }
  int failures = 0, count = 0;
  for (size_t i = 0; i < sizeof(SCENARIOS) / sizeof(SCENARIOS[0]); i++) {
    if (only && strcmp(only, SCENARIOS[i].name)) {
      continue;
    }
    fflush(stdout);
    const pid_t pid = fork();
    if (pid == 0) {
      exit(run(&SCENARIOS[i]));
    }
    int status;
    waitpid(pid, &status, 0);
    failures += !(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    count++;
  }
  if (count == 0) {
    fprintf(stderr, "no scenario named %s\n", only);
    return 2;
  }
  printf("%d of %d scenarios passed\n", count - failures, count);
  return failures ? 1 : 0;
}

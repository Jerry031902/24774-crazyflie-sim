/**
 * WedgeBug reactive navigation, platform-independent core. See wedgebug.h.
 *
 * Function-by-function port of matlab-trajectory-sim/src/nav/navigator_step.m
 * (reference Python port: sim/navigation/wedgebug.py). Comments name the
 * MATLAB subfunction each block comes from.
 */
#include "wedgebug.h"

#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define DEG_TO_RAD(d) ((d) * (PI_F / 180.0f))

// Unit vectors of directions 1..4 (forward, left, back, right) in the body frame.
static const float DIRS[4][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}, {0.0f, -1.0f}};

typedef struct {
  int dir;
  float speed;
  float yawRate;
  float u[2];  // used when dir == 0
} command_t;

typedef struct {
  bool blocked;
  float edge[2][2];  // row 0 left, row 1 right
  bool seen[2];
  float cost[2];
  int ends[2];
  int c;
} wedge_t;

// ----- helpers --------------------------------------------------------------

static command_t command(int dir, float speed, float yawRate) {
  command_t c = {dir, speed, yawRate, {0.0f, 0.0f}};
  return c;
}

static float wrapToPi(float a) {
  a = fmodf(a + PI_F, 2.0f * PI_F);
  if (a < 0.0f) {
    a += 2.0f * PI_F;
  }
  return a - PI_F;
}

static float norm2(float x, float y) { return sqrtf(x * x + y * y); }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int toward(int d, int hand) { return (d + (hand < 0 ? 2 : 0)) % 4 + 1; }

static int away(int d, int hand) { return (d + (hand > 0 ? 2 : 0)) % 4 + 1; }

static float roundHalfAway(float x) { return copysignf(floorf(fabsf(x) + 0.5f), x); }

static float yawCommand(const wedgebugParams_t* p, float err) {
  return clampf(p->kYaw * err, -p->yawRateMax, p->yawRateMax);
}

// World vector to body frame for yaw psi: R(psi)^T * v.
static void toBody(float psi, float wx, float wy, float* bx, float* by) {
  const float c = cosf(psi), s = sinf(psi);
  *bx = c * wx + s * wy;
  *by = -s * wx + c * wy;
}

static const float* target(const wedgebug_t* nav) { return nav->targets[nav->wp]; }

static float distTo(const float* a, const float* b) { return norm2(a[0] - b[0], a[1] - b[1]); }

static void logEvent(wedgebug_t* nav, float t, int type, const float* pos) {
  if (nav->stepEventCount < WEDGEBUG_MAX_STEP_EVENTS) {
    wedgebugEvent_t* e = &nav->stepEvents[nav->stepEventCount++];
    e->type = type;
    e->wp = nav->wp;
    e->t = t;
    e->x = pos[0];
    e->y = pos[1];
  }
}

// ----- episode / target bookkeeping ------------------------------------------

static void endEpisode(wedgebug_t* nav) {
  nav->episode = false;
  nav->side = 0;
  nav->subgoal[0] = nav->subgoal[1] = NAN;
  nav->hitPoint[0] = nav->hitPoint[1] = NAN;
  nav->hitDist = INFINITY;
}

static void nextTarget(wedgebug_t* nav) {
  endEpisode(nav);
  nav->following = false;
  nav->facing = false;
  nav->phase = WEDGEBUG_PHASE_NONE;
  if (nav->wp < nav->targetCount - 1) {
    nav->wp++;
    nav->mode = WEDGEBUG_GO_TO_GOAL;
  } else {
    nav->mode = WEDGEBUG_DONE;
  }
}

static bool giveUp(wedgebug_t* nav, const float* pos, float t) {
  logEvent(nav, t, WEDGEBUG_EVENT_UNREACHABLE, pos);
  nextTarget(nav);
  return true;
}

static void startAvoid(wedgebug_t* nav, const float* pos, float t) {
  nav->mode = WEDGEBUG_AVOID;
  if (!nav->episode) {  // first contact: this is the hit point
    const float* tg = target(nav);
    nav->episode = true;
    nav->hitPoint[0] = pos[0];
    nav->hitPoint[1] = pos[1];
    nav->hitDist = distTo(tg, pos);
    nav->side = 0;
    nav->epTravel = 0.0f;
    nav->sweepSpotCount = 0;
    nav->lastBearing = atan2f(pos[1] - tg[1], pos[0] - tg[0]);
    nav->winding = 0.0f;
    logEvent(nav, t, WEDGEBUG_EVENT_HIT, pos);
  }
}

static void startSweep(wedgebug_t* nav, float psi) {
  nav->phase = WEDGEBUG_PHASE_SWEEP;
  nav->sweepStage = WEDGEBUG_SWEEP_STOP;
  nav->sweepCenter = psi;
}

static void startFollow(wedgebug_t* nav, float psi, int hand) {
  nav->phase = WEDGEBUG_PHASE_ALIGN;
  nav->hand = hand;
  nav->psiHold = roundHalfAway(psi / (PI_F / 2.0f)) * (PI_F / 2.0f);  // nearest room axis
  nav->dFollowed = INFINITY;
  nav->following = false;
  nav->wallSeen = false;
  nav->sinceTurn = 0.0f;
  nav->travel = 0.0f;
}

static void turn(wedgebug_t* nav, int newDir, const float* pos, bool wallSeen) {
  nav->travelDir = newDir;
  nav->phase = WEDGEBUG_PHASE_FOLLOW;
  nav->wallSeen = wallSeen;
  nav->sinceTurn = 0.0f;
  if (!nav->following) {  // first turn: the wall is now beside the drone
    nav->following = true;
    nav->loopPoint[0] = pos[0];
    nav->loopPoint[1] = pos[1];
    nav->travel = 0.0f;
  }
}

// ----- fly_to -----------------------------------------------------------------

// Face point, then fly straight at it nose first. *event: 0 none, 1 blocked,
// 2 side_left, 3 side_right.
enum { EV_NONE = 0, EV_BLOCKED = 1, EV_SIDE_LEFT = 2, EV_SIDE_RIGHT = 3 };

static command_t flyTo(wedgebug_t* nav, const float* pos, float psi, float speed, const float* r,
                       const float* point, float dBlock, int* event) {
  const wedgebugParams_t* p = &nav->params;
  *event = EV_NONE;
  const float dx = point[0] - pos[0], dy = point[1] - pos[1];
  const float dist = norm2(dx, dy);

  if (dist <= p->yawFreeRadius) {
    // The last few centimetres: slide straight onto the point, no turning.
    command_t c = command(0, 0.0f, 0.0f);
    if (dist > 0.0f) {
      float bx, by;
      toBody(psi, dx, dy, &bx, &by);
      const float s = fminf(p->vCruise, p->kStop * dist) / dist;
      c.u[0] = s * bx;
      c.u[1] = s * by;
    }
    return c;
  }

  const float err = wrapToPi(atan2f(dy, dx) - psi);
  if (fabsf(err) > p->yawTol) {
    nav->facing = false;
  } else if (fabsf(err) < DEG_TO_RAD(2.0f)) {
    nav->facing = true;
  }
  if (!nav->facing) {
    // Not facing the point: stop, then turn on the spot until lined up.
    return command(1, 0.0f, speed < p->vTurn ? yawCommand(p, err) : 0.0f);
  }

  const float front = r[0];
  const bool shortOfPoint = front - p->dWall < dist - 0.02f;
  if (shortOfPoint && front <= dBlock + 0.02f) {
    *event = EV_BLOCKED;
    return command(1, 0.0f, 0.0f);
  }
  if (r[1] < p->dSide) {
    *event = EV_SIDE_LEFT;
    return command(1, 0.0f, 0.0f);
  }
  if (r[3] < p->dSide) {
    *event = EV_SIDE_RIGHT;
    return command(1, 0.0f, 0.0f);
  }

  const float stopAt = shortOfPoint ? dBlock : p->dWall;  // come to rest where it will stop and look
  const float speedCmd = fminf(fminf(p->vCruise, p->kStop * dist), p->kStop * fmaxf(front - stopAt, 0.0f));
  return command(1, speedCmd, yawCommand(p, err));  // keep the nose on the point
}

// ----- go_to_goal ---------------------------------------------------------------

static command_t goToGoal(wedgebug_t* nav, const float* pos, float psi, float speed, const float* r,
                          float t, bool* switched) {
  const float* tg = target(nav);
  if (nav->episode && distTo(tg, pos) < nav->hitDist - nav->params.leaveMargin) {
    endEpisode(nav);  // past the obstacle: the next one is a fresh start
  }
  int event;
  command_t c = flyTo(nav, pos, psi, speed, r, tg, nav->params.dSweep, &event);
  if (event == EV_BLOCKED) {
    startAvoid(nav, pos, t);
    startSweep(nav, psi);
  } else if (event == EV_SIDE_LEFT) {
    startAvoid(nav, pos, t);
    startFollow(nav, psi, +1);
  } else if (event == EV_SIDE_RIGHT) {
    startAvoid(nav, pos, t);
    startFollow(nav, psi, -1);
  }
  *switched = event != EV_NONE;
  return c;
}

// ----- sweep: read_wedge, choose_pass_point, seen_free, decide ----------------

static float scanRel(const wedgebug_t* nav, int i) { return wrapToPi(nav->scanAng[i] - nav->sweepCenter); }

// Stable insertion sort of the scan by angle relative to the sweep centre.
static void sortScan(wedgebug_t* nav) {
  for (int i = 1; i < nav->scanCount; i++) {
    const float ang = nav->scanAng[i], rng = nav->scanRng[i];
    const float key = scanRel(nav, i);
    int j = i - 1;
    while (j >= 0 && scanRel(nav, j) > key) {
      nav->scanAng[j + 1] = nav->scanAng[j];
      nav->scanRng[j + 1] = nav->scanRng[j];
      j--;
    }
    nav->scanAng[j + 1] = ang;
    nav->scanRng[j + 1] = rng;
  }
}

// Edges of the obstacle in a sorted sweep, one per side. Walk out from the
// middle; an edge is the last outline point before the range jumps up by more
// than edgeJump or returns nothing. Points sit half a cone inward of each
// reading, which puts an edge on the obstacle's corner.
static void readWedge(const wedgebug_t* nav, const float* pos, const float* tg, wedge_t* w) {
  const wedgebugParams_t* p = &nav->params;
  const float* r = nav->scanRng;
  memset(w, 0, sizeof(*w));
  for (int i = 0; i < 2; i++) {
    w->edge[i][0] = w->edge[i][1] = NAN;
    w->cost[i] = INFINITY;
  }
  int c = -1;
  for (int i = 0; i < nav->scanCount; i++) {
    if (fabsf(scanRel(nav, i)) <= DEG_TO_RAD(10.0f) && (c < 0 || r[i] < r[c])) {
      c = i;
    }
  }
  if (c < 0) {
    return;
  }
  w->blocked = r[c] <= p->dSweep + 0.3f;
  if (!w->blocked) {
    return;
  }
  w->c = c;
  const float h = p->rangerFov / 2.0f;
  for (int i = 0; i < 2; i++) {
    const int step = 1 - 2 * i;  // +1 walks left (increasing angle), -1 right
    int j = c;
    while (true) {
      const int jn = j + step;
      if (jn < 0 || jn >= nav->scanCount) {
        break;  // reached the end of the scan
      }
      if (isinf(r[jn]) || r[jn] - r[j] > p->edgeJump) {
        w->seen[i] = true;
        break;
      }
      j = jn;
    }
    w->ends[i] = j;
    const float th = nav->sweepCenter + scanRel(nav, j) - (float)step * h;
    w->edge[i][0] = pos[0] + r[j] * cosf(th);
    w->edge[i][1] = pos[1] + r[j] * sinf(th);
    w->cost[i] = distTo(w->edge[i], pos) + distTo(tg, w->edge[i]);
  }
}

// Smallest distance from the outline between the two edges to segment a-b.
// False if the outline has no points.
static bool outlineMinDistance(const wedgebug_t* nav, const wedge_t* w, const float* pos,
                               const float* a, const float* b, float* out) {
  const float h = nav->params.rangerFov / 2.0f;
  const float abx = b[0] - a[0], aby = b[1] - a[1];
  const float denom = abx * abx + aby * aby;
  bool any = false;
  float best = INFINITY;
  for (int i = w->ends[1]; i <= w->ends[0]; i++) {
    const float r = nav->scanRng[i];
    if (!isfinite(r)) {
      continue;
    }
    const float sgn = (float)((i > w->c) - (i < w->c));
    const float th = nav->sweepCenter + scanRel(nav, i) - sgn * h;
    const float px = pos[0] + r * cosf(th), py = pos[1] + r * sinf(th);
    const float s = denom == 0.0f ? 0.0f : clampf(((px - a[0]) * abx + (py - a[1]) * aby) / denom, 0.0f, 1.0f);
    best = fminf(best, norm2(px - (a[0] + s * abx), py - (a[1] + s * aby)));
    any = true;
  }
  *out = best;
  return any;
}

// Q, and the line from the sweep spot to it, is clear if some reading whose
// cone covers Q's direction reaches dWall past it.
static bool seenFree(const wedgebug_t* nav, const float* q, const float* pos) {
  const wedgebugParams_t* p = &nav->params;
  const float vx = q[0] - pos[0], vy = q[1] - pos[1];
  const float ang = atan2f(vy, vx), d = norm2(vx, vy);
  for (int i = 0; i < nav->scanCount; i++) {
    if (fabsf(wrapToPi(ang - nav->scanAng[i])) <= p->rangerFov / 2.0f &&
        d <= fminf(nav->scanRng[i], p->rangerRangeMax) - p->dWall) {
      return true;
    }
  }
  return false;
}

static void choosePassPoint(const wedgebug_t* nav, const wedge_t* w, const float* e, const float* pos,
                            const float* tg, int side, float* out) {
  const wedgebugParams_t* p = &nav->params;
  const float len = distTo(e, pos);
  const float rayX = (e[0] - pos[0]) / len, rayY = (e[1] - pos[1]) / len;
  const float outX = (float)side * -rayY, outY = (float)side * rayX;
  out[0] = e[0] + p->passMargin * outX;
  out[1] = e[1] + p->passMargin * outY;
  const int n = (int)floorf((p->passMax - p->passMargin) / 0.05f + 1e-4f) + 1;
  for (int k = 0; k < n; k++) {
    const float s = p->passMargin + (float)k * 0.05f;
    const float q[2] = {e[0] + s * outX, e[1] + s * outY};
    if (!seenFree(nav, q, pos)) {
      break;  // beyond what the sweep can vouch for
    }
    out[0] = q[0];
    out[1] = q[1];
    float d;
    if (outlineMinDistance(nav, w, pos, out, tg, &d) && d >= p->tangentClearance) {
      break;  // the next leg clears the obstacle
    }
  }
}

static bool decide(wedgebug_t* nav, const float* pos) {
  const float* tg = target(nav);
  sortScan(nav);
  wedge_t w;
  readWedge(nav, pos, tg, &w);
  if (!w.blocked) {
    nav->mode = WEDGEBUG_GO_TO_GOAL;  // nothing in the way after all
    nav->facing = false;
    return true;
  }
  if (nav->side == 0) {
    nav->side = w.cost[1] < w.cost[0] ? -1 : +1;  // first minimum wins ties, as in MATLAB
  }
  const int i = (1 - nav->side) / 2;

  float pass[2] = {NAN, NAN};
  if (w.seen[i]) {
    choosePassPoint(nav, &w, w.edge[i], pos, tg, nav->side, pass);
  }
  wedgebugSweep_t* s = &nav->lastSweep;
  s->x = pos[0];
  s->y = pos[1];
  s->center = nav->sweepCenter;
  s->edgeLeftX = w.edge[0][0];
  s->edgeLeftY = w.edge[0][1];
  s->edgeRightX = w.edge[1][0];
  s->edgeRightY = w.edge[1][1];
  s->seenLeft = w.seen[0];
  s->seenRight = w.seen[1];
  s->costLeft = w.cost[0];
  s->costRight = w.cost[1];
  s->side = nav->side;
  s->passX = pass[0];
  s->passY = pass[1];
  nav->sweepCount++;
  if (nav->sweepSpotCount < WEDGEBUG_MAX_SWEEP_SPOTS) {
    float* spot = nav->sweepSpots[nav->sweepSpotCount++];
    spot[0] = pos[0];
    spot[1] = pos[1];
    spot[2] = nav->epTravel;
  }

  if (w.seen[i]) {
    nav->subgoal[0] = pass[0];
    nav->subgoal[1] = pass[1];
    nav->phase = WEDGEBUG_PHASE_PASS;
    nav->facing = false;
  } else {
    // Round the left = wall on the right. Square up from the heading the
    // sweep looked along, not from where the sweep ended.
    startFollow(nav, nav->sweepCenter, -nav->side);
  }
  return true;
}

static command_t sweep(wedgebug_t* nav, const float* pos, float psi, float speed, const float* r, float t,
                       bool* switched) {
  const wedgebugParams_t* p = &nav->params;
  const float half = p->sweepHalf;
  command_t c = command(nav->execDir, 0.0f, 0.0f);
  *switched = false;

  if (nav->sweepStage == WEDGEBUG_SWEEP_STOP) {
    if (speed >= p->vTurn) {
      return c;  // hover first
    }
    // Back at an earlier sweep spot without having got closer, or swept too
    // often: the target cannot be reached.
    if (nav->sweepSpotCount > 0) {
      const int maxSweeps = nav->params.maxSweeps < WEDGEBUG_MAX_SWEEP_SPOTS ? nav->params.maxSweeps
                                                                             : WEDGEBUG_MAX_SWEEP_SPOTS;
      bool again = nav->sweepSpotCount >= maxSweeps;
      for (int i = 0; i < nav->sweepSpotCount && !again; i++) {
        const float* s = nav->sweepSpots[i];
        again = distTo(s, pos) < 2.0f * p->loopTol && nav->epTravel - s[2] > p->loopMinTravel;
      }
      if (again) {
        *switched = giveUp(nav, pos, t);
        return c;
      }
    }
    nav->sweepStage = WEDGEBUG_SWEEP_TO_START;
  } else if (nav->sweepStage == WEDGEBUG_SWEEP_TO_START) {  // turn to the right end of the wedge
    const float err = wrapToPi(nav->sweepCenter - half - psi);
    c = command(nav->execDir, 0.0f, yawCommand(p, err));
    if (fabsf(err) < DEG_TO_RAD(1.0f)) {
      nav->sweepStage = WEDGEBUG_SWEEP_SCAN;
      nav->scanCount = 0;
    }
  } else {
    // Sweep to the left end, logging front, left and right: the sides sit
    // 90 deg off the nose, so a +/-45 deg sweep sees +/-135 deg.
    if (nav->scanCount + 3 <= WEDGEBUG_MAX_SCAN) {
      const float ang[3] = {psi, psi + PI_F / 2.0f, psi - PI_F / 2.0f};
      const float rng[3] = {r[0], r[1], r[3]};
      for (int k = 0; k < 3; k++) {
        nav->scanAng[nav->scanCount] = ang[k];
        nav->scanRng[nav->scanCount] = rng[k];
        nav->scanCount++;
      }
    }
    if (wrapToPi(psi - (nav->sweepCenter - half)) < 2.0f * half - 1e-3f) {
      return command(nav->execDir, 0.0f, p->sweepRate);
    }
    *switched = decide(nav, pos);
  }
  return c;
}

// ----- pass: fly to the point outside the chosen edge ---------------------------

static command_t passEdge(wedgebug_t* nav, const float* pos, float psi, float speed, const float* r,
                          bool* switched) {
  const wedgebugParams_t* p = &nav->params;
  if (distTo(nav->subgoal, pos) < p->wpTol + 0.05f) {
    nav->mode = WEDGEBUG_GO_TO_GOAL;  // round the edge: head for the target
    nav->subgoal[0] = nav->subgoal[1] = NAN;
    nav->facing = false;
    *switched = true;
    return command(nav->execDir, 0.0f, 0.0f);
  }
  // Only something close ahead counts: the corner being rounded sits just
  // beside this path and shows in the front cone from further out.
  int event;
  const float goal[2] = {nav->subgoal[0], nav->subgoal[1]};
  command_t c = flyTo(nav, pos, psi, speed, r, goal, p->dDetect, &event);
  if (event == EV_BLOCKED) {
    startSweep(nav, psi);  // more of the obstacle: look again from here
  } else if (event == EV_SIDE_LEFT) {
    startFollow(nav, psi, +1);
  } else if (event == EV_SIDE_RIGHT) {
    startFollow(nav, psi, -1);
  }
  *switched = event != EV_NONE;
  return c;
}

// ----- follow: wall-following with the wall on side 'hand' ----------------------

// How far the drone can see it could fly toward the target (rel, body frame).
static float freeToward(const wedgebugParams_t* p, float relX, float relY, const float* r) {
  const float dist = norm2(relX, relY);
  if (dist == 0.0f) {
    return INFINITY;
  }
  float best = INFINITY;
  bool anyLook = false, allInf = true;
  for (int s = 0; s < 4; s++) {
    const float cosine = clampf((DIRS[s][0] * relX + DIRS[s][1] * relY) / dist, -1.0f, 1.0f);
    const float a = fmaxf(acosf(cosine) - p->rangerFov / 2.0f, 0.0f);
    if (a < DEG_TO_RAD(80.0f)) {
      anyLook = true;
      allInf = allInf && isinf(r[s]);
      best = fminf(best, fminf(r[s], p->rangerRangeMax) / cosf(a));
    }
  }
  if (!anyLook || allInf) {
    return INFINITY;
  }
  return fmaxf(best - p->dWall, 0.0f);
}

static command_t follow(wedgebug_t* nav, const float* pos, float psi, float speed, const float* r, float moved,
                        float t, bool* switched) {
  const wedgebugParams_t* p = &nav->params;
  const float holdRate = yawCommand(p, wrapToPi(nav->psiHold - psi));
  *switched = false;

  if (nav->phase == WEDGEBUG_PHASE_ALIGN) {
    const bool squared = fabsf(wrapToPi(nav->psiHold - psi)) < DEG_TO_RAD(0.5f);
    if (!(squared && speed < p->vTurn)) {
      // Stop, then turn on the spot to the nearest room axis.
      return command(nav->execDir, 0.0f, speed < p->vTurn ? holdRate : 0.0f);
    }
    // Head for the obstacle along whichever of front, left, right sees it nearest.
    const int sides[3] = {1, 2, 4};
    int best = 0;
    for (int i = 1; i < 3; i++) {
      if (r[sides[i] - 1] < r[sides[best] - 1]) {
        best = i;
      }
    }
    nav->travelDir = sides[best];
    if (r[sides[best] - 1] > p->dDetect + p->cornerMargin) {
      nav->travelDir = 1;
    }
    nav->phase = WEDGEBUG_PHASE_FOLLOW;
    nav->sinceTurn = 0.0f;
  }

  int k = nav->travelDir;
  const bool flying = nav->execDir == k;  // false while braking for a turn
  if (flying) {
    nav->sinceTurn += moved;
    if (nav->following) {
      nav->travel += moved;
    }
  }

  // Exit: closer to the target than the hit point, and (TangentBug's rule)
  // the free space seen toward the target reaches closer to it than any
  // point of the wall followed so far.
  const float* tg = target(nav);
  const float c = cosf(psi), s = sinf(psi);
  float relX, relY;
  toBody(psi, tg[0] - pos[0], tg[1] - pos[1], &relX, &relY);
  const int wallDirs[2] = {k, toward(k, nav->hand)};  // front and wall-side readings are wall points
  for (int i = 0; i < 2; i++) {
    const float rd = r[wallDirs[i] - 1];
    if (isfinite(rd)) {
      const float bx = DIRS[wallDirs[i] - 1][0] * rd, by = DIRS[wallDirs[i] - 1][1] * rd;
      const float wallPt[2] = {pos[0] + c * bx - s * by, pos[1] + s * bx + c * by};
      nav->dFollowed = fminf(nav->dFollowed, distTo(tg, wallPt));
    }
  }
  const float relDist = norm2(relX, relY);
  const float reach = fmaxf(relDist - freeToward(p, relX, relY, r), 0.0f);
  if (nav->following && relDist < nav->hitDist - p->leaveMargin && reach < nav->dFollowed - p->leaveMargin) {
    logEvent(nav, t, WEDGEBUG_EVENT_LEAVE, pos);
    endEpisode(nav);
    nav->mode = WEDGEBUG_GO_TO_GOAL;
    nav->facing = false;
    *switched = true;
    return command(k, 0.0f, 0.0f);
  }

  // Full loop around the obstacle without an exit: the target is unreachable.
  if (nav->following && nav->travel > p->loopMinTravel && distTo(pos, nav->loopPoint) < p->loopTol) {
    *switched = giveUp(nav, pos, t);
    return command(k, 0.0f, 0.0f);
  }

  // Nothing blocked the way in after squaring up: a false alarm, carry on.
  if (!nav->following && flying && nav->sinceTurn > p->dDetect + p->cornerMargin) {
    nav->mode = WEDGEBUG_GO_TO_GOAL;
    nav->facing = false;
    *switched = true;
    return command(k, 0.0f, 0.0f);
  }

  // Wall-following rules, only while actually flying the travel direction.
  const float front = r[k - 1];
  const float wall = r[toward(k, nav->hand) - 1];
  if (flying) {
    if (wall <= p->dOpen) {
      nav->wallSeen = true;
    }
    if (front <= p->dWall + 0.02f || wall < p->dClose) {
      turn(nav, away(k, nav->hand), pos, true);  // blocked ahead, or too close to the wall
    } else if (nav->phase == WEDGEBUG_PHASE_FOLLOW) {
      if (nav->wallSeen && wall > p->dOpen) {
        nav->phase = WEDGEBUG_PHASE_OVERSHOOT;  // the wall has ended
        nav->overshootLeft = p->cornerMargin;
      } else if (nav->following && !nav->wallSeen && nav->sinceTurn > 2.0f * (p->dWall + p->cornerMargin)) {
        turn(nav, toward(k, nav->hand), pos, false);  // no wall came back: wrap again
      }
    } else {
      nav->overshootLeft -= moved;
      if (nav->overshootLeft <= 0.02f) {
        turn(nav, toward(k, nav->hand), pos, false);  // clear of the corner: turn back toward the wall
      }
    }
  }

  k = nav->travelDir;
  float speedCmd = fminf(p->vFollow, p->kStop * fmaxf(r[k - 1] - p->dWall, 0.0f));
  if (nav->phase == WEDGEBUG_PHASE_OVERSHOOT) {
    speedCmd = fminf(speedCmd, p->kStop * fmaxf(nav->overshootLeft, 0.0f));
  }
  return command(k, speedCmd, holdRate);
}

static command_t avoid(wedgebug_t* nav, const float* pos, float psi, float speed, const float* r, float moved,
                       float t, bool* switched) {
  if (nav->phase == WEDGEBUG_PHASE_SWEEP) {
    return sweep(nav, pos, psi, speed, r, t, switched);
  }
  if (nav->phase == WEDGEBUG_PHASE_PASS) {
    return passEdge(nav, pos, psi, speed, r, switched);
  }
  return follow(nav, pos, psi, speed, r, moved, t, switched);
}

// ----- public API ------------------------------------------------------------------

void wedgebugDefaultParams(wedgebugParams_t* p) {
  p->yawRateMax = DEG_TO_RAD(180.0f);
  p->rangerFov = DEG_TO_RAD(27.0f);
  p->rangerRangeMax = 4.0f;
  p->vCruise = 0.5f;
  p->vFollow = 0.3f;
  p->kStop = 2.0f;
  p->kYaw = 4.0f;
  p->yawTol = DEG_TO_RAD(10.0f);
  p->yawFreeRadius = 0.2f;
  p->dSweep = 1.0f;
  p->sweepHalf = DEG_TO_RAD(45.0f);
  p->sweepRate = DEG_TO_RAD(60.0f);
  p->edgeJump = 0.5f;
  p->passMargin = 0.4f;
  p->tangentClearance = 0.5f;
  p->passMax = 1.5f;
  p->maxSweeps = 12;
  p->maxWinding = 2.5f * PI_F;
  p->dSide = 0.25f;
  p->dDetect = 0.6f;
  p->dWall = 0.3f;
  p->dOpen = 0.6f;
  p->dClose = 0.22f;
  p->cornerMargin = 0.3f;
  p->vTurn = 0.05f;
  p->wpTol = 0.10f;
  p->leaveMargin = 0.10f;
  p->loopTol = 0.15f;
  p->loopMinTravel = 1.0f;
}

void wedgebugInit(wedgebug_t* nav, const wedgebugParams_t* params, float startX, float startY) {
  memset(nav, 0, sizeof(*nav));
  nav->params = *params;
  nav->mode = WEDGEBUG_GO_TO_GOAL;
  nav->phase = WEDGEBUG_PHASE_NONE;
  nav->execDir = 1;
  nav->lastPos[0] = startX;
  nav->lastPos[1] = startY;
  endEpisode(nav);
  nav->hand = 1;
  nav->travelDir = 1;
  nav->loopPoint[0] = nav->loopPoint[1] = NAN;
  nav->dFollowed = INFINITY;
  nav->lastSweep.passX = nav->lastSweep.passY = NAN;
}

bool wedgebugAddTarget(wedgebug_t* nav, float x, float y) {
  if (nav->targetCount >= WEDGEBUG_MAX_TARGETS) {
    return false;
  }
  nav->targets[nav->targetCount][0] = x;
  nav->targets[nav->targetCount][1] = y;
  nav->targetCount++;
  return true;
}

void wedgebugStep(wedgebug_t* nav, const wedgebugInput_t* in, wedgebugOutput_t* out) {
  const wedgebugParams_t* p = &nav->params;
  const float pos[2] = {in->x, in->y};
  const float r[4] = {in->rangeFront, in->rangeLeft, in->rangeBack, in->rangeRight};
  const float psi = in->yaw, t = in->t;
  const float speed = norm2(in->vx, in->vy);
  const float moved = distTo(pos, nav->lastPos);
  nav->lastPos[0] = pos[0];
  nav->lastPos[1] = pos[1];
  nav->stepEventCount = 0;
  out->vxBody = out->vyBody = out->yawRate = 0.0f;

  if (nav->targetCount == 0) {
    nav->mode = WEDGEBUG_DONE;
  }
  if (nav->mode != WEDGEBUG_DONE && nav->episode) {
    nav->epTravel += moved;
    // Circling the target more than maxWinding without getting closer: walled in.
    const float* tg = target(nav);
    const float bearing = atan2f(pos[1] - tg[1], pos[0] - tg[0]);
    nav->winding += wrapToPi(bearing - nav->lastBearing);
    nav->lastBearing = bearing;
    if (fabsf(nav->winding) > p->maxWinding) {
      logEvent(nav, t, WEDGEBUG_EVENT_UNREACHABLE, pos);
      nextTarget(nav);
    }
  }

  if (nav->mode != WEDGEBUG_DONE && distTo(target(nav), pos) < p->wpTol) {
    logEvent(nav, t, WEDGEBUG_EVENT_REACHED, pos);
    nextTarget(nav);
  }

  // A mode or phase switch is handled within the same tick.
  command_t c = command(nav->execDir, 0.0f, 0.0f);
  for (int pass = 0; pass < 4; pass++) {
    if (nav->mode == WEDGEBUG_DONE) {
      return;
    }
    bool switched;
    if (nav->mode == WEDGEBUG_GO_TO_GOAL) {
      c = goToGoal(nav, pos, psi, speed, r, t, &switched);
    } else {
      c = avoid(nav, pos, psi, speed, r, moved, t, &switched);
    }
    if (!switched) {
      break;
    }
  }

  // Fly the command, braking first if it changes direction.
  if (c.dir == 0) {
    nav->execDir = 0;
    out->vxBody = c.u[0];
    out->vyBody = c.u[1];
    out->yawRate = c.yawRate;
    return;
  }
  if (c.dir != nav->execDir) {
    if (speed > p->vTurn) {
      return;
    }
    nav->execDir = c.dir;
  }
  out->vxBody = c.speed * DIRS[c.dir - 1][0];
  out->vyBody = c.speed * DIRS[c.dir - 1][1];
  out->yawRate = c.yawRate;
}

bool wedgebugGetStepEvent(const wedgebug_t* nav, int i, wedgebugEvent_t* out) {
  if (i < 0 || i >= nav->stepEventCount) {
    return false;
  }
  *out = nav->stepEvents[i];
  return true;
}

/* CPU-only tests for the typed C transaction and timestamped scan extension. */
#include "wedgebug.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#undef assert
#define assert(test) do { if (!(test)) { fprintf(stderr, "failed at line %d: %s\n", __LINE__, #test); return 1; } } while (0)

int main(void) {
  wedgebugParams_t p;
  wedgebug_t nav, saved;
  wedgebugInput_t in = {0};
  wedgebugOutput_t out;
  wedgebugDefaultParams(&p);
  wedgebugInit(&nav, &p, 1.0f, 4.0f);
  assert(wedgebugAddTarget(&nav, 9.0f, 4.0f));
  in.x = 1.0f; in.y = 4.0f;
  in.rangeFront = in.rangeLeft = in.rangeBack = in.rangeRight = INFINITY;
  wedgebugStep(&nav, &in, &out);
  assert(fabsf(out.vxBody - .5f) < 1e-6f && out.vyBody == 0.0f);
  wedgebugCopyState(&saved, &nav);
  nav.wp = 5; nav.scanCount = 71;
  wedgebugCopyState(&nav, &saved);
  assert(memcmp(&nav, &saved, sizeof(nav)) == 0);

  nav.episode = true; nav.side = 1; nav.hitDist = 8.0f;
  nav.sweepSpotCount = 1; nav.sweepSpots[0][0] = 1.2f;
  nav.scanCount = 6; nav.scanSequence = 9; nav.subgoal[0] = 3.0f;
  wedgebugResume(&nav, 2.0f, 4.0f, 0.0f, 1.0f);
  assert(nav.wp == 0 && nav.side == 1 && nav.hitDist == 8.0f && nav.sweepSpotCount == 1);
  assert(nav.mode == WEDGEBUG_AVOID && nav.phase == WEDGEBUG_PHASE_SWEEP);
  assert(nav.scanCount == 0 && nav.scanSequence == 0 && isnan(nav.subgoal[0]));

  nav.sweepStage = WEDGEBUG_SWEEP_TO_START;
  in.x = 2.0f; in.timestampedRanges = true; in.rangerSequence = 1;
  in.yaw = -p.sweepHalf; in.rangerSampleYaw = 0.0f;
  in.rawFront = 2.0f; in.rawLeft = 3.0f; in.rawRight = 4.0f;
  wedgebugStep(&nav, &in, &out);
  assert(nav.sweepStage == WEDGEBUG_SWEEP_SETTLE);
  wedgebugStep(&nav, &in, &out);
  assert(nav.sweepStage == WEDGEBUG_SWEEP_SETTLE && nav.scanCount == 0);
  in.rangerSampleYaw = -p.sweepHalf;
  wedgebugStep(&nav, &in, &out);
  assert(nav.sweepStage == WEDGEBUG_SWEEP_SCAN);

  in.yaw = -.5f;  /* Delivery orientation must NOT label the measurement. */
  wedgebugStep(&nav, &in, &out);
  assert(nav.scanCount == 3 && fabsf(nav.scanAng[0]+p.sweepHalf) < 1e-6f);
  assert(nav.scanRng[0] == 2.0f);  /* Raw scan, rather than processed INFINITY. */
  wedgebugStep(&nav, &in, &out);
  assert(nav.scanCount == 3);  /* Held 10 Hz frame counted once. */
  in.yaw = p.sweepHalf;
  wedgebugStep(&nav, &in, &out);
  assert(nav.sweepCount == 0);  /* Wait for the delayed endpoint packet. */
  in.rangerSequence = 2; in.rangerSampleYaw = p.sweepHalf;
  wedgebugStep(&nav, &in, &out);
  assert(nav.mode == WEDGEBUG_GO_TO_GOAL || nav.sweepCount == 1);
  puts("C_SAFETY_NAV_PASSED");
  return 0;
}

// Host-only adapter for testing the REAL PID/LQR/mixer source, not a controller mock.
#include <string.h>
#include <stdlib.h>
#include "controller_pid.h"
#include "horizontal_lqr.h"
#include "position_controller.h"
#include "power_distribution.h"
#include "filter.h"
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif
static state_t state;
static sensorData_t sensors;
static control_t control;
static setpoint_t sp;
static lpf2pData gyroFilter[3];
static bool firstGyro;
void assertFail(char *exp, char *file, int line) { (void)exp; (void)file; (void)line; abort(); }
API void bridgeReset(int lqr) {
  memset(&state,0,sizeof(state)); memset(&sensors,0,sizeof(sensors));
  memset(&control,0,sizeof(control)); memset(&sp,0,sizeof(sp));
  horizontalLqrSetEnabled(lqr != 0); horizontalLqrSetBrake(false);
  positionControllerSetTiltLimit(10);
  controllerPidInit();
  for (int i=0;i<3;i++) lpf2pInit(&gyroFilter[i],1000,80);
  firstGyro=true;
  controllerPid(&control,&sp,&sensors,&state,0); // stock zero-thrust reset
}
API void bridgeStep(const float *s, const float *ref, int velocityMode, int body,
                    int stop, int brake, unsigned tick, float *out) {
  // s: world xyz/vxyz, legacy roll/pitch/yaw, BODY gyro xyz in deg/s.
  state.position.x=s[0]; state.position.y=s[1]; state.position.z=s[2];
  state.velocity.x=s[3]; state.velocity.y=s[4]; state.velocity.z=s[5];
  state.attitude.roll=s[6]; state.attitude.pitch=s[7]; state.attitude.yaw=s[8];
  float gyro[3];
  for (int i=0;i<3;i++) gyro[i]=firstGyro ? lpf2pReset(&gyroFilter[i],s[9+i]) :
                                                        lpf2pApply(&gyroFilter[i],s[9+i]);
  firstGyro=false;
  sensors.gyro.x=gyro[0]; sensors.gyro.y=gyro[1]; sensors.gyro.z=gyro[2];
  memset(&sp,0,sizeof(sp));
  if (!stop) {
    sp.mode.x=sp.mode.y=velocityMode ? modeVelocity : modeAbs;
    sp.mode.z=modeAbs; sp.mode.yaw=modeAbs;
    sp.position.x=ref[0]; sp.position.y=ref[1]; sp.position.z=ref[2];
    sp.velocity.x=ref[3]; sp.velocity.y=ref[4]; sp.velocity_body=body;
    sp.attitude.yaw=ref[5];
  }
  horizontalLqrSetBrake(brake != 0);
  controllerPid(&control,&sp,&sensors,&state,tick);
  motors_thrust_uncapped_t motors;
  powerDistribution(&control,&motors);
  out[0]=controllerPidGetRollReference(); out[1]=controllerPidGetPitchReference();
  out[2]=control.thrust; out[3]=control.roll; out[4]=control.pitch; out[5]=control.yaw;
  out[6]=horizontalLqrGetMode(); out[7]=horizontalLqrGetSaturated();
  out[8]=horizontalLqrGetUpdates();
  out[9]=motors.motors.m1; out[10]=motors.motors.m2;
  out[11]=motors.motors.m3; out[12]=motors.motors.m4;
  motors_thrust_pwm_t capped;
  out[17]=powerDistributionCap(&motors,&capped);
  out[13]=capped.motors.m1; out[14]=capped.motors.m2;
  out[15]=capped.motors.m3; out[16]=capped.motors.m4;
}
API void bridgeEnable(int enabled) { horizontalLqrSetEnabled(enabled != 0); }


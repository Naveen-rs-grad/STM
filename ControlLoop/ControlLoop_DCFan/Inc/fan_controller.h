#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

#include <stdbool.h>

typedef enum { FAN_MODE_OPEN_LOOP, FAN_MODE_P, FAN_MODE_PD, FAN_MODE_PID } FanControlMode;

typedef struct
{
  float kp, ki, kd;
  float dt, max_rpm, derivative_tau;
  float integral, previous_rpm, filtered_derivative;
  bool initialized;
} FanController;

void FanController_Reset(FanController *controller);
float Fan_OpenLoop(FanController *controller, float target);
float Fan_P(FanController *controller, float target, float rpm);
float Fan_PD(FanController *controller, float target, float rpm);
float Fan_PID(FanController *controller, float target, float rpm);

#endif


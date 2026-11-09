#include "fan_controller.h"

static float Limit(float value, float minimum, float maximum)
{
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

void FanController_Reset(FanController *c)
{
  c->integral = 0.0f;
  c->previous_rpm = 0.0f;
  c->filtered_derivative = 0.0f;
  c->initialized = false;
}

/* Filtered derivative on measured RPM, avoiding a target-step kick. */
static float Derivative(FanController *c, float rpm)
{
  if (c->initialized)
  {
    float rate = (rpm - c->previous_rpm) / c->dt;
    float alpha = c->dt / (c->derivative_tau + c->dt);
    c->filtered_derivative += alpha * (rate - c->filtered_derivative);
  }
  c->previous_rpm = rpm;
  c->initialized = true;
  return c->kd * c->filtered_derivative;
}

float Fan_OpenLoop(FanController *c, float target)
{
  return Limit(100.0f * target / c->max_rpm, 0.0f, 100.0f);
}

float Fan_P(FanController *c, float target, float rpm)
{
  if (target <= 0.0f)
  {
    FanController_Reset(c);
    return 0.0f;
  }
  return Limit(c->kp * (target - rpm), 0.0f, 100.0f);
}

float Fan_PD(FanController *c, float target, float rpm)
{
  if (target <= 0.0f)
  {
    FanController_Reset(c);
    return 0.0f;
  }
  float output = c->kp * (target - rpm) - Derivative(c, rpm);
  return Limit(output, 0.0f, 100.0f);
}

float Fan_PID(FanController *c, float target, float rpm)
{
  if (target <= 0.0f)
  {
    FanController_Reset(c);
    return 0.0f;
  }
  float error = target - rpm;
  float base = c->kp * error - Derivative(c, rpm);
  float increment = c->ki * error * c->dt;
  float candidate = Limit(c->integral + increment, -100.0f, 100.0f);
  float output = base + candidate;
  /* Anti-windup: integrate only when not pushing further into saturation. */
  if (!((output > 100.0f && increment > 0.0f) ||
        (output < 0.0f && increment < 0.0f)))
  {
    c->integral = candidate;
  }
  return Limit(base + c->integral, 0.0f, 100.0f);
}


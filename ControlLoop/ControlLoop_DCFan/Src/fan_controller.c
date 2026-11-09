#include "fan_controller.h"
#include <math.h>
#include <string.h>

static float Clamp(float value, float minimum, float maximum)
{
  return value < minimum ? minimum : (value > maximum ? maximum : value);
}

bool FanController_Init(FanController *controller, const FanControllerConfig *config)
{
  if ((controller == NULL) || (config == NULL) || (config->map == NULL) ||
      (config->map_count < 2U) || !isfinite(config->sample_seconds) ||
      (config->sample_seconds <= 0.0f) || !isfinite(config->kp) ||
      !isfinite(config->ki) || !isfinite(config->kd) ||
      !isfinite(config->derivative_tau) || (config->kp < 0.0f) ||
      (config->ki < 0.0f) || (config->kd < 0.0f) || (config->derivative_tau < 0.0f) ||
      !isfinite(config->duty_min) || !isfinite(config->duty_max) ||
      (config->duty_min < 0.0f) || (config->duty_max > 100.0f) ||
      (config->duty_min >= config->duty_max))
  {
    return false;
  }
  for (size_t i = 0U; i < config->map_count; i++)
  {
    if (!isfinite(config->map[i].rpm) || !isfinite(config->map[i].duty_percent) ||
        (config->map[i].rpm < 0.0f) || (config->map[i].duty_percent < 0.0f) ||
        (config->map[i].duty_percent > 100.0f) ||
        ((i > 0U) && ((config->map[i].rpm <= config->map[i - 1U].rpm) ||
                     (config->map[i].duty_percent < config->map[i - 1U].duty_percent))))
    {
      return false;
    }
  }
  memset(controller, 0, sizeof(*controller));
  controller->config = *config;
  controller->mode = FAN_MODE_OPEN_LOOP;
  return true;
}

void FanController_Reset(FanController *controller)
{
  controller->integral = 0.0f;
  controller->filtered_derivative = 0.0f;
  controller->previous_rpm = 0.0f;
  controller->previous_rpm_valid = false;
}

bool FanController_SetMode(FanController *controller, FanControlMode mode)
{
  if ((unsigned int)mode >= FAN_MODE_COUNT)
  {
    return false;
  }
  if (controller->mode != mode)
  {
    controller->mode = mode;
    FanController_Reset(controller);
  }
  return true;
}

float FanController_OpenLoopDuty(const FanController *controller, float target_rpm)
{
  const FanControllerConfig *config = &controller->config;
  if (!isfinite(target_rpm) || (target_rpm <= 0.0f))
  {
    return 0.0f;
  }
  float duty = config->map[config->map_count - 1U].duty_percent;
  if (target_rpm <= config->map[0].rpm)
  {
    duty = config->map[0].duty_percent;
  }
  else
  {
    for (size_t i = 1U; i < config->map_count; i++)
    {
      if (target_rpm <= config->map[i].rpm)
      {
        const FanMapPoint *lower = &config->map[i - 1U];
        const FanMapPoint *upper = &config->map[i];
        float fraction = (target_rpm - lower->rpm) / (upper->rpm - lower->rpm);
        duty = lower->duty_percent + fraction * (upper->duty_percent - lower->duty_percent);
        break;
      }
    }
  }
  return Clamp(duty, config->duty_min, config->duty_max);
}

float FanController_Update(FanController *controller, float target_rpm, float measured_rpm)
{
  const FanControllerConfig *config = &controller->config;
  if (!isfinite(target_rpm) || !isfinite(measured_rpm) ||
      (target_rpm <= 0.0f) || (measured_rpm < 0.0f))
  {
    FanController_Reset(controller);
    return 0.0f;
  }
  float feedforward = FanController_OpenLoopDuty(controller, target_rpm);
  if (controller->mode == FAN_MODE_OPEN_LOOP)
  {
    return feedforward;
  }
  if (!config->use_feedforward)
  {
    feedforward = 0.0f;
  }
  float error = target_rpm - measured_rpm;
  float proportional = config->kp * error;
  float derivative = 0.0f;
  if ((controller->mode == FAN_MODE_PD) || (controller->mode == FAN_MODE_PID))
  {
    if (controller->previous_rpm_valid)
    {
      /* Derivative on measurement avoids a kick when the target steps. */
      float rate = (measured_rpm - controller->previous_rpm) / config->sample_seconds;
      float alpha = config->sample_seconds / (config->derivative_tau + config->sample_seconds);
      controller->filtered_derivative += alpha * (rate - controller->filtered_derivative);
    }
    derivative = -config->kd * controller->filtered_derivative;
  }
  controller->previous_rpm = measured_rpm;
  controller->previous_rpm_valid = true;

  if (controller->mode == FAN_MODE_PID)
  {
    float increment = config->ki * error * config->sample_seconds;
    float range = config->duty_max - config->duty_min;
    float candidate = Clamp(controller->integral + increment, -range, range);
    float candidate_output = feedforward + proportional + candidate + derivative;
    /* Conditional integration: allow unwinding, block motion further into
       either output limit. The integral term is also bounded independently. */
    bool winding_up = ((candidate_output > config->duty_max) && (increment > 0.0f)) ||
                      ((candidate_output < config->duty_min) && (increment < 0.0f));
    if (!winding_up)
    {
      controller->integral = candidate;
    }
  }
  float output = feedforward + proportional + derivative;
  if (controller->mode == FAN_MODE_PID)
  {
    output += controller->integral;
  }
  if (!isfinite(output))
  {
    FanController_Reset(controller);
    return 0.0f;
  }
  return Clamp(output, config->duty_min, config->duty_max);
}

const char *FanController_ModeName(FanControlMode mode)
{
  static const char *const names[] = {"OPEN_LOOP", "P", "PD", "PID"};
  return (unsigned int)mode < FAN_MODE_COUNT ? names[mode] : "INVALID";
}

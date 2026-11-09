#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>

typedef enum
{
  FAN_MODE_OPEN_LOOP = 0,
  FAN_MODE_P,
  FAN_MODE_PD,
  FAN_MODE_PID,
  FAN_MODE_COUNT
} FanControlMode;

typedef struct
{
  float rpm;
  float duty_percent;
} FanMapPoint;

typedef struct
{
  float sample_seconds;
  float kp;                 /* duty percent / RPM */
  float ki;                 /* duty percent / (RPM * second) */
  float kd;                 /* duty percent * second / RPM */
  float derivative_tau;     /* derivative low-pass time constant, seconds */
  float duty_min;
  float duty_max;
  bool use_feedforward;     /* same open-loop baseline in P, PD and PID */
  const FanMapPoint *map;
  size_t map_count;
} FanControllerConfig;

typedef struct
{
  FanControllerConfig config;
  FanControlMode mode;
  float integral;
  float filtered_derivative;
  float previous_rpm;
  bool previous_rpm_valid;
} FanController;

bool FanController_Init(FanController *controller, const FanControllerConfig *config);
void FanController_Reset(FanController *controller);
bool FanController_SetMode(FanController *controller, FanControlMode mode);
float FanController_OpenLoopDuty(const FanController *controller, float target_rpm);
float FanController_Update(FanController *controller, float target_rpm, float measured_rpm);
const char *FanController_ModeName(FanControlMode mode);

#endif

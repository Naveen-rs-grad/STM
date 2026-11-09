#include "fan_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const FanMapPoint map[] = {{0.0f, 0.0f}, {2000.0f, 50.0f}, {4000.0f, 100.0f}};

static FanController CreateController(void)
{
  FanController result;
  const FanControllerConfig config = {
    .sample_seconds = 0.1f, .kp = 0.01f, .ki = 0.02f, .kd = 0.001f,
    .derivative_tau = 0.2f, .duty_min = 0.0f, .duty_max = 100.0f,
    .use_feedforward = true, .map = map, .map_count = 3U
  };
  assert(FanController_Init(&result, &config));
  return result;
}

static void Near(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.001f);
}

static void TestSyntheticPlant(void)
{
  /* A reference first-order model, not a claim about the measured fan.
     Deliberate map mismatch: plant gain 45 RPM/% vs feedforward map 40. */
  float final_rpm[FAN_MODE_COUNT] = {0};
  for (int mode = FAN_MODE_OPEN_LOOP; mode < FAN_MODE_COUNT; mode++)
  {
    FanController controller = CreateController();
    assert(FanController_SetMode(&controller, (FanControlMode)mode));
    float rpm = 0.0f;
    for (unsigned int tick = 0U; tick < 600U; tick++)
    {
      float duty = FanController_Update(&controller, 3500.0f, rpm);
      assert(isfinite(duty) && (duty >= 0.0f) && (duty <= 100.0f));
      rpm += 0.2f * (45.0f * duty - rpm);
    }
    final_rpm[mode] = rpm;
    Near(FanController_Update(&controller, 0.0f, rpm), 0.0f);
  }
  assert(fabsf(final_rpm[FAN_MODE_PID] - 3500.0f) < 2.0f);
  assert(fabsf(final_rpm[FAN_MODE_P] - 3500.0f) > 100.0f);
  assert(fabsf(final_rpm[FAN_MODE_PD] - final_rpm[FAN_MODE_P]) < 2.0f);
}

static void WriteReferenceTrace(const char *path)
{
  FILE *output = fopen(path, "w");
  assert(output != NULL);
  fputs("time_ms,target_rpm,actual_rpm,pwm_duty,control_mode\n", output);
  for (int mode = FAN_MODE_OPEN_LOOP; mode < FAN_MODE_COUNT; mode++)
  {
    FanController controller = CreateController();
    assert(FanController_SetMode(&controller, (FanControlMode)mode));
    float rpm = 0.0f;
    for (unsigned int tick = 0U; tick < 400U; tick++)
    {
      float target = (tick < 50U || tick >= 350U) ? 0.0f :
                     (tick < 150U || tick >= 250U ? 3500.0f : 2450.0f);
      float duty = FanController_Update(&controller, target, rpm);
      fprintf(output, "%u,%.0f,%.0f,%.1f,%s\n", (unsigned int)mode * 40000U + tick * 100U,
              (double)target, (double)rpm, (double)duty,
              FanController_ModeName((FanControlMode)mode));
      rpm += 0.2f * (45.0f * duty - rpm);
    }
  }
  fclose(output);
}

int main(int argc, char **argv)
{
  FanController controller = CreateController();
  Near(FanController_OpenLoopDuty(&controller, 3500.0f), 87.5f);
  Near(FanController_OpenLoopDuty(&controller, 5000.0f), 100.0f);
  Near(FanController_Update(&controller, 3500.0f, 0.0f), 87.5f);
  Near(FanController_Update(&controller, 3500.0f, 9000.0f), 87.5f);
  assert(FanController_SetMode(&controller, FAN_MODE_P));
  Near(FanController_Update(&controller, 3500.0f, 3500.0f), 87.5f);
  Near(FanController_Update(&controller, 3500.0f, 3000.0f), 92.5f);
  Near(FanController_Update(&controller, 3500.0f, 0.0f), 100.0f);
  Near(FanController_Update(&controller, 1000.0f, 9000.0f), 0.0f);

  assert(FanController_SetMode(&controller, FAN_MODE_PID));
  for (unsigned int i = 0U; i < 10000U; i++)
  {
    Near(FanController_Update(&controller, 5000.0f, 0.0f), 100.0f);
  }
  Near(controller.integral, 0.0f); /* no windup at the upper limit */
  for (unsigned int i = 0U; i < 10000U; i++)
  {
    Near(FanController_Update(&controller, 100.0f, 10000.0f), 0.0f);
  }
  Near(controller.integral, 0.0f); /* no windup at the lower limit */
  FanController_Reset(&controller);
  (void)FanController_Update(&controller, 2000.0f, 1900.0f);
  assert(controller.integral > 0.0f);
  float integral = controller.integral;
  assert(FanController_SetMode(&controller, FAN_MODE_PID));
  Near(controller.integral, integral); /* same mode must not reset every tick */
  assert(FanController_SetMode(&controller, FAN_MODE_PD));
  Near(controller.integral, 0.0f);
  Near(controller.filtered_derivative, 0.0f);
  assert(!controller.previous_rpm_valid);

  /* A target step with constant measurement has zero derivative kick. */
  (void)FanController_Update(&controller, 2000.0f, 1000.0f);
  (void)FanController_Update(&controller, 3500.0f, 1000.0f);
  Near(controller.filtered_derivative, 0.0f);
  (void)FanController_Update(&controller, 3500.0f, 1300.0f);
  Near(controller.filtered_derivative, 1000.0f); /* alpha=1/3, raw rate=3000 */
  (void)FanController_Update(&controller, 3500.0f, 1300.0f);
  Near(controller.filtered_derivative, 666.6667f);
  Near(FanController_Update(&controller, 0.0f, 3500.0f), 0.0f);
  Near(controller.integral, 0.0f);
  Near(controller.filtered_derivative, 0.0f);
  Near(FanController_Update(&controller, NAN, 1000.0f), 0.0f);
  Near(FanController_Update(&controller, 1000.0f, INFINITY), 0.0f);
  assert(!FanController_SetMode(&controller, (FanControlMode)99));

  controller = CreateController();
  controller.config.use_feedforward = false;
  assert(FanController_SetMode(&controller, FAN_MODE_P));
  Near(FanController_Update(&controller, 2000.0f, 1000.0f), 10.0f);
  FanControllerConfig invalid = controller.config;
  invalid.sample_seconds = 0.0f;
  assert(!FanController_Init(&controller, &invalid));
  TestSyntheticPlant();
  if (argc == 2)
  {
    WriteReferenceTrace(argv[1]);
    puts("Wrote a synthetic reference-model trace; this is not hardware data.");
  }
  invalid.sample_seconds = 0.1f;
  invalid.kp = NAN;
  assert(!FanController_Init(&controller, &invalid));
  puts("PASS: interpolation, four modes, limits, anti-windup, derivative filter, resets and invalid inputs");
  return 0;
}

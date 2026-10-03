// su_vel_from_pos.c

#include "su_vel_from_pos.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "log.h"
#include "su_params.h"

#define SU_VELOCITY_WINDOW_SIZE 5
#define SU_VELOCITY_LPF_HZ 10.0f

// Accepted external-position samples, ordered from oldest to newest.
static float su_position_window[SU_VELOCITY_WINDOW_SIZE][3];
static float su_time_window[SU_VELOCITY_WINDOW_SIZE];
static uint8_t su_window_count = 0;
static uint32_t su_last_position_sample_count = 0;
static float su_time = 0.0f;
static float su_last_velocity_update_time = 0.0f;
static bool su_velocity_initialized = false;
static float su_vel_lpf[3] = {0.0f, 0.0f, 0.0f};
static float su_vel_raw[3] = {0.0f, 0.0f, 0.0f};
static float su_pos_delta[3] = {0.0f, 0.0f, 0.0f};
static uint32_t su_velocity_rejection_count = 0;
static uint32_t su_velocity_buffer_reset_count = 0;

// 간단 1차 LPF
static inline float lpf1(float y_prev, float x, float alpha)
{
  return y_prev + alpha * (x - y_prev);
}

static bool linearRegressionVelocity(const float times[SU_VELOCITY_WINDOW_SIZE],
                                     const float positions[SU_VELOCITY_WINDOW_SIZE][3],
                                     float velocity[3])
{
  float meanTime = 0.0f;
  float meanPosition[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < SU_VELOCITY_WINDOW_SIZE; ++i) {
    meanTime += times[i];
    for (int axis = 0; axis < 3; ++axis) {
      meanPosition[axis] += positions[i][axis];
    }
  }
  meanTime /= SU_VELOCITY_WINDOW_SIZE;
  for (int axis = 0; axis < 3; ++axis) {
    meanPosition[axis] /= SU_VELOCITY_WINDOW_SIZE;
  }

  float timeVariance = 0.0f;
  float covariance[3] = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < SU_VELOCITY_WINDOW_SIZE; ++i) {
    const float centeredTime = times[i] - meanTime;
    timeVariance += centeredTime * centeredTime;
    for (int axis = 0; axis < 3; ++axis) {
      covariance[axis] += centeredTime * (positions[i][axis] - meanPosition[axis]);
    }
  }
  if (!isfinite(timeVariance) || timeVariance < 1.0e-8f) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    velocity[axis] = covariance[axis] / timeVariance;
    if (!isfinite(velocity[axis])) {
      return false;
    }
  }
  return true;
}

static void updateLowPassFilter(const float updateDt)
{
  const float twoPi = 6.28318530718f;
  float alpha = 1.0f - expf(-twoPi * SU_VELOCITY_LPF_HZ * updateDt);
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  for (int axis = 0; axis < 3; ++axis) {
    su_vel_lpf[axis] = lpf1(su_vel_lpf[axis], su_vel_raw[axis], alpha);
  }
}

// ==============================
// 초기화
// ==============================
void suVelFromPosInit(void)
{
  for (int i = 0; i < SU_VELOCITY_WINDOW_SIZE; ++i) {
    su_time_window[i] = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
      su_position_window[i][axis] = 0.0f;
    }
  }
  for (int i = 0; i < 3; ++i) {
    su_vel_lpf[i]  = 0.0f;
    su_vel_raw[i]  = 0.0f;
    su_pos_delta[i] = 0.0f;
  }
  su_window_count = 0;
  su_last_position_sample_count = 0;
  su_time = 0.0f;
  su_last_velocity_update_time = 0.0f;
  su_velocity_initialized = false;
  su_velocity_rejection_count = 0;
  su_velocity_buffer_reset_count++;
}

// ==============================
// 100 Hz: consume at most one new extPos, then always advance the 10 Hz LPF.
// Identical-valued samples are valid samples; no new packet means raw velocity hold.
// ==============================
void suVelFromPosUpdate(const float position_world[3], uint32_t position_sample_count,
                        float update_dt)
{
  if (!isfinite(update_dt) || update_dt <= 0.0f) {
    su_velocity_rejection_count++;
    return;
  }
  su_time += update_dt;

  const bool hasNewSample = position_sample_count != 0 &&
                            position_sample_count != su_last_position_sample_count;
  if (hasNewSample) {
    su_last_position_sample_count = position_sample_count;
    const bool finitePosition = position_world && isfinite(position_world[0]) &&
                                isfinite(position_world[1]) && isfinite(position_world[2]);
    if (!finitePosition) {
      su_velocity_rejection_count++;
    } else if (su_window_count < SU_VELOCITY_WINDOW_SIZE) {
      const int index = su_window_count;
      su_time_window[index] = su_time;
      for (int axis = 0; axis < 3; ++axis) {
        su_position_window[index][axis] = position_world[axis];
        if (index > 0) {
          su_pos_delta[axis] = position_world[axis] - su_position_window[index - 1][axis];
        }
      }
      su_window_count++;
      if (su_window_count == SU_VELOCITY_WINDOW_SIZE) {
        float candidate_velocity[3];
        if (linearRegressionVelocity(su_time_window, su_position_window, candidate_velocity)) {
          for (int axis = 0; axis < 3; ++axis) {
            su_vel_raw[axis] = candidate_velocity[axis];
          }
          su_last_velocity_update_time = su_time;
          su_velocity_initialized = true;
        } else {
          su_velocity_rejection_count++;
        }
      }
    } else {
      float candidate_times[SU_VELOCITY_WINDOW_SIZE];
      float candidate_positions[SU_VELOCITY_WINDOW_SIZE][3];
      for (int i = 0; i < SU_VELOCITY_WINDOW_SIZE - 1; ++i) {
        candidate_times[i] = su_time_window[i + 1];
        for (int axis = 0; axis < 3; ++axis) {
          candidate_positions[i][axis] = su_position_window[i + 1][axis];
        }
      }
      candidate_times[SU_VELOCITY_WINDOW_SIZE - 1] = su_time;
      for (int axis = 0; axis < 3; ++axis) {
        candidate_positions[SU_VELOCITY_WINDOW_SIZE - 1][axis] = position_world[axis];
      }

      float candidate_velocity[3];
      const float gate_dt = su_time - su_last_velocity_update_time;
      bool accepted = linearRegressionVelocity(candidate_times, candidate_positions,
                                               candidate_velocity) &&
                      isfinite(gate_dt) && gate_dt > 0.0f;
      if (accepted && su_velocity_initialized) {
        const float dvx = candidate_velocity[0] - su_vel_raw[0];
        const float dvy = candidate_velocity[1] - su_vel_raw[1];
        const float dvz = candidate_velocity[2] - su_vel_raw[2];
        const float xy_limit = fmaxf(0.0f, su_velocity_accel_xy_max) * gate_dt;
        const float z_limit = fmaxf(0.0f, su_velocity_accel_z_max) * gate_dt;
        accepted = hypotf(dvx, dvy) <= xy_limit && fabsf(dvz) <= z_limit;
      }

      if (accepted) {
        for (int i = 0; i < SU_VELOCITY_WINDOW_SIZE; ++i) {
          su_time_window[i] = candidate_times[i];
          for (int axis = 0; axis < 3; ++axis) {
            su_position_window[i][axis] = candidate_positions[i][axis];
          }
        }
        for (int axis = 0; axis < 3; ++axis) {
          su_pos_delta[axis] = position_world[axis] -
                               su_position_window[SU_VELOCITY_WINDOW_SIZE - 2][axis];
          su_vel_raw[axis] = candidate_velocity[axis];
        }
        su_last_velocity_update_time = su_time;
        su_velocity_initialized = true;
      } else {
        // A rejected outlier never contaminates the accepted regression window.
        su_velocity_rejection_count++;
      }
    }
  }

  updateLowPassFilter(update_dt);
}

// ==============================
// World-frame velocity 읽기
// ==============================
void suVelFromPosGetWorld(float out_vW[3])
{
  if (!out_vW) return;

  out_vW[0] = su_vel_lpf[0];
  out_vW[1] = su_vel_lpf[1];
  out_vW[2] = su_vel_lpf[2];

  if (!isfinite(out_vW[0])) out_vW[0] = 0.0f;
  if (!isfinite(out_vW[1])) out_vW[1] = 0.0f;
  if (!isfinite(out_vW[2])) out_vW[2] = 0.0f;
}

// ==============================
// 로그 그룹 (ROS2에서 velocity만 따로 로깅용)
// ==============================
LOG_GROUP_START(suVelFromPos)

LOG_ADD(LOG_FLOAT, vx, &su_vel_lpf[0])   // [m/s]
LOG_ADD(LOG_FLOAT, vy, &su_vel_lpf[1])   // [m/s]
LOG_ADD(LOG_FLOAT, vz, &su_vel_lpf[2])   // [m/s]
LOG_ADD(LOG_FLOAT, rawVx, &su_vel_raw[0])
LOG_ADD(LOG_FLOAT, rawVy, &su_vel_raw[1])
LOG_ADD(LOG_FLOAT, rawVz, &su_vel_raw[2])
LOG_ADD(LOG_FLOAT, dPosX, &su_pos_delta[0])
LOG_ADD(LOG_FLOAT, dPosY, &su_pos_delta[1])
LOG_ADD(LOG_FLOAT, dPosZ, &su_pos_delta[2])
LOG_ADD(LOG_UINT32, rejectCnt, &su_velocity_rejection_count)
LOG_ADD(LOG_UINT32, resetCnt, &su_velocity_buffer_reset_count)

LOG_GROUP_STOP(suVelFromPos)

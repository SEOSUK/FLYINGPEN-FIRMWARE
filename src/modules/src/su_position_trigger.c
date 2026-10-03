#include "su_position_trigger.h"

#include "app_channel.h"
#include "log.h"
#include "param_logic.h"
#include "su_params.h"
#include "su_wrench_observer.h"
#include <math.h>

#define SU_POSITION_TRIGGER_MAGIC   0xA5
#define SU_POSITION_TRIGGER_VERSION 0x02
#define SU_HOVER_CALIBRATION_TRIGGER_MAGIC   0xA6
#define SU_HOVER_CALIBRATION_TRIGGER_VERSION 0x01

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint8_t positionMode;
  uint8_t reserved;
  uint8_t commandReference;
  float forceDesired;
} su_position_trigger_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
} su_hover_calibration_packet_t;

typedef enum {
  SU_CALIB_IDLE = 0,
  // State value 1 was the removed WAIT_HOVER state. Keep the remaining wire
  // values stable so older ROS clients and logs decode calibration correctly.
  SU_CALIB_IMU_TRIM = 2,
  SU_CALIB_COM_COLLECT = 3,
  SU_CALIB_DONE = 4,
  SU_CALIB_ERROR = 5,
} su_calib_state_t;

// Hardware-tunable calibration constants.
#define SU_CALIB_IMU_LPF_CUTOFF_HZ       0.5f
#define SU_CALIB_TRIM_UPDATE_PERIOD_S     0.2f
#define SU_CALIB_TRIM_GAIN                0.2f
#define SU_CALIB_TRIM_MAX_STEP_DEG        0.2f
#define SU_CALIB_RESIDUAL_TOL_DEG         0.25f
#define SU_CALIB_CONVERGENCE_TIME_S       1.5f
#define SU_CALIB_MAX_TRIM_DEG              5.0f
#define SU_CALIB_IMU_TIMEOUT_S            30.0f
#define SU_CALIB_ACC_MAG_MIN_G             0.3f
#define SU_CALIB_ACC_MAG_MAX_G             2.0f
#define SU_CALIB_COM_WINDOW_S              2.0f
#define SU_CALIB_COM_MIN_SAMPLES          20u
#define SU_CALIB_MIN_HOVER_THRUST_N        0.05f

static uint8_t calibState = SU_CALIB_IDLE;
static uint16_t calibRunId = 0;
static float calibFinalAccTrimRoll = 0.0f;
static float calibFinalAccTrimPitch = 0.0f;
static float calibFinalComOffX = 0.0f;
static float calibFinalComOffY = 0.0f;
static float calibStateTime = 0.0f;
static float calibTrimUpdateTime = 0.0f;
static float calibConvergedTime = 0.0f;
static float calibAccLpf[3] = {0.0f, 0.0f, 0.0f};
static bool calibAccLpfInitialized = false;
static float calibComThrustSum = 0.0f;
static float calibComTauXSum = 0.0f;
static float calibComTauYSum = 0.0f;
static uint32_t calibComSampleCount = 0;
static paramVarId_t calibAccTrimRollId;
static paramVarId_t calibAccTrimPitchId;

static uint8_t currentPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t currentCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
static float currentForceDesired = 0.0f;

static uint8_t sanitizePositionMode(const uint8_t mode)
{
  return (mode == SU_POSITION_MODE_VELOCITY) ? SU_POSITION_MODE_VELOCITY : SU_POSITION_MODE_POSITION;
}

static uint8_t sanitizeCommandReference(const uint8_t reference)
{
  return (reference == SU_COMMAND_REFERENCE_END_EFFECTOR) ?
    SU_COMMAND_REFERENCE_END_EFFECTOR : SU_COMMAND_REFERENCE_DRONE;
}

void suPositionTriggerInit(void)
{
  currentPositionMode = SU_POSITION_MODE_POSITION;
  currentCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
  currentForceDesired = 0.0f;
  calibState = SU_CALIB_IDLE;
  calibAccTrimRollId = paramGetVarId("imu_sensors", "accTrimRoll");
  calibAccTrimPitchId = paramGetVarId("imu_sensors", "accTrimPitch");
}

static float clampf(const float value, const float low, const float high)
{
  return fminf(high, fmaxf(low, value));
}

static void calibrationStart(void)
{
  if (calibState == SU_CALIB_IMU_TRIM || calibState == SU_CALIB_COM_COLLECT) {
    return;
  }
  ++calibRunId;
  if (!PARAM_VARID_IS_VALID(calibAccTrimRollId) ||
      !PARAM_VARID_IS_VALID(calibAccTrimPitchId)) {
    calibState = SU_CALIB_ERROR;
    return;
  }
  calibState = SU_CALIB_IMU_TRIM;
  calibStateTime = 0.0f;
  calibTrimUpdateTime = 0.0f;
  calibConvergedTime = 0.0f;
  calibAccLpfInitialized = false;
  calibComThrustSum = 0.0f;
  calibComTauXSum = 0.0f;
  calibComTauYSum = 0.0f;
  calibComSampleCount = 0;
}

void suPositionTriggerUpdate(void)
{
  uint8_t packetBuffer[APPCHANNEL_MTU];
  size_t packetLength = 0;

  while ((packetLength = appchannelReceiveDataPacket(packetBuffer, sizeof(packetBuffer), 0)) > 0) {
    if (packetLength >= sizeof(su_position_trigger_packet_t)) {
      const su_position_trigger_packet_t* packet = (const su_position_trigger_packet_t*)packetBuffer;
      if (packet->magic == SU_POSITION_TRIGGER_MAGIC && packet->version == SU_POSITION_TRIGGER_VERSION) {
        currentPositionMode = sanitizePositionMode(packet->positionMode);
        currentCommandReference = sanitizeCommandReference(packet->commandReference);
        currentForceDesired = isfinite(packet->forceDesired) ? packet->forceDesired : 0.0f;
        continue;
      }
    }

    if (packetLength >= sizeof(su_hover_calibration_packet_t)) {
      const su_hover_calibration_packet_t* packet = (const su_hover_calibration_packet_t*)packetBuffer;
      if (packet->magic == SU_HOVER_CALIBRATION_TRIGGER_MAGIC &&
          packet->version == SU_HOVER_CALIBRATION_TRIGGER_VERSION) {
        calibrationStart();
      }
    }
  }
}

void suPositionTriggerCalibrationUpdate(const Axis3f *postTrimAcc, float dt)
{
  if (!postTrimAcc || !isfinite(dt) || dt <= 0.0f || dt > 0.1f ||
      calibState == SU_CALIB_IDLE || calibState == SU_CALIB_DONE || calibState == SU_CALIB_ERROR) {
    return;
  }

  float motorThrust[4];
  float bodyTorque[3];
  suWrenchObserverGetMotorThrust(motorThrust);
  suWrenchObserverGetBodyInputTorque(bodyTorque);
  const float totalThrust = motorThrust[0] + motorThrust[1] + motorThrust[2] + motorThrust[3];
  calibStateTime += dt;

  if (calibState == SU_CALIB_IMU_TRIM) {
    const float acc[3] = {postTrimAcc->x, postTrimAcc->y, postTrimAcc->z};
    const float accMag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (!isfinite(accMag) || accMag < SU_CALIB_ACC_MAG_MIN_G || accMag > SU_CALIB_ACC_MAG_MAX_G) {
      calibConvergedTime = 0.0f;
      if (calibStateTime >= SU_CALIB_IMU_TIMEOUT_S) calibState = SU_CALIB_ERROR;
      return;
    }
    if (!calibAccLpfInitialized) {
      for (int i = 0; i < 3; ++i) calibAccLpf[i] = acc[i];
      calibAccLpfInitialized = true;
    } else {
      const float tau = 1.0f / (2.0f * (float)M_PI * SU_CALIB_IMU_LPF_CUTOFF_HZ);
      const float alpha = dt / (tau + dt);
      for (int i = 0; i < 3; ++i) calibAccLpf[i] += alpha * (acc[i] - calibAccLpf[i]);
    }

    const float residualRollDeg = atan2f(calibAccLpf[1], calibAccLpf[2]) * 180.0f / (float)M_PI;
    const float residualPitchDeg = atan2f(-calibAccLpf[0],
      sqrtf(calibAccLpf[1]*calibAccLpf[1] + calibAccLpf[2]*calibAccLpf[2])) * 180.0f / (float)M_PI;
    if (!isfinite(residualRollDeg) || !isfinite(residualPitchDeg)) {
      calibState = SU_CALIB_ERROR;
      return;
    }

    if (fabsf(residualRollDeg) <= SU_CALIB_RESIDUAL_TOL_DEG &&
        fabsf(residualPitchDeg) <= SU_CALIB_RESIDUAL_TOL_DEG) {
      calibConvergedTime += dt;
    } else {
      calibConvergedTime = 0.0f;
    }

    calibTrimUpdateTime += dt;
    if (calibTrimUpdateTime >= SU_CALIB_TRIM_UPDATE_PERIOD_S) {
      calibTrimUpdateTime = 0.0f;
      const float currentRoll = paramGetFloat(calibAccTrimRollId);
      const float currentPitch = paramGetFloat(calibAccTrimPitchId);
      // Actual transform gives roll residual=(sensor roll-trim roll), while
      // pitch residual=(sensor pitch+trim pitch); hence opposite pitch sign.
      const float rollStep = clampf(SU_CALIB_TRIM_GAIN * residualRollDeg,
                                    -SU_CALIB_TRIM_MAX_STEP_DEG, SU_CALIB_TRIM_MAX_STEP_DEG);
      const float pitchStep = clampf(-SU_CALIB_TRIM_GAIN * residualPitchDeg,
                                     -SU_CALIB_TRIM_MAX_STEP_DEG, SU_CALIB_TRIM_MAX_STEP_DEG);
      const float nextRoll = currentRoll + rollStep;
      const float nextPitch = currentPitch + pitchStep;
      if (!isfinite(nextRoll) || !isfinite(nextPitch) ||
          fabsf(nextRoll) > SU_CALIB_MAX_TRIM_DEG || fabsf(nextPitch) > SU_CALIB_MAX_TRIM_DEG) {
        calibState = SU_CALIB_ERROR;
        return;
      }
      paramSetFloat(calibAccTrimRollId, nextRoll);
      paramSetFloat(calibAccTrimPitchId, nextPitch);
    }

    if (calibConvergedTime >= SU_CALIB_CONVERGENCE_TIME_S) {
      calibFinalAccTrimRoll = paramGetFloat(calibAccTrimRollId);
      calibFinalAccTrimPitch = paramGetFloat(calibAccTrimPitchId);
      calibState = SU_CALIB_COM_COLLECT;
      calibStateTime = 0.0f;
      calibComThrustSum = calibComTauXSum = calibComTauYSum = 0.0f;
      calibComSampleCount = 0;
    } else if (calibStateTime >= SU_CALIB_IMU_TIMEOUT_S) {
      calibState = SU_CALIB_ERROR;
    }
    return;
  }

  if (calibState == SU_CALIB_COM_COLLECT) {
    if (isfinite(totalThrust) && isfinite(bodyTorque[0]) && isfinite(bodyTorque[1])) {
      calibComThrustSum += totalThrust;
      calibComTauXSum += bodyTorque[0];
      calibComTauYSum += bodyTorque[1];
      ++calibComSampleCount;
    }
    if (calibStateTime >= SU_CALIB_COM_WINDOW_S) {
      if (calibComSampleCount < SU_CALIB_COM_MIN_SAMPLES) {
        calibState = SU_CALIB_ERROR;
        return;
      }
      const float invCount = 1.0f / (float)calibComSampleCount;
      const float hoverThrust = calibComThrustSum * invCount;
      const float tauX = calibComTauXSum * invCount;
      const float tauY = calibComTauYSum * invCount;
      if (!isfinite(hoverThrust) || hoverThrust <= SU_CALIB_MIN_HOVER_THRUST_N ||
          !isfinite(tauX) || !isfinite(tauY)) {
        calibState = SU_CALIB_ERROR;
        return;
      }
      const float deltaComX = -tauY / hoverThrust;
      const float deltaComY = tauX / hoverThrust;
      su_com_offset_x = su_com_offset_x - deltaComX;
      su_com_offset_y = su_com_offset_y - deltaComY;
      if (!isfinite(su_com_offset_x) || !isfinite(su_com_offset_y)) {
        calibState = SU_CALIB_ERROR;
        return;
      }
      calibFinalComOffX = su_com_offset_x;
      calibFinalComOffY = su_com_offset_y;
      calibState = SU_CALIB_DONE;
    }
  }
}

uint8_t suPositionTriggerGetMode(void)
{
  return currentPositionMode;
}

uint8_t suPositionTriggerGetCommandReference(void)
{
  return currentCommandReference;
}

float suPositionTriggerGetForceDesired(void)
{
  return currentForceDesired;
}

LOG_GROUP_START(suCalib)
LOG_ADD(LOG_UINT8, state, &calibState)
LOG_ADD(LOG_UINT16, runId, &calibRunId)
LOG_ADD(LOG_FLOAT, accRoll, &calibFinalAccTrimRoll)
LOG_ADD(LOG_FLOAT, accPitch, &calibFinalAccTrimPitch)
LOG_ADD(LOG_FLOAT, comX, &calibFinalComOffX)
LOG_ADD(LOG_FLOAT, comY, &calibFinalComOffY)
LOG_GROUP_STOP(suCalib)

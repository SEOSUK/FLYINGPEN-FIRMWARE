#include "su_position_reference.h"

#include <math.h>
#include <stdbool.h>

#include "log.h"
#include "su_params.h"
#include "su_position_trigger.h"
#include "su_thrust_effectiveness.h"
#include "su_wrench_observer.h"

#define SU_POSITION_VELOCITY_RATE_HZ 100
#define SU_RAD2DEG (180.0f / (float)M_PI)
#define SU_YAW_ALIGN_SAT_DEG 70.0f
#define SU_NORMAL_VELOCITY_LEAKAGE_LPF_HZ 0.4f

static bool referenceInitialized = false;
static point_t referencePosition;
static point_t eeReferenceLog;
static float referenceBaseYawDeg = 0.0f;
static float referenceYawCorrectionDeg = 0.0f;
static uint8_t lastPositionMode = SU_POSITION_MODE_POSITION;
static uint8_t lastCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
static float referenceYawDegLog = 0.0f;
static float normalEstimatorMatrix[3][3];
static float normalForceEvidenceWorld[3] = {-1.0f, 0.0f, 0.0f};
static float normalProjectedCandidateWorld[3] = {-1.0f, 0.0f, 0.0f};
static float normalEstimateWorld[3] = {-1.0f, 0.0f, 0.0f};
static float normalEstimateDotWorld[3] = {0.0f, 0.0f, 0.0f};
static float contactVelRawWorld[3] = {0.0f, 0.0f, 0.0f};
static float filteredContactVelWorld[3] = {0.0f, 0.0f, 0.0f};
static float velocityModulationNormalDotWorld[3] = {0.0f, 0.0f, 0.0f};
static float curvatureEstimate = 0.0f;
static float velocityModulationAlpha = 1.0f;
static float normalVelocityLeakageRaw = 0.0f;
static float normalVelocityLeakageLpf = 0.0f;
static bool contactVelRawValid = false;
static bool filteredContactVelInitialized = false;
static bool velocityModulationNormalDotInitialized = false;
static bool normalEstimateInitialized = false;
static bool normalEstimateValid = false;
static bool curvatureEstimateValid = false;
static bool normalVelocityLeakageInitialized = false;

static float wrapAngleDeg180(const float angleDeg);

static float getReferenceYawDeg(void)
{
  return wrapAngleDeg180(referenceBaseYawDeg + referenceYawCorrectionDeg);
}

static bool isPositionSetpointCandidate(const setpoint_t *setpoint)
{
  if (!setpoint) {
    return false;
  }

  return setpoint->mode.x == modeAbs &&
         setpoint->mode.y == modeAbs &&
         setpoint->mode.z == modeAbs &&
         setpoint->mode.yaw == modeAbs &&
         setpoint->mode.roll == modeDisable &&
         setpoint->mode.pitch == modeDisable &&
         setpoint->mode.quat == modeDisable;
}

static void rotateBodyOffsetToWorld(const float yawDeg, point_t *offsetWorld)
{
  if (!offsetWorld) {
    return;
  }

  const float yawRad = yawDeg * ((float)M_PI / 180.0f);
  const float cosYaw = cosf(yawRad);
  const float sinYaw = sinf(yawRad);

  offsetWorld->x = cosYaw * su_r_offset_x - sinYaw * su_r_offset_y;
  offsetWorld->y = sinYaw * su_r_offset_x + cosYaw * su_r_offset_y;
  offsetWorld->z = su_r_offset_z;
}

static void convertReferencePosition(point_t *position, const uint8_t fromReference, const uint8_t toReference, const float yawDeg)
{
  if (!position || fromReference == toReference) {
    return;
  }

  point_t offsetWorld;
  rotateBodyOffsetToWorld(yawDeg, &offsetWorld);

  if (fromReference == SU_COMMAND_REFERENCE_DRONE && toReference == SU_COMMAND_REFERENCE_END_EFFECTOR) {
    position->x += offsetWorld.x;
    position->y += offsetWorld.y;
    position->z += offsetWorld.z;
  } else if (fromReference == SU_COMMAND_REFERENCE_END_EFFECTOR && toReference == SU_COMMAND_REFERENCE_DRONE) {
    position->x -= offsetWorld.x;
    position->y -= offsetWorld.y;
    position->z -= offsetWorld.z;
  }
}

static void initializeReference(const setpoint_t *setpoint, const state_t *state, const uint8_t commandReference)
{
  float initialYawDeg = 0.0f;

  if (setpoint && isPositionSetpointCandidate(setpoint)) {
    referencePosition = setpoint->position;
    initialYawDeg = setpoint->attitude.yaw;
  } else if (state) {
    referencePosition = state->position;
    initialYawDeg = state->attitude.yaw;
  } else {
    referencePosition.x = 0.0f;
    referencePosition.y = 0.0f;
    referencePosition.z = 0.0f;
    initialYawDeg = 0.0f;
  }

  referenceBaseYawDeg = wrapAngleDeg180(initialYawDeg);
  referenceYawCorrectionDeg = 0.0f;
  convertReferencePosition(&referencePosition, SU_COMMAND_REFERENCE_DRONE, commandReference, getReferenceYawDeg());
  referenceInitialized = true;
}

static void writeReferenceToSetpoint(setpoint_t *setpoint, const uint8_t commandReference)
{
  point_t droneReference = referencePosition;
  point_t eeReference = referencePosition;
  const float referenceYawDeg = getReferenceYawDeg();
  referenceYawDegLog = referenceYawDeg;

  convertReferencePosition(&droneReference, commandReference, SU_COMMAND_REFERENCE_DRONE, referenceYawDeg);
  convertReferencePosition(&eeReference, commandReference, SU_COMMAND_REFERENCE_END_EFFECTOR, referenceYawDeg);
  eeReferenceLog = eeReference;

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeAbs;

  setpoint->position = droneReference;
  setpoint->attitude.yaw = referenceYawDeg;
}

static float clampPositive(const float value)
{
  return (value > 0.0f) ? value : 0.0f;
}

static bool isLpfBypassed(const float cutoffHz)
{
  return !isfinite(cutoffHz) || cutoffHz <= 0.0f;
}

static float wrapAngleDeg180(const float angleDeg)
{
  float wrapped = fmodf(angleDeg + 180.0f, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped - 180.0f;
}

static float clampSymmetric(const float value, const float limit)
{
  const float positiveLimit = clampPositive(limit);
  if (value > positiveLimit) {
    return positiveLimit;
  }
  if (value < -positiveLimit) {
    return -positiveLimit;
  }
  return value;
}

static float vec3Dot(const float a[3], const float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vec3Copy(float out[3], const float in[3])
{
  out[0] = in[0];
  out[1] = in[1];
  out[2] = in[2];
}

static void vec3Cross(float out[3], const float a[3], const float b[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static float vec3Norm(const float v[3])
{
  return sqrtf(vec3Dot(v, v));
}

static bool vec3Normalize(float out[3], const float in[3], const float eps)
{
  const float norm = vec3Norm(in);
  if (!isfinite(norm) || norm <= eps) {
    return false;
  }

  const float invNorm = 1.0f / norm;
  out[0] = in[0] * invNorm;
  out[1] = in[1] * invNorm;
  out[2] = in[2] * invNorm;
  return true;
}

static bool vec3IsFinite(const float v[3])
{
  return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static void getFixedNormalWorld(float outNormal[3])
{
  if (!outNormal) {
    return;
  }

  outNormal[0] = -1.0f;
  outNormal[1] = 0.0f;
  outNormal[2] = 0.0f;
}

static bool isNormalEstimatorEnabled(void)
{
  return su_normal_estimation != 0;
}

static bool isContactFrameControlEnabled(void)
{
  return isNormalEstimatorEnabled();
}

static bool isAdvancedVelocityControlMode(const uint8_t positionMode)
{
  return positionMode == SU_POSITION_MODE_VELOCITY;
}

static void resetContactVelocityFilter(void)
{
  contactVelRawWorld[0] = 0.0f;
  contactVelRawWorld[1] = 0.0f;
  contactVelRawWorld[2] = 0.0f;
  contactVelRawValid = false;
  filteredContactVelWorld[0] = 0.0f;
  filteredContactVelWorld[1] = 0.0f;
  filteredContactVelWorld[2] = 0.0f;
  filteredContactVelInitialized = false;
}

static void resetNormalEstimator(void)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      normalEstimatorMatrix[row][col] = 0.0f;
    }
  }

  getFixedNormalWorld(normalEstimateWorld);
  normalEstimateDotWorld[0] = 0.0f;
  normalEstimateDotWorld[1] = 0.0f;
  normalEstimateDotWorld[2] = 0.0f;
  getFixedNormalWorld(normalForceEvidenceWorld);
  getFixedNormalWorld(normalProjectedCandidateWorld);
  normalVelocityLeakageRaw = 0.0f;
  normalVelocityLeakageLpf = 0.0f;
  normalEstimateInitialized = false;
  normalEstimateValid = false;
  normalVelocityLeakageInitialized = false;
}

static void resetVelocityModulation(void)
{
  velocityModulationNormalDotWorld[0] = 0.0f;
  velocityModulationNormalDotWorld[1] = 0.0f;
  velocityModulationNormalDotWorld[2] = 0.0f;
  velocityModulationNormalDotInitialized = false;
  curvatureEstimate = 0.0f;
  curvatureEstimateValid = false;
  velocityModulationAlpha = 1.0f;
}

static void getEstimatedNormalWorld(float outNormal[3])
{
  if (!outNormal) {
    return;
  }

  if (isNormalEstimatorEnabled() && normalEstimateInitialized) {
    vec3Copy(outNormal, normalEstimateWorld);
    return;
  }

  getFixedNormalWorld(outNormal);
}

static void getControlNormalWorld(float outNormal[3])
{
  if (!outNormal) {
    return;
  }

  if (isContactFrameControlEnabled()) {
    getEstimatedNormalWorld(outNormal);
  } else {
    getFixedNormalWorld(outNormal);
  }
}

static bool updateNormalFromDirectionalMemory(const float candidate[3],
                                              const float correctedForce[3],
                                              const float dt)
{
  bool updateValid = true;

  if (!normalEstimateInitialized) {
    vec3Copy(normalEstimateWorld, candidate);
    normalEstimateDotWorld[0] = 0.0f;
    normalEstimateDotWorld[1] = 0.0f;
    normalEstimateDotWorld[2] = 0.0f;
    normalEstimateInitialized = true;
  } else {
    const float lnN[3] = {
      normalEstimatorMatrix[0][0] * normalEstimateWorld[0] +
        normalEstimatorMatrix[0][1] * normalEstimateWorld[1] +
        normalEstimatorMatrix[0][2] * normalEstimateWorld[2],
      normalEstimatorMatrix[1][0] * normalEstimateWorld[0] +
        normalEstimatorMatrix[1][1] * normalEstimateWorld[1] +
        normalEstimatorMatrix[1][2] * normalEstimateWorld[2],
      normalEstimatorMatrix[2][0] * normalEstimateWorld[0] +
        normalEstimatorMatrix[2][1] * normalEstimateWorld[1] +
        normalEstimatorMatrix[2][2] * normalEstimateWorld[2],
    };
    const float scalar = vec3Dot(normalEstimateWorld, lnN);
    const float gamma = clampPositive(su_normal_gamma);

    for (int i = 0; i < 3; ++i) {
      normalEstimateDotWorld[i] = gamma *
        (lnN[i] - normalEstimateWorld[i] * scalar);
    }

    const float estimateNext[3] = {
      normalEstimateWorld[0] + dt * normalEstimateDotWorld[0],
      normalEstimateWorld[1] + dt * normalEstimateDotWorld[1],
      normalEstimateWorld[2] + dt * normalEstimateDotWorld[2],
    };
    float estimateNormalized[3];
    if (vec3Normalize(estimateNormalized, estimateNext, 1e-6f)) {
      vec3Copy(normalEstimateWorld, estimateNormalized);
    } else {
      vec3Copy(normalEstimateWorld, candidate);
      normalEstimateDotWorld[0] = 0.0f;
      normalEstimateDotWorld[1] = 0.0f;
      normalEstimateDotWorld[2] = 0.0f;
      updateValid = false;
    }
  }

  if (vec3Dot(normalEstimateWorld, correctedForce) < 0.0f) {
    for (int i = 0; i < 3; ++i) {
      normalEstimateWorld[i] = -normalEstimateWorld[i];
      normalEstimateDotWorld[i] = -normalEstimateDotWorld[i];
    }
  }

  return updateValid && vec3IsFinite(normalEstimateWorld) &&
         vec3IsFinite(normalEstimateDotWorld);
}

static void updateContactPointVelocity(void)
{
  float contactVelWorld[3];
  suWrenchObserverGetContactPointVelocityWorld(contactVelWorld);
  vec3Copy(contactVelRawWorld, contactVelWorld);
  contactVelRawValid = vec3IsFinite(contactVelRawWorld);
  if (!contactVelRawValid) {
    return;
  }

  if (!filteredContactVelInitialized) {
    vec3Copy(filteredContactVelWorld, contactVelRawWorld);
    filteredContactVelInitialized = true;
  } else {
    const float cutoffHz = su_contact_velocity_lpf_hz;
    if (isLpfBypassed(cutoffHz)) {
      vec3Copy(filteredContactVelWorld, contactVelRawWorld);
    } else {
      const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
      const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
      const float alpha = dt / (tau + dt);
      for (int i = 0; i < 3; ++i) {
        filteredContactVelWorld[i] +=
          alpha * (contactVelRawWorld[i] - filteredContactVelWorld[i]);
      }
    }
  }
}

static void updateNormalEstimator(void)
{
  normalEstimateValid = false;

  if (!contactVelRawValid) {
    return;
  }

  if (!isNormalEstimatorEnabled()) {
    getFixedNormalWorld(normalEstimateWorld);
    normalEstimateDotWorld[0] = 0.0f;
    normalEstimateDotWorld[1] = 0.0f;
    normalEstimateDotWorld[2] = 0.0f;
    normalEstimateInitialized = false;
    return;
  }

  float worldForce[3] = {0.0f, 0.0f, 0.0f};
  suThrustEffectivenessGetContactForceWorld(worldForce);

  const float epsilonF = clampPositive(su_normal_epsilon_f);
  if (vec3Norm(worldForce) <= epsilonF) {
    return;
  }

  float qf[3];
  if (!vec3Normalize(qf, worldForce, epsilonF)) {
    return;
  }
  vec3Copy(normalForceEvidenceWorld, qf);

  float qg[3];
  if (su_normal_epsilon_v <= 0.0f) {
    vec3Copy(qg, qf);
  } else {
    const float epsilonV = clampPositive(su_normal_epsilon_v);
    const float velNormSq = vec3Dot(filteredContactVelWorld, filteredContactVelWorld);
    const float velProjScale = vec3Dot(filteredContactVelWorld, qf) / (velNormSq + epsilonV);

    qg[0] = qf[0] - filteredContactVelWorld[0] * velProjScale;
    qg[1] = qf[1] - filteredContactVelWorld[1] * velProjScale;
    qg[2] = qf[2] - filteredContactVelWorld[2] * velProjScale;
  }

  float nRaw[3];
  if (!vec3Normalize(nRaw, qg, 1e-6f)) {
    return;
  }
  vec3Copy(normalProjectedCandidateWorld, nRaw);

  float projectorRaw[3][3];
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      projectorRaw[row][col] = nRaw[row] * nRaw[col];
    }
  }

  const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
  const float beta = clampPositive(su_normal_beta);
  const float sigma = beta;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const float matrixDot = -beta * normalEstimatorMatrix[row][col] + sigma * projectorRaw[row][col];
      normalEstimatorMatrix[row][col] += dt * matrixDot;
    }
  }

  normalEstimateValid = updateNormalFromDirectionalMemory(nRaw, worldForce, dt);

  const float cutoffHz = SU_NORMAL_VELOCITY_LEAKAGE_LPF_HZ;
  const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
  const float alpha = dt / (tau + dt);

  normalVelocityLeakageRaw = fabsf(
    vec3Dot(normalEstimateWorld, filteredContactVelWorld));

  if (!normalVelocityLeakageInitialized) {
    normalVelocityLeakageLpf = normalVelocityLeakageRaw;
    normalVelocityLeakageInitialized = true;
  } else {
    normalVelocityLeakageLpf += alpha * (normalVelocityLeakageRaw - normalVelocityLeakageLpf);
  }
}

static void updateVelocityModulationNormalDot(void)
{
  if (!normalEstimateValid || !vec3IsFinite(normalEstimateDotWorld)) {
    return;
  }

  if (!velocityModulationNormalDotInitialized) {
    vec3Copy(velocityModulationNormalDotWorld, normalEstimateDotWorld);
    velocityModulationNormalDotInitialized = true;
    return;
  }

  const float cutoffHz = su_velocity_modulation_n_dot_lpf_hz;
  if (isLpfBypassed(cutoffHz)) {
    vec3Copy(velocityModulationNormalDotWorld, normalEstimateDotWorld);
    return;
  }

  const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
  const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
  const float alpha = dt / (tau + dt);
  for (int i = 0; i < 3; ++i) {
    velocityModulationNormalDotWorld[i] +=
      alpha * (normalEstimateDotWorld[i] - velocityModulationNormalDotWorld[i]);
  }
}

static void updateCurvatureEstimate(void)
{
  if (!filteredContactVelInitialized ||
      !velocityModulationNormalDotInitialized || !normalEstimateValid) {
    return;
  }

  const float velNormSq = vec3Dot(filteredContactVelWorld,
                                  filteredContactVelWorld);
  const float vMin = clampPositive(su_velocity_modulation_v_min);
  if (!isfinite(velNormSq) || velNormSq <= 1.0e-12f ||
      sqrtf(velNormSq) <= vMin) {
    return;
  }

  const float curvatureRaw =
    fabsf(vec3Dot(velocityModulationNormalDotWorld,
                  filteredContactVelWorld)) / velNormSq;
  if (!isfinite(curvatureRaw)) {
    return;
  }

  const float cutoffHz = su_velocity_modulation_kappa_lpf_hz;
  if (!curvatureEstimateValid || isLpfBypassed(cutoffHz)) {
    curvatureEstimate = curvatureRaw;
  } else {
    const float dt = 1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ;
    const float tau = 1.0f / (2.0f * (float)M_PI * cutoffHz);
    const float alpha = dt / (tau + dt);
    curvatureEstimate += alpha * (curvatureRaw - curvatureEstimate);
  }
  curvatureEstimateValid = true;
}

static void applyVelocityModulation(float velocityCmdLocal[3])
{
  velocityModulationAlpha = 1.0f;
  if (!velocityCmdLocal || su_velocity_modulation_enable == 0 ||
      !curvatureEstimateValid) {
    return;
  }

  const float tangentialVelocityNormSq =
    velocityCmdLocal[1] * velocityCmdLocal[1] +
    velocityCmdLocal[2] * velocityCmdLocal[2];
  const float aBarN = su_velocity_modulation_a_bar_n;
  const float nominalNormalAcceleration =
    curvatureEstimate * tangentialVelocityNormSq;
  if (!isfinite(tangentialVelocityNormSq) ||
      !isfinite(nominalNormalAcceleration) ||
      nominalNormalAcceleration <= 1.0e-12f ||
      !isfinite(aBarN) || aBarN <= 0.0f) {
    return;
  }

  float alpha = sqrtf(aBarN / nominalNormalAcceleration);
  if (!isfinite(alpha)) {
    return;
  }
  if (alpha > 1.0f) {
    alpha = 1.0f;
  } else if (alpha < 0.0f) {
    alpha = 0.0f;
  }

  velocityModulationAlpha = alpha;
  velocityCmdLocal[1] *= alpha;
  velocityCmdLocal[2] *= alpha;
}

static void buildContactFrame(const float normalWorld[3], float t1World[3], float t2World[3])
{
  const float alphaRef[3] = {0.0f, 0.0f, 1.0f};
  float t1Candidate[3];
  vec3Cross(t1Candidate, alphaRef, normalWorld);

  if (!vec3Normalize(t1World, t1Candidate, 1e-6f)) {
    const float fallbackAxis[3] = {0.0f, 1.0f, 0.0f};
    vec3Cross(t1Candidate, fallbackAxis, normalWorld);
    if (!vec3Normalize(t1World, t1Candidate, 1e-6f)) {
      t1World[0] = 0.0f;
      t1World[1] = -1.0f;
      t1World[2] = 0.0f;
    }
  }

  vec3Cross(t2World, normalWorld, t1World);
  if (!vec3Normalize(t2World, t2World, 1e-6f)) {
    t2World[0] = 0.0f;
    t2World[1] = 0.0f;
    t2World[2] = 1.0f;
  }
}

static void applyTangentialVelocityControl(float velocityCmdWorld[3])
{
  if (!velocityCmdWorld) {
    return;
  }

  if (!isContactFrameControlEnabled()) {
    return;
  }

  float normalWorld[3];
  getControlNormalWorld(normalWorld);

  float t1World[3];
  float t2World[3];
  buildContactFrame(normalWorld, t1World, t2World);

  const float normalCoeff = velocityCmdWorld[0];
  const float tangentialCoeff1 = velocityCmdWorld[1];
  const float tangentialCoeff2 = velocityCmdWorld[2];

  float remappedVelocity[3];
  remappedVelocity[0] = -normalCoeff * normalWorld[0] +
                        tangentialCoeff1 * t1World[0] +
                        tangentialCoeff2 * t2World[0];
  remappedVelocity[1] = -normalCoeff * normalWorld[1] +
                        tangentialCoeff1 * t1World[1] +
                        tangentialCoeff2 * t2World[1];
  remappedVelocity[2] = -normalCoeff * normalWorld[2] +
                        tangentialCoeff1 * t1World[2] +
                        tangentialCoeff2 * t2World[2];

  vec3Copy(velocityCmdWorld, remappedVelocity);
}

static bool isPreloadVelocityControlActive(const uint8_t positionMode,
                                           const float forceDesired)
{
  return positionMode == SU_POSITION_MODE_VELOCITY &&
         fabsf(forceDesired) > 1e-6f;
}

static void applyPreloadVelocityControl(float velocityCmdWorld[3],
                                        const state_t *state,
                                        const float forceDesired)
{
  if (!velocityCmdWorld || !state) {
    return;
  }

  float normalWorld[3];
  getControlNormalWorld(normalWorld);

  float worldForce[3] = {0.0f, 0.0f, 0.0f};
  suThrustEffectivenessGetContactForceWorld(worldForce);

  const float f_n = vec3Dot(normalWorld, worldForce);
  const float stateVelocityWorld[3] = {
    state->velocity.x,
    state->velocity.y,
    state->velocity.z,
  };
  const float r_v = vec3Dot(normalWorld, stateVelocityWorld);
  const float nu_n = clampSymmetric(
    su_g_nf * (forceDesired - f_n) + su_g_nv * r_v,
    su_nu_n_bar);

  velocityCmdWorld[0] -= nu_n * normalWorld[0];
  velocityCmdWorld[1] -= nu_n * normalWorld[1];
  velocityCmdWorld[2] -= nu_n * normalWorld[2];
}

static void updateYawFromMobForce(void)
{
  if (!isNormalEstimatorEnabled()) {
    referenceYawCorrectionDeg = 0.0f;
    return;
  }

  float normalWorld[3];
  getEstimatedNormalWorld(normalWorld);
  const float targetDirXY[2] = {-normalWorld[0], -normalWorld[1]};

  const float targetDirNormXY = sqrtf(targetDirXY[0] * targetDirXY[0] +
                                      targetDirXY[1] * targetDirXY[1]);
  if (targetDirNormXY <= 1.0e-6f) {
    referenceYawCorrectionDeg = 0.0f;
    return;
  }

  const float targetYawDeg = atan2f(targetDirXY[1], targetDirXY[0]) * SU_RAD2DEG;
  const float yawErrorDeg = wrapAngleDeg180(targetYawDeg - referenceBaseYawDeg);
  float yawCorrectionDeg = yawErrorDeg;
  const float yawAlignMaxDeg = SU_YAW_ALIGN_SAT_DEG;

  if (yawAlignMaxDeg > 0.0f) {
    yawCorrectionDeg = clampSymmetric(yawCorrectionDeg, yawAlignMaxDeg);
  }

  referenceYawCorrectionDeg = yawCorrectionDeg;
}

void suPositionReferenceInit(void)
{
  referenceInitialized = false;
  referencePosition.x = 0.0f;
  referencePosition.y = 0.0f;
  referencePosition.z = 0.0f;
  eeReferenceLog.x = 0.0f;
  eeReferenceLog.y = 0.0f;
  eeReferenceLog.z = 0.0f;
  referenceBaseYawDeg = 0.0f;
  referenceYawCorrectionDeg = 0.0f;
  referenceYawDegLog = 0.0f;
  lastPositionMode = SU_POSITION_MODE_POSITION;
  lastCommandReference = SU_COMMAND_REFERENCE_END_EFFECTOR;
  resetContactVelocityFilter();
  resetNormalEstimator();
  resetVelocityModulation();

  suPositionTriggerInit();
}

void suPositionReferenceUpdateSetpoint(setpoint_t *setpoint, const state_t *state, stabilizerStep_t stabilizerStep)
{
  suPositionTriggerUpdate();

  if (!setpoint || !isPositionSetpointCandidate(setpoint)) {
    return;
  }

  if (!referenceInitialized) {
    initializeReference(setpoint, state, suPositionTriggerGetCommandReference());
  }

  const uint8_t positionMode = suPositionTriggerGetMode();
  const uint8_t commandReference = suPositionTriggerGetCommandReference();
  const float forceDesired = suPositionTriggerGetForceDesired();
  const bool advancedVelocityControlEnabled = isAdvancedVelocityControlMode(positionMode);
  const float currentReferenceYawDeg = getReferenceYawDeg();
  if (commandReference != lastCommandReference) {
    convertReferencePosition(&referencePosition, lastCommandReference, commandReference, currentReferenceYawDeg);
  }

  referenceBaseYawDeg = wrapAngleDeg180(setpoint->attitude.yaw);

  if (positionMode != lastPositionMode) {
    referenceYawCorrectionDeg = 0.0f;
    resetContactVelocityFilter();
    resetVelocityModulation();
    if (!advancedVelocityControlEnabled) {
      resetNormalEstimator();
    }
  }

  if (RATE_DO_EXECUTE(SU_POSITION_VELOCITY_RATE_HZ, stabilizerStep)) {
    float velocityCmdWorld[3] = {
      setpoint->position.x,
      setpoint->position.y,
      setpoint->position.z,
    };
    updateContactPointVelocity();
    if (advancedVelocityControlEnabled) {
      updateNormalEstimator();
      updateVelocityModulationNormalDot();
      updateCurvatureEstimate();
      applyVelocityModulation(velocityCmdWorld);
      applyTangentialVelocityControl(velocityCmdWorld);
      if (isPreloadVelocityControlActive(positionMode, forceDesired)) {
        applyPreloadVelocityControl(velocityCmdWorld, state, forceDesired);
      }
    }

    referencePosition.x += velocityCmdWorld[0] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
    referencePosition.y += velocityCmdWorld[1] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);
    referencePosition.z += velocityCmdWorld[2] * (1.0f / (float)SU_POSITION_VELOCITY_RATE_HZ);

    if (advancedVelocityControlEnabled) {
      updateYawFromMobForce();
    } else {
      referenceYawCorrectionDeg = 0.0f;
    }
  }

  writeReferenceToSetpoint(setpoint, commandReference);

  lastPositionMode = positionMode;
  lastCommandReference = commandReference;
}

LOG_GROUP_START(suPosRef)
LOG_ADD(LOG_FLOAT, nPreX, &normalForceEvidenceWorld[0])
LOG_ADD(LOG_FLOAT, nPreY, &normalForceEvidenceWorld[1])
LOG_ADD(LOG_FLOAT, nPreZ, &normalForceEvidenceWorld[2])
LOG_ADD(LOG_FLOAT, nPostX, &normalProjectedCandidateWorld[0])
LOG_ADD(LOG_FLOAT, nPostY, &normalProjectedCandidateWorld[1])
LOG_ADD(LOG_FLOAT, nPostZ, &normalProjectedCandidateWorld[2])
LOG_ADD(LOG_FLOAT, nEstX, &normalEstimateWorld[0])
LOG_ADD(LOG_FLOAT, nEstY, &normalEstimateWorld[1])
LOG_ADD(LOG_FLOAT, nEstZ, &normalEstimateWorld[2])
LOG_ADD(LOG_FLOAT, nDotX, &normalEstimateDotWorld[0])
LOG_ADD(LOG_FLOAT, nDotY, &normalEstimateDotWorld[1])
LOG_ADD(LOG_FLOAT, nDotZ, &normalEstimateDotWorld[2])
LOG_ADD(LOG_FLOAT, vEeX, &filteredContactVelWorld[0])
LOG_ADD(LOG_FLOAT, vEeY, &filteredContactVelWorld[1])
LOG_ADD(LOG_FLOAT, vEeZ, &filteredContactVelWorld[2])
LOG_ADD(LOG_FLOAT, nVelLeak, &normalVelocityLeakageLpf)
LOG_ADD(LOG_FLOAT, eeCmdX, &eeReferenceLog.x)
LOG_ADD(LOG_FLOAT, eeCmdY, &eeReferenceLog.y)
LOG_ADD(LOG_FLOAT, eeCmdZ, &eeReferenceLog.z)
LOG_ADD(LOG_FLOAT, eeCmdYaw, &referenceYawDegLog)
LOG_GROUP_STOP(suPosRef)

LOG_GROUP_START(suVelMod)
LOG_ADD(LOG_FLOAT, nHatDotX, &velocityModulationNormalDotWorld[0])
LOG_ADD(LOG_FLOAT, nHatDotY, &velocityModulationNormalDotWorld[1])
LOG_ADD(LOG_FLOAT, nHatDotZ, &velocityModulationNormalDotWorld[2])
LOG_ADD(LOG_FLOAT, kappaHat, &curvatureEstimate)
LOG_ADD(LOG_FLOAT, alphaStar, &velocityModulationAlpha)
LOG_ADD(LOG_FLOAT, vcX, &filteredContactVelWorld[0])
LOG_ADD(LOG_FLOAT, vcY, &filteredContactVelWorld[1])
LOG_ADD(LOG_FLOAT, vcZ, &filteredContactVelWorld[2])
LOG_GROUP_STOP(suVelMod)

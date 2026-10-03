#include "platform_defaults.h"
#include "param.h"
#include "su_params.h"

#include <math.h>

// ========= 전역 공유 파라미터 정의 (단일 소스) =========
// 플랫폼/모델
float su_mass            = CF_MASS;      // [kg] 원래는 0.0393


// Wrench observer / MOB 관련
float su_mob_lpf_hz      = 1.5f;         // [Hz], wn=2*pi*f; shared critical damping
float su_com_offset_x    = 0.0f;         // [m] body-frame CoM offset x
float su_com_offset_y    = 0.0f;         // [m] body-frame CoM offset y
float su_com_offset_z    = 0.0f;         // [m] body-frame CoM offset z
float su_r_offset_x      = 0.085f;       // [m] body-frame point-contact offset x
float su_r_offset_y      = 0.0f;         // [m] body-frame point-contact offset y
float su_r_offset_z      = 0.020f;       // [m] body-frame point-contact offset z
uint8_t su_normal_estimation = 1;        // 0: fixed normal, 1: enable force-dominant normal estimator
float su_normal_beta      = 3.0f;        // [1/s] normal-axis memory decay
float su_normal_gamma     = 6.0f;        // [1/s] normal-axis tracking gain
float su_normal_epsilon_v = 0.003f;      // [m^2/s^2] velocity projection regularization
float su_normal_epsilon_f = 0.01f;       // [N] minimum force evidence norm
uint8_t su_velocity_modulation_enable = 1;
float su_velocity_modulation_a_bar_n = 0.013f; // [m/s^2] normal acceleration bound
float su_velocity_modulation_v_min = 0.01f;    // [m/s] minimum contact speed for curvature update
float su_contact_velocity_lpf_hz = 2.0f;       // [Hz] shared contact-velocity LPF cutoff; <= 0 bypasses LPF
float su_velocity_accel_xy_max = 5.0f;         // [m/s^2] numerical velocity XY acceleration gate
float su_velocity_accel_z_max = 8.0f;          // [m/s^2] numerical velocity Z acceleration gate
float su_velocity_modulation_n_dot_lpf_hz = 3.0f; // [Hz] normal-derivative LPF cutoff; <= 0 bypasses LPF
float su_velocity_modulation_kappa_lpf_hz = 1.0f; // [Hz] curvature LPF cutoff; <= 0 bypasses LPF
float su_g_nf             = 1.0f;        // normal force tracking gain
float su_g_nv             = 0.0f;        // normal velocity damping gain (disabled by default)
float su_nu_n_bar         = 0.08f;       // [m/s] symmetric saturation of normal velocity command

// ========= PARAM 등록 =========
// PARAM_GROUP_START(su_platform)
// // Platform / model parameters
// PARAM_ADD(PARAM_FLOAT, mass, &su_mass)
// PARAM_GROUP_STOP(su_platform)

// // Wrench/MOB 파라미터: 기존 su_wrench 그룹명 유지(로그/툴 호환성)
PARAM_GROUP_START(su_wrench)
PARAM_ADD(PARAM_FLOAT, mass,            &su_mass)
PARAM_ADD(PARAM_FLOAT, mobLPF,          &su_mob_lpf_hz)
PARAM_ADD(PARAM_FLOAT, comOffX,         &su_com_offset_x)
PARAM_ADD(PARAM_FLOAT, comOffY,         &su_com_offset_y)
PARAM_ADD(PARAM_FLOAT, comOffZ,         &su_com_offset_z)
PARAM_ADD(PARAM_FLOAT, rOffX,           &su_r_offset_x)
PARAM_ADD(PARAM_FLOAT, rOffY,           &su_r_offset_y)
PARAM_ADD(PARAM_FLOAT, rOffZ,           &su_r_offset_z)
PARAM_GROUP_STOP(su_wrench)

void suGetMobLpfGains(float *stiffness_gain, float *damping_gain)
{
  const float cutoff_hz = (isfinite(su_mob_lpf_hz) && su_mob_lpf_hz > 0.0f) ?
    su_mob_lpf_hz : 0.0f;
  const float omega_n = 2.0f * (float)M_PI * cutoff_hz;
  if (stiffness_gain) {
    *stiffness_gain = omega_n * omega_n;
  }
  if (damping_gain) {
    *damping_gain = 2.0f * omega_n;
  }
}

PARAM_GROUP_START(su_position)
PARAM_ADD(PARAM_UINT8, preloadEn,       &su_normal_estimation)
PARAM_ADD(PARAM_FLOAT, normBeta,        &su_normal_beta)
PARAM_ADD(PARAM_FLOAT, normGamma,       &su_normal_gamma)
PARAM_ADD(PARAM_FLOAT, normEpsV,        &su_normal_epsilon_v)
PARAM_ADD(PARAM_FLOAT, normEpsF,        &su_normal_epsilon_f)
PARAM_ADD(PARAM_UINT8, velModEn,        &su_velocity_modulation_enable)
PARAM_ADD(PARAM_FLOAT, velModABar,      &su_velocity_modulation_a_bar_n)
PARAM_ADD(PARAM_FLOAT, velModVMin,      &su_velocity_modulation_v_min)
PARAM_ADD(PARAM_FLOAT, contactVelLPF,    &su_contact_velocity_lpf_hz)
PARAM_ADD(PARAM_FLOAT, velAccXY,         &su_velocity_accel_xy_max)
PARAM_ADD(PARAM_FLOAT, velAccZ,          &su_velocity_accel_z_max)
PARAM_ADD(PARAM_FLOAT, velModNDotLPF,   &su_velocity_modulation_n_dot_lpf_hz)
PARAM_ADD(PARAM_FLOAT, velModKLPF,      &su_velocity_modulation_kappa_lpf_hz)
PARAM_ADD(PARAM_FLOAT, preloadGf,       &su_g_nf)
PARAM_ADD(PARAM_FLOAT, preloadGv,       &su_g_nv)
PARAM_ADD(PARAM_FLOAT, preloadNu,       &su_nu_n_bar)
PARAM_GROUP_STOP(su_position)

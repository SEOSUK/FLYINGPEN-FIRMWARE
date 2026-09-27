#ifndef SU_PARAMS_H_
#define SU_PARAMS_H_
/*
 * Centralized shared parameters for SU modules
 *
 * These are defined (storage allocated) in su_params.c and registered
 * to the Crazyflie PARAM system there. Include this header from any
 * module (estimator / controller / trajectory / observers ...) that
 * needs to read/write the same parameters at runtime.
 *
 * Units:
 *  - mass:            [kg]
 *  - Ktau:            [1/s] in MOB, [1/s^2] in matched-filter dynamics
 *  - Kh:              [1/s]
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// -------- Platform / common --------
extern float su_mass;             // [kg]

// -------- Wrench observer / MOB --------
extern float su_Ktau;             // Shared MOB estimate / matched-filter stiffness gain
extern float su_Kh;               // [1/s] shared MOB / matched-filter damping gain

extern float su_com_offset_x;     // [m] body-frame CoM offset x
extern float su_com_offset_y;     // [m] body-frame CoM offset y
extern float su_com_offset_z;     // [m] body-frame CoM offset z
extern float su_r_offset_x;       // [m] body-frame contact offset x
extern float su_r_offset_y;       // [m] body-frame contact offset y
extern float su_r_offset_z;       // [m] body-frame contact offset z

// -------- Position reference --------
extern uint8_t su_normal_estimation; // 0: fixed normal, 1: force-dominant normal estimator enabled
extern float su_normal_beta;      // [1/s] normal-axis memory decay
extern float su_normal_gamma;     // [1/s] normal-axis tracking gain
extern float su_normal_epsilon_v; // [m^2/s^2] velocity projection regularization
extern float su_normal_epsilon_f; // [N] minimum force evidence norm
extern uint8_t su_velocity_modulation_enable; // 0: disabled, 1: enabled
extern float su_velocity_modulation_a_bar_n;  // [m/s^2] normal acceleration bound
extern float su_velocity_modulation_v_min;    // [m/s] minimum contact speed for curvature update
extern float su_contact_velocity_lpf_hz;      // [Hz] shared contact-velocity LPF cutoff
extern float su_velocity_modulation_n_dot_lpf_hz; // [Hz] normal-derivative LPF cutoff for curvature
extern float su_velocity_modulation_kappa_lpf_hz; // [Hz] curvature LPF cutoff
extern float su_g_nf;             // [m/(s*N)] gain from normal force tracking error
extern float su_g_nv;             // [-] gain from normal velocity leakage
extern float su_nu_n_bar;         // [m/s] symmetric saturation limit for normal velocity command
extern float su_epsilon_f_min;    // [N] lower threshold where force-aligned yaw smoothing starts
extern float su_epsilon_f_max;    // [N] upper threshold where force-aligned yaw reaches full weight

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SU_PARAMS_H_ */

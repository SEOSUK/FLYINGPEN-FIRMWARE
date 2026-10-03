// su_vel_from_pos.h
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void suVelFromPosInit(void);

// 100 Hz에서 호출한다. 새 외부 position sample만 회귀창에 넣고 LPF는 매 호출마다 갱신한다.
void suVelFromPosUpdate(const float position_world[3], uint32_t position_sample_count, float update_dt);

// World-frame velocity [m/s]
void suVelFromPosGetWorld(float out_vW[3]);

#ifdef __cplusplus
}
#endif

#pragma once
#ifndef __cplusplus
#include <stdbool.h>
#endif
    

#ifdef __cplusplus
extern "C" {
#endif
    
bool pwmSetPositionOrSpeedHW(int servoOrMotor, uint32_t speedOrPosition);
#ifdef __cplusplus
}
#endif

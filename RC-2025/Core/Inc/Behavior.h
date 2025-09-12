#ifndef _BEHAVIOR_H__
#define _BEHAVIOR_H__

// #include "CAN_Receive.h"
#include "Position_task.h"
#include "Reset.h"
#include "Mapping.h"

#define lu 0 << 1 | 1 << 0 // 1
#define ru 1 << 1 | 1 << 0 // 3
#define ld 0 << 1 | 0 << 0 // 0
#define rd 1 << 1 | 0 << 0 // 2

extern double h;

void Behavior_Init(void);
void Behavior_Update(void);
void Behavior_Task(void);
void Behavior_reset(void); // 结合Reset写
int leg_inverse_kinematics(double x, double z, double *theta1, double *theta2);
float calc_leg_encoder_value(double z, double forward);

#endif

#ifndef _RESET_H__
#define _RESET_H__

#include "behavior.h"

// 左零右一；上零下一。
// 设置编号
#define lul 0 << 2 | 0 << 1 | 0 << 0 // 0   1601
#define ldl 0 << 2 | 1 << 1 | 0 << 0 // 2   5917
#define lur 0 << 2 | 0 << 1 | 1 << 0 // 1   7550
#define ldr 0 << 2 | 1 << 1 | 1 << 0 // 3   3720
#define rul 1 << 2 | 0 << 1 | 0 << 0 // 4   3812
#define rdl 1 << 2 | 1 << 1 | 0 << 0 // 6   6219
#define rur 1 << 2 | 0 << 1 | 1 << 0 // 5   7498
#define rdr 1 << 2 | 1 << 1 | 1 << 0 // 7   6812

extern int Read_ID[8];
extern int Read_angle[8]; // 读电机初始角度
extern int Read_state[8];
extern int Read_OK;
extern double Read_Cpa[8]; // 初始复位目标值

int Read_update(void);
// int State_run(void);
void Reset_task(void);

#endif

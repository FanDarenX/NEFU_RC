#ifndef YAW_TASK
#define YAW_TASK

#include "pid.h"
#include "remote_control.h"

#include "position_task.h"

typedef struct
{
	float xx1;
	float xx2;
	float fh1;
	float r;
	float h1;
	float h2;
} TD;

extern TD Dog_td;
extern TD Dog_lul_td;
extern TD Dog_rul_td;
extern TD Dog_ldl_td;
extern TD Dog_rdl_td;

extern TD Dog_lur_td;
extern TD Dog_rur_td;
extern TD Dog_ldr_td;
extern TD Dog_rdr_td;

extern TD Cup_pitch_td;
extern TD Cup_roll_td;
extern TD yaw_td;
extern TD pic_td;
extern TD Uplift_td;
extern TD Y_td;
extern TD X_td;

void TD_Fast_Change(float Dog_R);

void TD_init(TD *TT, float r, float h1, float h2);
void TD_INIT(void);
void TD_task(void);
void Tracking_Differentiator(TD *TT, float target, float r, float h1, float h2);
#endif

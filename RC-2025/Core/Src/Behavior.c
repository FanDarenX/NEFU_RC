#include "Behavior.h"
#include "remote_control.h"
#include "math.h"
/*机械参数*/
#define L1 125.0
#define L2 254.0
/*坐标系*/
double h = 187.08f; // 145.08,��155�ȣ��Ƕ�תxyOK
double ox = 0.0f;
double oy = 0.0f;
/*时间*/
uint64_t foot_time = 0;
/*相位*/
// 整体周期
double Whole_phase = 2 * PI; // 基调
// 单腿周期+补偿
double phase_current[4] = {0.0f, PI, PI, 0.0f}; // TODO细微相位差

/*存放当前足末端实际(x,y)*/
double ANG_X[4] = {0.0f};
double ANG_Y[4] = {0.0f};

/*存放目标足末端目标(X,Y)*/
double TAR_X[4] = {0.0f};
double TAR_Y[4] = {0.0f};

/*设定坐标系下足末端的最高位置*/
double FootH = 30.0f * 1.5f; // 抬腿y为正   30.0f * 16.0f  //TODO 改成1.0f时步距不足
/*设定坐标系下足末端的最远位置*/
double FootX = 30.0f * 4.5f; // 前进x为负   -30.0f * 16.0f //TODO 第一次竞速4.0f    ,4.1
/*切换姿态所需坐标系变量*/
double Base_Y = 0.0f;
double Base_X = -20.0f;

/*跳跃STEP*/
int Jump_step = 0; // 初始第0步

/*设定坐标系下足末端的最高位置*/
double FootH_P = 40.0f; // 抬腿y为正   30.0f * 16.0f
/*设定坐标系下足末端的最远位置*/
double FootX_P = 130.0f; // 前进x为负   -30.0f * 16.0f
/*切换姿态所需坐标系变量*/
double Base_Y_P = 0.0f;

/*设定坐标系下足末端的最高位置*/
double FootH_S = 40.0f; // 抬腿y为正   30.0f * 16.0f
/*设定坐标系下足末端的最远位置*/
double FootX_S = 130.0f; // 前进x为负   -30.0f * 16.0f
/*切换姿态所需坐标系变量*/
double Base_Y_S = 0.0f;

/*初始复位最大角度*/
double init_angle = 145.0f;

/*标志位*/
int reset_manual = 0;

// 示例：定义腿部抬升高度和前后移动幅度
double AMP_Z = 30.0f; //! 抬腿高度（单位：毫米）
#define BASE_Z 55.0   //! 默认站立高度
double dic_ang = 0.0f;
double Stockpile = 0.0f; // ��ʼΪĬ��ֱ��������������С
double Release = 350.0f; // ѡ��Ҫ�ͷŵ����λ�ã�Խ���ٶ�Խ�죬������ϻ��գ���ֹ�в��ϰ���
int Jump_flag = 0;
double current_phase = 0.0;
double delta_phase = 0.0193 * 3.0f; //! 控制速度//0.01
uint8_t initialized = 0;

/*实现遥控器左纵控制前进后退以及速度快慢*/
// 定义最小和最大步态频率系数
#define MIN_FREQ_COEF 0.002f
#define MAX_FORWARD_FREQ_COEF 0.01f  // 前进最大频率
#define MAX_BACKWARD_FREQ_COEF 0.08f // 后退最大频率（通常比前进慢）

// 获取左纵摇杆值
#define ch3 rc_ctrl.rc.ch[3]
#define ch1 rc_ctrl.rc.ch[1]
#define ch2 rc_ctrl.rc.ch[2]
#define ch4 rc_ctrl.rc.ch[4]
#define ch0 rc_ctrl.rc.ch[0]

/***************************************分割线😍**************************************/

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void limit_behavior(double *value, double max, double min)
{
    if (*value > max)
        *value = max;
    else if (*value < min)
        *value = min;
}
void Trot_XY(void)
{
    if (Lift_Down) // Trot
    {
        Base_Y = 20.0f; // 400
        for (int i = 0; i < 4; i++)
        {
            // 周期循环
            if (phase_current[i] >= 2 * PI)
            {
                phase_current[i] -= 2 * PI;
            }
            else if (phase_current[i] < 0)
            {
                phase_current[i] += 2 * PI;
            }

            /*START*/
            if (phase_current[i] < PI / 2.0) //
            {
                TAR_X[i] = phase_current[i] * FootX / (PI / 2.0) + Base_X; //
                TAR_Y[i] = cos(phase_current[i]) * FootH + Base_Y;
            }
            else if (phase_current[i] > PI / 2.0 && phase_current[i] < PI)
            {
                TAR_X[i] = (1 - (phase_current[i] - (PI / 2.0)) / (PI / 2.0)) * FootX + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
            else if (phase_current[i] > PI && phase_current[i] < 3.0 * PI / 2.0)
            {
                TAR_X[i] = (phase_current[i] - PI) * (-FootX) / (PI / 2.0) + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
            else
            {
                TAR_X[i] = (1.0f - (phase_current[i] - (3.0f * PI / 2.0f)) / (PI / 2.0f)) * (-FootX) + Base_X;
                TAR_Y[i] = cos(phase_current[i]) * FootH + Base_Y;
            }
            phase_current[i] += delta_phase;
        }

        /*转向*/
        if (ch0 < -100) // 右转
        {
            TAR_X[ru] *= 0.7; // -0.5   //TODO!此处改成负号为了障碍赛斜坡转向
            TAR_X[rd] *= 0.7; // 0.6
        }
        else if (ch0 > 100) // 左转
        {
            TAR_X[lu] *= 0.7; // 0.6
            TAR_X[ld] *= 0.7; // 0.6
        }

        // delta_phase = 0.0193 * 3;

        /*统一周期speed*/
        if (ch3 > 400)
        {
            FootX = 30.0f * 4.5f;
            Base_X = -20;
            delta_phase = 0.0193 * 6.0f; // ch3 / 660.0f * 0.0193 * 3.0 + (0.0193 * 2.0f)
        }
        else if (ch3 > 20 && ch3 < 400)
        {
            FootX = 30.0f * 3.5f;
            Base_X = -20;
            delta_phase = 0.0193 * 3.0f; // ch3 / 660.0f * 0.0193 * 3.0 + (0.0193 * 2.0f)
        }
        else if (ch3 <= 20 && ch3 >= -20)
        {
            Base_X = -20;
            delta_phase = 0.0f;
            limit_behavior(&Base_Y, 50, 0);
            for (int i = 0; i < 4; i++)
            {
                TAR_X[i] = 0.0f + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
        }
        else if (ch3 < -20)
        {
            Base_X = 10;
            delta_phase = -0.0193 * 3.0f;
        }
    }
    else if (Lift_Up) // 速度
    {
        Base_Y = 20.0f; // 400
        for (int i = 0; i < 4; i++)
        {
            // 周期循环
            if (phase_current[i] >= 2 * PI)
            {
                phase_current[i] -= 2 * PI;
            }
            else if (phase_current[i] < 0)
            {
                phase_current[i] += 2 * PI;
            }

            /*START*/
            if (phase_current[i] < PI / 2.0) //
            {
                TAR_X[i] = phase_current[i] * FootX / (PI / 2.0) + Base_X; //
                TAR_Y[i] = cos(phase_current[i]) * FootH + Base_Y;
            }
            else if (phase_current[i] > PI / 2.0 && phase_current[i] < PI)
            {
                TAR_X[i] = (1 - (phase_current[i] - (PI / 2.0)) / (PI / 2.0)) * FootX + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
            else if (phase_current[i] > PI && phase_current[i] < 3.0 * PI / 2.0)
            {
                TAR_X[i] = (phase_current[i] - PI) * (-FootX) / (PI / 2.0) + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
            else
            {
                TAR_X[i] = (1.0f - (phase_current[i] - (3.0f * PI / 2.0f)) / (PI / 2.0f)) * (-FootX) + Base_X;
                TAR_Y[i] = cos(phase_current[i]) * FootH + Base_Y;
            }
            phase_current[i] += delta_phase;
        }

        /*转向*/
        if (ch0 < -100) // 右转
        {
            TAR_X[ru] *= 0.7; // -0.5   //TODO!此处改成负号为了障碍赛斜坡转向
            TAR_X[rd] *= 0.7; // 0.6
        }
        else if (ch0 > 100) // 左转
        {
            TAR_X[lu] *= 0.7; // 0.6
            TAR_X[ld] *= 0.7; // 0.6
        }

        // delta_phase = 0.0193 * 3;

        /*统一周期speed*/
        if (ch3 > 400)
        {
            FootX = 30.0f * 4.0f;
            FootH = 30.0f * 1.5f;
            Base_X = -20;
            delta_phase = 0.0193 * 4.0f; // ch3 / 660.0f * 0.0193 * 3.0 + (0.0193 * 2.0f)
        }
        else if (ch3 > 20 && ch3 < 400)
        {
            FootX = 30.0f * 3.5f;
            FootH = 30.0f * 2.0f;
            Base_X = -20;
            delta_phase = 0.0193 * 3.0f; // ch3 / 660.0f * 0.0193 * 3.0 + (0.0193 * 2.0f)
        }
        else if (ch3 <= 20 && ch3 >= -20)
        {
            Base_X = -20;
            delta_phase = 0.0f;
            limit_behavior(&Base_Y, 50, 0);
            for (int i = 0; i < 4; i++)
            {
                TAR_X[i] = 0.0f + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
        }
        else if (ch3 < -20)
        {
            Base_X = 10;
            delta_phase = -0.0193 * 3.0f;
        }
    }
    else if (Lift_Middle) // 攀爬
    {
        Base_Y = 20.0f; // 400
        for (int i = 0; i < 4; i++)
        {
            // 周期循环
            if (phase_current[i] >= 2 * PI)
            {
                phase_current[i] -= 2 * PI;
            }
            else if (phase_current[i] < 0)
            {
                phase_current[i] += 2 * PI;
            }

            /*START*/
            if (phase_current[i] < PI / 2.0) //
            {
                TAR_X[i] = phase_current[i] * FootX / (PI / 2.0) - 20.0f; //
                TAR_Y[i] = cos(phase_current[i]) * FootH + Base_Y;
            }
            else if (phase_current[i] > PI / 2.0 && phase_current[i] < PI)
            {
                TAR_X[i] = (1 - (phase_current[i] - (PI / 2.0)) / (PI / 2.0)) * FootX - 20.0f;
                TAR_Y[i] = 0.0f + Base_Y;
            }
            else if (phase_current[i] > PI && phase_current[i] < 3.0 * PI / 2.0)
            {
                TAR_X[i] = (phase_current[i] - PI) * (-FootX) / (PI / 2.0) - 20.0f;
                TAR_Y[i] = 0.0f + Base_Y;
            }
            else
            {
                TAR_X[i] = (1.0f - (phase_current[i] - (3.0f * PI / 2.0f)) / (PI / 2.0f)) * (-FootX) - 20.0f;
                TAR_Y[i] = cos(phase_current[i]) * FootH + Base_Y;
            }
            phase_current[i] += delta_phase;
        }

        /*转向*/
        if (ch0 < -100) // 右转
        {
            TAR_X[ru] *= -0.8; // -0.5   //TODO!此处改成负号为了障碍赛斜坡转向
            TAR_X[rd] *= -0.8; // 0.6
        }
        else if (ch0 > 100) // 左转
        {
            TAR_X[lu] *= -0.8; // 0.6
            TAR_X[ld] *= -0.8; // 0.6
        }

        // delta_phase = 0.0193 * 3;
        /*统一周期speed*/
        /*统一周期speed*/
        if (ch3 > 400)
        {
            FootX = 30.0f * 4.5f;
            Base_X = -20;
            delta_phase = 0.0193 * 6.0f; // ch3 / 660.0f * 0.0193 * 3.0 + (0.0193 * 2.0f)
        }
        else if (ch3 > 20 && ch3 < 400)
        {
            FootX = 30.0f * 3.5f;
            Base_X = -20;
            delta_phase = 0.0193 * 3.0f; // ch3 / 660.0f * 0.0193 * 3.0 + (0.0193 * 2.0f)
        }
        else if (ch3 <= 20 && ch3 >= -20)
        {
            Base_X = -20;
            delta_phase = 0.0f;
            limit_behavior(&Base_Y, 50, 0);
            for (int i = 0; i < 4; i++)
            {
                TAR_X[i] = 0.0f + Base_X;
                TAR_Y[i] = 0.0f + Base_Y;
            }
        }
        else if (ch3 < -20)
        {
            Base_X = 10;
            delta_phase = -0.0193 * 3.0f;
        }
    }
}

void Jump_XY(void) // 坐标系下的跳跃
{

    // 1.旋转角度范围等于大腿旋转角度，注意镜像对称分半；
    // 2.蓄力半径长度，与释放最大半径长度固定；
    // 3.拨轮控制控制半圆的前后加减；
    // 4.位置坐标系用半径和旋转角度确定
}

void Jump_Z(void) // 依靠的是硬调参
{
    if (Lift_Down)
    {
        /*LU*/
        Dog_target_lul = -(BASE_Z + Stockpile + dic_ang);
        Dog_target_lur = -(BASE_Z + Stockpile - dic_ang);

        /*LD*/
        Dog_target_ldl = -(BASE_Z + Stockpile + dic_ang);
        Dog_target_ldr = -(BASE_Z + Stockpile - dic_ang);

        /*RU*/
        Dog_target_rul = (BASE_Z + Stockpile - dic_ang);
        Dog_target_rur = (BASE_Z + Stockpile + dic_ang);

        /*RD*/
        Dog_target_rdl = (BASE_Z + Stockpile - dic_ang);
        Dog_target_rdr = (BASE_Z + Stockpile + dic_ang);
    }
    else if (Lift_Middle)
    {
        /*LU*/
        Dog_target_lul = -(BASE_Z + dic_ang);
        Dog_target_lur = -(BASE_Z - dic_ang);

        /*LD*/
        Dog_target_ldl = -(BASE_Z + Stockpile + dic_ang);
        Dog_target_ldr = -(BASE_Z + Stockpile - dic_ang);

        /*RU*/
        Dog_target_rul = (BASE_Z - dic_ang);
        Dog_target_rur = (BASE_Z + dic_ang);

        /*RD*/
        Dog_target_rdl = (BASE_Z + Stockpile - dic_ang);
        Dog_target_rdr = (BASE_Z + Stockpile + dic_ang);
    }
    else if (Lift_Up)
    {
        /*LU*/
        Dog_target_lul = -(BASE_Z + Stockpile + dic_ang);
        Dog_target_lur = -(BASE_Z + Stockpile - dic_ang);

        /*LD*/
        Dog_target_ldl = -(BASE_Z + dic_ang);
        Dog_target_ldr = -(BASE_Z - dic_ang);

        /*RU*/
        Dog_target_rul = (BASE_Z + Stockpile - dic_ang);
        Dog_target_rur = (BASE_Z + Stockpile + dic_ang);

        /*RD*/
        Dog_target_rdl = (BASE_Z - dic_ang);
        Dog_target_rdr = (BASE_Z + dic_ang);
    }

    if (ch4 > 20) // �����̶�
    {
        Stockpile -= 0.05;
    }
    else if (ch4 < -20)
    {
        Stockpile += 0.05;
    }

    if (ch1 > 20) // ��Ծ�Ƕ�
    {
        dic_ang -= 0.05;
    }
    else if (ch1 < -20)
    {
        dic_ang += 0.05;
    }

    if (ch3 >= 600 && !Jump_flag) // ����
    {
        if (!Jump_step)
        {
            Stockpile = Release;
        }
        // if ((init_angle - (Dog_moto_1[0].total_angle_true + dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_1[1].total_angle_true - dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_1[2].total_angle_true + dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_1[3].total_angle_true - dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_2[0].total_angle_true + dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_2[1].total_angle_true - dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_2[2].total_angle_true + dic_ang) >= BASE_Z) && (init_angle - (Dog_moto_2[3].total_angle_true - dic_ang) >= BASE_Z))
        if (Lift_Down || Lift_Middle)
        {
            if (dic_ang <= 0)
            {
                if ((init_angle - (Dog_moto_1[2].total_angle_true) >= BASE_Z - dic_ang) && !Jump_step) //&& (init_angle - (Dog_moto_1[3].total_angle_true - dic_ang) >= BASE_Z)
                {
                    Stockpile = -30.0f;
                    Jump_step = 1;
                }
                if ((init_angle - (Dog_moto_1[2].total_angle_true) <= BASE_Z - dic_ang - 20.0f) && Jump_step == 1)
                {
                    Stockpile = -20.0f;
                    dic_ang = 0.0f; // 改成真值好像会和条件冲突，还是0吧
                    Jump_step = 0;
                    Jump_flag = 1;
                }
            }
            else
            {
                if ((init_angle - (Dog_moto_1[2].total_angle_true) >= BASE_Z + dic_ang) && !Jump_step) //&& (init_angle - (Dog_moto_1[3].total_angle_true - dic_ang) >= BASE_Z)
                {
                    Stockpile = -30.0f;
                    Jump_step = 1;
                }
                if ((init_angle - (Dog_moto_1[2].total_angle_true) <= BASE_Z + dic_ang - 20.0f) && Jump_step == 1)
                {
                    Stockpile = -20.0f;
                    dic_ang = 0.0f;
                    Jump_step = 0;
                    Jump_flag = 1;
                }
            }
        }
        if (Lift_Up)
        {
            if ((init_angle - (Dog_moto_1[0].total_angle_true) >= BASE_Z + dic_ang)) //&& (init_angle - (Dog_moto_1[3].total_angle_true - dic_ang) >= BASE_Z)
            {
                Stockpile = -20.0f;
                dic_ang = -30.0f;
                Jump_flag = 1;
            }
        }
    }
    // 1.BASE_Z 基础高度即蓄力半径；
    // 2.JUMP_Z 跳跃高度即最大释放半径，同加同减；
    // 3.JUMP_X 跳跃起点前后，一加一减；
    // dic_ang为正，BASE_Z加dic_ang；dic_ang为负
}

void Behavior_Task(void)
{
    Behavior_reset();

    Dog_location(Dog_lul_td.xx1, Dog_rul_td.xx1, Dog_ldl_td.xx1, Dog_rdl_td.xx1, Dog_lur_td.xx1, Dog_rur_td.xx1, Dog_ldr_td.xx1, Dog_rdr_td.xx1);
    // 严格顺序把控，这样目标值计算完成后才传进电流发送函数

    if (!Read_OK)
        return;

    /*LU*/
    Dog_moto_1[0].total_angle_true = init_angle - fabs(Dog_moto_1[0].total_angle / 8192.0f / 19.0f * 360.0f);
    Dog_moto_1[1].total_angle_true = init_angle - fabs(Dog_moto_1[1].total_angle / 8192.0f / 19.0f * 360.0f);
    /*LD*/
    Dog_moto_1[2].total_angle_true = init_angle - fabs(Dog_moto_1[2].total_angle / 8192.0f / 19.0f * 360.0f);
    Dog_moto_1[3].total_angle_true = init_angle - fabs(Dog_moto_1[3].total_angle / 8192.0f / 19.0f * 360.0f);
    /*RU*/
    Dog_moto_2[0].total_angle_true = init_angle - fabs(Dog_moto_2[0].total_angle / 8192.0f / 19.0f * 360.0f);
    Dog_moto_2[1].total_angle_true = init_angle - fabs(Dog_moto_2[1].total_angle / 8192.0f / 19.0f * 360.0f);
    /*RD*/
    Dog_moto_2[2].total_angle_true = init_angle - fabs(Dog_moto_2[2].total_angle / 8192.0f / 19.0f * 360.0f);
    Dog_moto_2[3].total_angle_true = init_angle - fabs(Dog_moto_2[3].total_angle / 8192.0f / 19.0f * 360.0f);

    // 坐标系下足末端当前实际(x,y)，由当前实际坐标系旋转角度求得
    // ANG_X[1] = ang_x(Dog_moto_1[2].total_angle_true, Dog_moto_1[3].total_angle_true);
    // ANG_Y[1] = ang_y(Dog_moto_1[2].total_angle_true, Dog_moto_1[3].total_angle_true);

    // 坐标系下左右电机的旋转角度,由目标(x,y)求得
    /*LU*/
    Dog_target_lul = -(init_angle - Ang1(TAR_X[lu], TAR_Y[lu]));
    Dog_target_lur = -(init_angle - Ang2(TAR_X[lu], TAR_Y[lu]));
    /*LD*/
    Dog_target_ldl = -(init_angle - Ang1(TAR_X[ld], TAR_Y[ld]));
    Dog_target_ldr = -(init_angle - Ang2(TAR_X[ld], TAR_Y[ld])); // 可以被Jump的目标值覆盖，除了Jump目标值其余时候就在这被赋值的
    /*RU*/
    Dog_target_rul = (init_angle - Ang1(-TAR_X[ru], TAR_Y[ru]));
    Dog_target_rur = (init_angle - Ang2(-TAR_X[ru], TAR_Y[ru]));
    /*RD*/
    Dog_target_rdl = (init_angle - Ang1(-TAR_X[rd], TAR_Y[rd]));
    Dog_target_rdr = (init_angle - Ang2(-TAR_X[rd], TAR_Y[rd]));
    // 之后想添加模式直接在下面加目标坐标值就OK
    if (Right_Up && (Lift_Down || Lift_Middle || Lift_Up))
    {
        Trot_XY(); // OK
    }
    else if (Right_Down && (Lift_Down || Lift_Middle || Lift_Up))
    {
        Jump_Z(); // OK
    }
    // 比如单独拉出来加一个背越式，或者只在Trot_XY()里面加
    else
    {
        Jump_flag = 0; // 一次跳一下，防止过冲连续卡腿
        for (int i = 0; i < 4; i++)
        {
            TAR_X[i] = 0.0f;
            TAR_Y[i] = 0.0f;
        }
    }
}
void Behavior_reset(void) // 结合Reset写
{
    if (Lift_Up)
    {
        reset_manual = 1;
    }
    if (reset_manual && !Read_OK)
    {
        /*LU*/
        if (!Read_ID[lul] && !Read_state[lul])
        {
            Read_Cpa[lul] += 0.01;
        }
        else if (Read_ID[lul] && !Read_state[lul])
        {
            Read_Cpa[lul] = 0.0;
            Dog_moto_1[0].round_cnt = 0;
            Read_state[lul] = 1;
            Read_angle[lul] = Dog_moto_1[0].angle;
        }
        if (!Read_ID[lur] && !Read_state[lur])
        {
            Read_Cpa[lur] -= 0.01;
        }
        else if (Read_ID[lur] && !Read_state[lur])
        {
            Read_Cpa[lur] = 0.0;
            Dog_moto_1[1].round_cnt = 0;
            Read_state[lur] = 1;
            Read_angle[lur] = Dog_moto_1[1].angle;
        }
        /*LD*/
        if (!Read_ID[ldl] && !Read_state[ldl])
        {
            Read_Cpa[ldl] += 0.01; //-
        }
        else if (Read_ID[ldl] && !Read_state[ldl])
        {
            Read_Cpa[ldl] = 0.0;
            Dog_moto_1[2].round_cnt = 0;
            Read_state[ldl] = 1;
            Read_angle[ldl] = Dog_moto_1[2].angle;
        }
        if (!Read_ID[ldr] && !Read_state[ldr])
        {
            Read_Cpa[ldr] -= 0.01; //+
        }
        else if (Read_ID[ldr] && !Read_state[ldr])
        {
            Read_Cpa[ldr] = 0.0;
            Dog_moto_1[3].round_cnt = 0;
            Read_state[ldr] = 1;
            Read_angle[ldr] = Dog_moto_1[3].angle;
        }
        /*RU*/
        if (!Read_ID[rul] && !Read_state[rul])
        {
            Read_Cpa[rul] += 0.01;
        }
        else if (Read_ID[rul] && !Read_state[rul])
        {
            Read_Cpa[rul] = 0.0;
            Dog_moto_2[0].round_cnt = 0;
            Read_state[rul] = 1;
            Read_angle[rul] = Dog_moto_2[0].angle;
        }
        if (!Read_ID[rur] && !Read_state[rur])
        {
            Read_Cpa[rur] -= 0.01;
        }
        else if (Read_ID[rur] && !Read_state[rur])
        {
            Read_Cpa[rur] = 0.0;
            Dog_moto_2[1].round_cnt = 0;
            Read_state[rur] = 1;
            Read_angle[rur] = Dog_moto_2[1].angle;
        }

        /*RD*/
        if (!Read_ID[rdl] && !Read_state[rdl])
        {
            Read_Cpa[rdl] += 0.01; //-
        }
        else if (Read_ID[rdl] && !Read_state[rdl])
        {
            Read_Cpa[rdl] = 0.0;
            Dog_moto_2[2].round_cnt = 0;
            Read_state[rdl] = 1;
            Read_angle[rdl] = Dog_moto_2[2].angle;
        }
        if (!Read_ID[rdr] && !Read_state[rdr])
        {
            Read_Cpa[rdr] -= 0.01; //-
        }
        else if (Read_ID[rdr] && !Read_state[rdr])
        {
            Read_Cpa[rdr] = 0.0;
            Dog_moto_2[3].round_cnt = 0;
            Read_state[rdr] = 1;
            Read_angle[rdr] = Dog_moto_2[3].angle;
        }
    }
    if (Read_state[lul] && Read_state[lur] && Read_state[ldl] && Read_state[ldr] && Read_state[rul] && Read_state[rur] && Read_state[rdl] && Read_state[rdr]) //
    {
        Read_OK = 1;
    }
}

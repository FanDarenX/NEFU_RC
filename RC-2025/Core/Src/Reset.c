#include "reset.h"

/*GPIO*/
/*
N1O1P1Q1
N2O2P2Q2
*/

int Read_ID[8] = {0};        // 有或无
int Read_state[8] = {0};     // 初始化标志位
int Read_angle[8] = {0};     // 读电机复位时初始角度
double Read_Cpa[8] = {0.0f}; // 初始复位目标值
int Read_OK = 0;

int Read_update(void)
{
    // if (Read_ID[lul])
    //     Read_state[lul] = 2000;
    // if (Read_ID[lur])
    //     Read_state[lur] = 2000;
    // if (Read_ID[rul])
    //     Read_state[rul] = 2000;
    // if (Read_ID[rur])
    //     Read_state[rur] = 2000;
    // if (Read_ID[ldl])
    //     Read_state[ldl] = 2000;
    // if (Read_ID[ldr])
    //     Read_state[ldr] = 2000;
    // if (Read_ID[rdl])
    //     Read_state[rdl] = 2000;
    // if (Read_ID[rdr])
    //     Read_state[rdr] = 2000;

    Read_ID[ldr] = HAL_GPIO_ReadPin(GPIOC, N1_Pin); // ldr
    Read_ID[ldl] = HAL_GPIO_ReadPin(GPIOC, N2_Pin); // ldl
    Read_ID[lur] = HAL_GPIO_ReadPin(GPIOC, O1_Pin); // lur
    Read_ID[lul] = HAL_GPIO_ReadPin(GPIOC, O2_Pin); // lul
    Read_ID[rdl] = HAL_GPIO_ReadPin(GPIOA, P1_Pin); // rdl
    Read_ID[rdr] = HAL_GPIO_ReadPin(GPIOA, P2_Pin); //
    Read_ID[rul] = HAL_GPIO_ReadPin(GPIOF, Q1_Pin); // rul
    Read_ID[rur] = HAL_GPIO_ReadPin(GPIOI, Q2_Pin); // rur
    //! 完全形态时解封
    // if (Read_ID[lul] == 1 && Read_ID[lur] == 1 && Read_ID[rul] == 1 && Read_ID[rur] == 1 && Read_ID[ldl] == 1 && Read_ID[ldr] == 1 && Read_ID[rdl] == 1 && Read_ID[rdr] == 1)
    // {
    //     Read_OK = 1;
    // }
    // else if (Read_ID[lul] == 0 || Read_ID[lur] == 0 || Read_ID[rul] == 0 || Read_ID[rur] == 0 || Read_ID[ldl] == 0 || Read_ID[ldr] == 0 || Read_ID[rdl] == 0 || Read_ID[rdr] == 0)
    // {
    //     Read_OK = 0;
    // }
}

// int State_run(void)
// {
//     if (Read_state[lul] > 0)
//         Read_state[lul]--;
//     if (Read_state[lur] > 0)
//         Read_state[lur]--;
//     if (Read_state[rul] > 0)
//         Read_state[rul]--;
//     if (Read_state[rur] > 0)
//         Read_state[rur]--;
//     if (Read_state[ldl] > 0)
//         Read_state[ldl]--;
//     if (Read_state[ldr] > 0)
//         Read_state[ldr]--;
//     if (Read_state[rdl] > 0)
//         Read_state[rdl]--;
//     if (Read_state[rdr] > 0)
//         Read_state[rdr]--;
// }

void Reset_task(void)
{

    Read_update();
    // State_run();
}
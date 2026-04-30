#ifndef __ROBOTCONTROL_TASK_H
#define __ROBOTCONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "robot_select_init.h"

enum {
    NONE_CMD = 0,
    BootLoader,
    RCJOY_CMD,
    ROS_CMD,
    CAN_CMD,
    UnKnownCMD
};

enum {
    errCode_LowPower        = (1U << 0),
    errCode_StopKeyEn       = (1U << 1),
    errCode_SoftWareStop    = (1U << 2),
    errCode_Driver1_offline = (1U << 3),
    errCode_Driver1_Err     = (1U << 4),
    errCode_Driver2_offline = (1U << 5),
    errCode_Driver2_Err     = (1U << 6),
    errCode_Driver3_offline = (1U << 7),
    errCode_Driver3_Err     = (1U << 8)
};

typedef struct {
    uint8_t cmdsource;
    float Vx;
    float Vy;
    float Vz;
} RobotControlCMDType_t;

extern volatile uint8_t RobotControl_CMDsource;

uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd, BaseType_t* woken);
float target_limit_float(float insert, float low, float high);
int target_limit_int(int insert, int low, int high);
float wheelCoefficient(uint32_t diffparam, uint8_t isLeftWheel);
void _System_Reset_FromAPP_RTOS(char uart_recv);
float rpm_to_linearVel(float rpm, float wheelper);
void RobotControl_SetDebugLevel(char uart_recv);
uint8_t Get_RobotErrorCode(uint32_t error_code);
void RobotControl_task(void* param);

#endif

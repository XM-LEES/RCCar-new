#include "RobotControl_task.h"

#include "main.h"
#include <stdint.h>

#include "bsp_buzzer.h"

QueueHandle_t g_xQueueRobotControl = NULL;
volatile uint8_t RobotControl_CMDsource = NONE_CMD;

void RobotControl_task(void *param)
{
    (void)param;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

uint8_t WriteRobotControlQueue(RobotControlCMDType_t *cmd, BaseType_t *woken)
{
    (void)woken;
    if (cmd == NULL || cmd->cmdsource == NONE_CMD || cmd->cmdsource >= UnKnownCMD)
    {
        return 1U;
    }

    RobotControl_CMDsource = cmd->cmdsource;
    return 0U;
}

float target_limit_float(float insert, float low, float high)
{
    if (insert < low)
    {
        return low;
    }
    if (insert > high)
    {
        return high;
    }
    return insert;
}

int target_limit_int(int insert, int low, int high)
{
    if (insert < low)
    {
        return low;
    }
    if (insert > high)
    {
        return high;
    }
    return insert;
}

float wheelCoefficient(uint32_t diffparam, uint8_t isLeftWheel)
{
    if (isLeftWheel == 1U)
    {
        if (diffparam > 50U)
        {
            return 1.0f + 0.004f * (float)(diffparam - 50U);
        }
    }
    else if (diffparam < 50U)
    {
        return 1.0f + 0.004f * (float)(50U - diffparam);
    }

    return 1.0f;
}

float rpm_to_linearVel(float rpm, float wheelper)
{
    return rpm / 60.0f * wheelper;
}

void _System_Reset_FromAPP_RTOS(char uart_recv)
{
    static char res_buf[5] = {0};
    static uint8_t res_count = 0U;

    res_buf[res_count] = uart_recv;

    if (uart_recv == 'r' || res_count > 0U)
    {
        res_count++;
    }
    else
    {
        res_count = 0U;
    }

    if (res_count != 5U)
    {
        return;
    }

    res_count = 0U;
    if (res_buf[0] == 'r' && res_buf[1] == 'e' && res_buf[2] == 's' && res_buf[3] == 'e' && res_buf[4] == 't')
    {
        RobotControl_CMDsource = BootLoader;
        NVIC_SystemReset();
    }
}

void RobotControl_SetDebugLevel(char uart_recv)
{
    static char res_buf[4] = {0};
    static uint8_t res_count = 0U;

    res_buf[res_count] = uart_recv;

    if (uart_recv == 'L' || res_count > 0U)
    {
        res_count++;
    }
    else
    {
        res_count = 0U;
    }

    if (res_count != 4U)
    {
        return;
    }

    res_count = 0U;
    if (res_buf[0] == 'L' && res_buf[1] == 'O' && res_buf[2] == 'G')
    {
        switch (res_buf[3])
        {
        case '0': RobotControlParam.DebugLevel = 0U; break;
        case '1': RobotControlParam.DebugLevel = 1U; break;
        case '2': RobotControlParam.DebugLevel = 2U; break;
        case '3': RobotControlParam.DebugLevel = 3U; break;
        default:  RobotControlParam.DebugLevel = 0U; break;
        }

        {
            pBuzzerInterface_t tips = &UserBuzzer;
            tips->AddTask(1U, 200U);
        }

    }
}

uint8_t Get_RobotErrorCode(uint32_t error_code)
{
    return ((RobotControlParam.ErrNum & error_code) != 0U) ? 1U : 0U;
}

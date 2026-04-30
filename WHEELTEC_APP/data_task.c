#include "usart.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_adc.h"
#include "bsp_icm20948.h"
#include "robot_select_init.h"
#include "servo_basic_control.h"

extern uint8_t Calculate_BCC(const uint8_t* checkdata, uint16_t datalen);

static UART_HandleTypeDef *serial = &huart4;

#define BaseFRAME_HEAD 0x7B
#define BaseFRAME_TAIL 0x7D
#define BaseFRAME_LEN  24U

#if BaseFRAME_LEN != 24U
#error "UART4 ROS telemetry frame must remain 24 bytes for the upper computer parser."
#endif

static void update_power_state(void)
{
    RobotControlParam.Vol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL) / 4095.0f * 3.3f * 11.0f;
    RobotControlParam.LowPowerFlag = (RobotControlParam.Vol < 11.0f) ? 1U : 0U;
}

void RobotDataTransmitTask(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    const uint16_t TaskFreq = 20U;
    uint8_t basebuffer[BaseFRAME_LEN];

    (void)param;

    for (;;)
    {
        float feedback_vx = 0.0f;
        float feedback_vy = 0.0f;
        float feedback_vz = 0.0f;
        const uint8_t telemetry_en_flag = 1U;

        update_power_state();
        if (ServoBasic_GetOrinFeedback(&feedback_vx, &feedback_vy, &feedback_vz) == 0U)
        {
            feedback_vx = 0.0f;
            feedback_vy = 0.0f;
            feedback_vz = 0.0f;
        }

        basebuffer[0] = BaseFRAME_HEAD;
        basebuffer[1] = telemetry_en_flag;
        basebuffer[2] = (uint8_t)(((int16_t)(feedback_vx * 1000.0f)) >> 8);
        basebuffer[3] = (uint8_t)((int16_t)(feedback_vx * 1000.0f));
        basebuffer[4] = (uint8_t)(((int16_t)(feedback_vy * 1000.0f)) >> 8);
        basebuffer[5] = (uint8_t)((int16_t)(feedback_vy * 1000.0f));
        basebuffer[6] = (uint8_t)(((int16_t)(feedback_vz * 1000.0f)) >> 8);
        basebuffer[7] = (uint8_t)((int16_t)(feedback_vz * 1000.0f));
        basebuffer[8] = (uint8_t)(((int16_t)axis_9ValOri.accel.y) >> 8);
        basebuffer[9] = (uint8_t)((int16_t)axis_9ValOri.accel.y);
        basebuffer[10] = (uint8_t)(((int16_t)(-axis_9ValOri.accel.x)) >> 8);
        basebuffer[11] = (uint8_t)((int16_t)(-axis_9ValOri.accel.x));
        basebuffer[12] = (uint8_t)(((int16_t)axis_9ValOri.accel.z) >> 8);
        basebuffer[13] = (uint8_t)((int16_t)axis_9ValOri.accel.z);
        basebuffer[14] = (uint8_t)(((int16_t)axis_9ValOri.gyro.y) >> 8);
        basebuffer[15] = (uint8_t)((int16_t)axis_9ValOri.gyro.y);
        basebuffer[16] = (uint8_t)(((int16_t)(-axis_9ValOri.gyro.x)) >> 8);
        basebuffer[17] = (uint8_t)((int16_t)(-axis_9ValOri.gyro.x));
        basebuffer[18] = (uint8_t)(((int16_t)axis_9ValOri.gyro.z) >> 8);
        basebuffer[19] = (uint8_t)((int16_t)axis_9ValOri.gyro.z);
        basebuffer[20] = (uint8_t)(((int16_t)(RobotControlParam.Vol * 1000.0f)) >> 8);
        basebuffer[21] = (uint8_t)((int16_t)(RobotControlParam.Vol * 1000.0f));
        basebuffer[22] = Calculate_BCC(basebuffer, 22U);
        basebuffer[23] = BaseFRAME_TAIL;

        HAL_UART_Transmit_DMA(serial, basebuffer, BaseFRAME_LEN);

        if (RobotControlParam.DebugLevel == 0U)
        {
            HAL_UART_Transmit_DMA(&huart1, basebuffer, BaseFRAME_LEN);
        }

        vTaskDelayUntil(&preTime, pdMS_TO_TICKS((1000.0f / (float)TaskFreq)));
    }
}

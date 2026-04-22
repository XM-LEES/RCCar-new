#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_adc.h"
#include "bsp_icm20948.h"
#include "sensor_ranger.h"
#include "AutoRecharge_task.h"
#include "main.h"
#include "hall_speed.h"
#include "robot_select_init.h"
#include "servo_basic_control.h"

extern uint8_t Calculate_BCC(const uint8_t* checkdata, uint16_t datalen);

static UART_HandleTypeDef *serial = &huart4;

#define BaseFRAME_HEAD 0x7B
#define BaseFRAME_TAIL 0x7D
#define BaseFRAME_LEN  24

#define HallFRAME_HEAD 0xEB
#define HallFRAME_TAIL 0xED
#define HallFRAME_LEN  32

#define RangerFRAME_HEAD 0xFA
#define RangerFRAME_TAIL 0xFC
#define RangerFRAME_LEN  19

#define ChargeFRAME_HEAD 0x7C
#define ChargeFRAME_TAIL 0x7F
#define ChargeFRAME_LEN   8

volatile uint32_t g_uart4_hall_debug_enable = 0U;

static void update_power_state(void)
{
    RobotControlParam.Vol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL) / 4095.0f * 3.3f * 11.0f;
    RobotControlParam.LowPowerFlag = (RobotControlParam.Vol < 11.0f) ? 1U : 0U;
}

static void pack_be32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

static void pack_be16(uint8_t *dst, int16_t value)
{
    dst[0] = (uint8_t)(((uint16_t)value) >> 8);
    dst[1] = (uint8_t)value;
}

static void build_hall_debug_frame(uint8_t *frame)
{
    hall_speed_state_t hall_snapshot;
    float hall_speed_mps = 0.0f;
    int16_t hall_speed_mmps = 0;

    if (frame == NULL)
    {
        return;
    }

    hall_snapshot = HallSpeed_GetState();
    (void)HallSpeed_GetSignedSpeedMps(&hall_speed_mps);
    hall_speed_mmps = (int16_t)(hall_speed_mps * 1000.0f);

    frame[0] = HallFRAME_HEAD;
    frame[1] = (HAL_GPIO_ReadPin(HallA_GPIO_Port, HallA_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    frame[2] = (HAL_GPIO_ReadPin(HallB_GPIO_Port, HallB_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
    frame[3] = (uint8_t)hall_snapshot.direction;
    frame[4] = hall_snapshot.speed_valid;
    frame[5] = hall_snapshot.timeout_active;
    pack_be32(&frame[6], (uint32_t)hall_snapshot.event_count_total);
    pack_be32(&frame[10], hall_snapshot.last_period_us);
    pack_be16(&frame[14], (int16_t)hall_snapshot.fault_count);
    pack_be16(&frame[16], hall_speed_mmps);
    pack_be32(&frame[18], g_hall_exti_irq_count);
    pack_be32(&frame[22], g_hall_exti_callback_count);
    pack_be32(&frame[26], g_hall_accepted_edge_count);
    frame[30] = Calculate_BCC(frame, 30U);
    frame[31] = HallFRAME_TAIL;
}

void RobotDataTransmitTask(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    const uint16_t TaskFreq = 20U;
    uint8_t basebuffer[BaseFRAME_LEN];
    uint8_t uart4buffer[BaseFRAME_LEN + HallFRAME_LEN];
    uint8_t hallbuffer[HallFRAME_LEN];
    uint8_t rangerbuffer[RangerFRAME_LEN];
    uint8_t autorechargerbuffer[ChargeFRAME_LEN];

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

        rangerbuffer[0] = RangerFRAME_HEAD;
        rangerbuffer[1] = (uint8_t)(((int16_t)(RangerHAL_A.dis * 1000.0f)) >> 8);
        rangerbuffer[2] = (uint8_t)((int16_t)(RangerHAL_A.dis * 1000.0f));
        rangerbuffer[3] = (uint8_t)(((int16_t)(RangerHAL_B.dis * 1000.0f)) >> 8);
        rangerbuffer[4] = (uint8_t)((int16_t)(RangerHAL_B.dis * 1000.0f));
        rangerbuffer[5] = (uint8_t)(((int16_t)(RangerHAL_C.dis * 1000.0f)) >> 8);
        rangerbuffer[6] = (uint8_t)((int16_t)(RangerHAL_C.dis * 1000.0f));
        rangerbuffer[7] = (uint8_t)(((int16_t)(RangerHAL_D.dis * 1000.0f)) >> 8);
        rangerbuffer[8] = (uint8_t)((int16_t)(RangerHAL_D.dis * 1000.0f));
        rangerbuffer[9] = (uint8_t)(((int16_t)(RangerHAL_E.dis * 1000.0f)) >> 8);
        rangerbuffer[10] = (uint8_t)((int16_t)(RangerHAL_E.dis * 1000.0f));
        rangerbuffer[11] = (uint8_t)(((int16_t)(RangerHAL_F.dis * 1000.0f)) >> 8);
        rangerbuffer[12] = (uint8_t)((int16_t)(RangerHAL_F.dis * 1000.0f));
        rangerbuffer[13] = 0U;
        rangerbuffer[14] = 0U;
        rangerbuffer[15] = 0U;
        rangerbuffer[16] = 0U;
        rangerbuffer[17] = Calculate_BCC(rangerbuffer, 17U);
        rangerbuffer[18] = RangerFRAME_TAIL;

        autorechargerbuffer[0] = ChargeFRAME_HEAD;
        autorechargerbuffer[1] = (uint8_t)(((int16_t)ChargeDev.ChargingCur) >> 8);
        autorechargerbuffer[2] = (uint8_t)((int16_t)ChargeDev.ChargingCur);
        autorechargerbuffer[3] = ChargeDev.RedNum;
        autorechargerbuffer[4] = ChargeDev.ChargingFlag;
        autorechargerbuffer[5] = RobotControlParam.ChargeMode;
        autorechargerbuffer[6] = Calculate_BCC(autorechargerbuffer, 6U);
        autorechargerbuffer[7] = ChargeFRAME_TAIL;

        if (g_uart4_hall_debug_enable != 1U)
        {
            memset(uart4buffer, 0, sizeof(uart4buffer));
            build_hall_debug_frame(hallbuffer);
            memcpy(uart4buffer, basebuffer, BaseFRAME_LEN);
            memcpy(uart4buffer + BaseFRAME_LEN, hallbuffer, HallFRAME_LEN);
            HAL_UART_Transmit_DMA(serial, uart4buffer, sizeof(uart4buffer));
        }
        else
        {
            HAL_UART_Transmit_DMA(serial, basebuffer, BaseFRAME_LEN);
        }

        if (RobotControlParam.DebugLevel == 0U)
        {
            uint8_t tmp[BaseFRAME_LEN + RangerFRAME_LEN + ChargeFRAME_LEN] = {0U};
            memcpy(tmp, basebuffer, BaseFRAME_LEN);
            memcpy(tmp + BaseFRAME_LEN, rangerbuffer, RangerFRAME_LEN);
            memcpy(tmp + BaseFRAME_LEN + RangerFRAME_LEN, autorechargerbuffer, ChargeFRAME_LEN);
            HAL_UART_Transmit_DMA(&huart1, tmp, sizeof(tmp));
        }

        vTaskDelayUntil(&preTime, pdMS_TO_TICKS((1000.0f / (float)TaskFreq)));
    }
}

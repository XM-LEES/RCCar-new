#include "show_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_oled.h"
#include "bsp_adc.h"
#include "bsp_icm20948.h"
#include "robot_select_init.h"
#include "servo_basic_control.h"

extern volatile uint32_t g_orin_ackermann_wheelbase_mm;
extern volatile uint32_t g_orin_ackermann_track_width_mm;
extern volatile uint32_t g_orin_ackermann_max_steering_millirad;
extern volatile uint32_t g_orin_vx_forward_cap_mmps;
extern volatile uint32_t g_orin_vx_reverse_cap_mmps;

static pOLEDInterface_t oled = &UserOLED;

void oled_page_up(void)
{
}

void oled_page_down(void)
{
}

static void show_ackermann_status(void)
{
    const servo_basic_state_t *state = ServoBasic_GetState();

    oled->ShowString(0, 0, "Ackermann");
    oled->ShowString(0, 12, "Mode:");
    oled->ShowString(42, 12, (state->control_mode == SERVO_CTRL_MODE_RC_PASSTHROUGH) ? "RC " : "AUTO");

    oled->ShowString(0, 24, "ESC:");
    oled->ShowNumber(36, 24, state->esc_pulse_us, 4, 12);
    oled->ShowString(72, 24, "STR:");
    oled->ShowNumber(102, 24, state->servo_pulse_us, 4, 12);

    oled->ShowString(0, 36, "RC:");
    oled->ShowNumber(24, 36, ServoBasic_IsRcOverrideActive(), 1, 12);
    oled->ShowString(44, 36, "GD:");
    oled->ShowNumber(68, 36, ServoBasic_IsRcEmergencyActive(), 1, 12);
    oled->ShowString(86, 36, "V:");
    oled->ShowFloat(98, 36, RobotControlParam.Vol, 2, 2);

    oled->ShowString(0, 48, "WB:");
    oled->ShowNumber(24, 48, g_orin_ackermann_wheelbase_mm, 3, 12);
    oled->ShowString(54, 48, "TW:");
    oled->ShowNumber(78, 48, g_orin_ackermann_track_width_mm, 3, 12);

    oled->RefreshGram();
}

void show_task(void* param)
{
    TickType_t preTime = xTaskGetTickCount();
    const uint16_t TaskFreq = 10U;

    (void)param;

    while (1)
    {
        show_ackermann_status();
        vTaskDelayUntil(&preTime, pdMS_TO_TICKS((1000.0f / (float)TaskFreq)));
    }
}

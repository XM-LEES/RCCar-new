/**
 * @file SerialControl_task.c
 * @brief ROS UART command parser for the Ackermann-only control path.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#include "bsp_buzzer.h"
#include "main.h"
#include "robot_select_init.h"
#include "servo_basic_control.h"

#define ROS_CMD_FRAME_LEN 11U

#if ROS_CMD_FRAME_LEN != 11U
#error "ROS downlink command frame must remain 11 bytes for the firmware parser."
#endif

uint8_t Calculate_BCC(const uint8_t *checkdata, uint16_t datalen)
{
	uint8_t bccval = 0U;
	uint16_t i = 0U;
	for (i = 0U; i < datalen; i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}

static uint8_t serial_control_is_zero_motion(float vx_mps, float vy_mps, float vz_rad_s)
{
	return (vx_mps == 0.0f && vy_mps == 0.0f && vz_rad_s == 0.0f) ? 1U : 0U;
}

static void serial_control_send_zero_command(void)
{
	ServoBasic_UpdateFromOrin(0.0f, 0.0f, 0.0f, 1U);
}

static void serial_control_check_reset(char uart_recv)
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
		NVIC_SystemReset();
	}
}

static void serial_control_set_debug_level(char uart_recv)
{
	static char log_buf[4] = {0};
	static uint8_t log_count = 0U;

	log_buf[log_count] = uart_recv;

	if (uart_recv == 'L' || log_count > 0U)
	{
		log_count++;
	}
	else
	{
		log_count = 0U;
	}

	if (log_count != 4U)
	{
		return;
	}

	log_count = 0U;
	if (log_buf[0] != 'L' || log_buf[1] != 'O' || log_buf[2] != 'G')
	{
		return;
	}

	switch (log_buf[3])
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

void SerialControlTask(void *param)
{
	extern QueueHandle_t g_xQueueROSserial;

	uint8_t recv = 0U;
	uint8_t roscmdBuf[20] = {0U};
	uint8_t roscmdCount = 0U;
	const uint8_t cmdLen = ROS_CMD_FRAME_LEN;
	static uint8_t s_rc_override_blocking = 0U;
	static uint8_t idleCount = 0U;

	(void)param;

	for (;;)
	{
		if (pdPASS != xQueueReceive(g_xQueueROSserial, &recv, portMAX_DELAY))
		{
			continue;
		}

		serial_control_check_reset((char)recv);
		serial_control_set_debug_level((char)recv);

		if (recv == 0x7BU || roscmdCount > 0U)
		{
			roscmdBuf[roscmdCount++] = recv;
		}
		else
		{
			roscmdCount = 0U;
		}

		if (roscmdCount != cmdLen)
		{
			continue;
		}

		roscmdCount = 0U;

		if (roscmdBuf[cmdLen - 1U] != 0x7DU || roscmdBuf[cmdLen - 2U] != Calculate_BCC(roscmdBuf, cmdLen - 2U))
		{
			continue;
		}

		if (roscmdBuf[1] == 4U)
		{
			continue;
		}

		if (roscmdBuf[1] != 0U)
		{
			continue;
		}

		RobotControlParam.SecurityLevel = roscmdBuf[2] & 0x01U;
		RobotControlParam.softwareEnflag = (roscmdBuf[2] & 0x80U) ? 1U : 0U;

		{
			const float vx_mps = (float)((int16_t)(((uint16_t)roscmdBuf[3] << 8) | roscmdBuf[4])) / 1000.0f;
			const float vy_mps = (float)((int16_t)(((uint16_t)roscmdBuf[5] << 8) | roscmdBuf[6])) / 1000.0f;
			const float vz_rad_s = (float)((int16_t)(((uint16_t)roscmdBuf[7] << 8) | roscmdBuf[8])) / 1000.0f;
			const uint8_t flag_stop = (roscmdBuf[2] & 0x80U) ? 1U : 0U;
			const uint8_t motion_is_zero = serial_control_is_zero_motion(vx_mps, vy_mps, vz_rad_s);
			uint8_t allow_serial_motion = 1U;
			const uint8_t rc_override_active = ServoBasic_IsRcOverrideActive();

			if (motion_is_zero != 0U)
			{
				if (idleCount < 10U)
				{
					idleCount++;
				}
			}
			else
			{
				idleCount = 0U;
			}

			if (rc_override_active != 0U)
			{
				if (s_rc_override_blocking == 0U)
				{
					serial_control_send_zero_command();
					s_rc_override_blocking = 1U;
				}
				allow_serial_motion = (motion_is_zero != 0U || flag_stop != 0U) ? 1U : 0U;
			}
			else
			{
				s_rc_override_blocking = 0U;
			}

			if (allow_serial_motion != 0U && (idleCount < 10U || motion_is_zero != 0U || flag_stop != 0U))
			{
				ServoBasic_UpdateFromOrin(vx_mps, vy_mps, vz_rad_s, flag_stop);
			}
		}
	}
}

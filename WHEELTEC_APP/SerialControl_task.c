/**
 * @file SerialControl_task.c
 * @brief ROS UART command parser for the Ackermann-only control path.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#include "bsp_RGBLight.h"
#include "robot_select_init.h"
#include "RGBStripControl_task.h"
#include "servo_basic_control.h"
#include "RobotControl_task.h"

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

void SerialControlTask(void *param)
{
	extern QueueHandle_t g_xQueueROSserial;

	uint8_t recv = 0U;
	uint8_t roscmdBuf[20] = {0U};
	uint8_t roscmdCount = 0U;
	const uint8_t cmdLen = 11U;
	static uint8_t s_rc_override_blocking = 0U;
	static uint8_t idleCount = 0U;

	(void)param;

	for (;;)
	{
		if (pdPASS != xQueueReceive(g_xQueueROSserial, &recv, portMAX_DELAY))
		{
			continue;
		}

		_System_Reset_FromAPP_RTOS((char)recv);
		RobotControl_SetDebugLevel((char)recv);

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
			if (roscmdBuf[2] == 1U)
			{
				userdefine_rgb[0] = 1U;
				userdefine_rgb[1] = roscmdBuf[3];
				userdefine_rgb[2] = roscmdBuf[4];
				userdefine_rgb[3] = roscmdBuf[5];
			}
			else
			{
				userdefine_rgb[0] = 0U;
			}
			continue;
		}

		if (roscmdBuf[1] == 1U || roscmdBuf[1] == 2U)
		{
			RobotControlParam.ChargeMode = 1U;
			continue;
		}

		if (roscmdBuf[1] != 0U)
		{
			continue;
		}

		RobotControlParam.ChargeMode = 0U;
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

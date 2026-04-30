#include "RGBStripControl_task.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//BSP Include File
#include "bsp_RGBLight.h"

//APP Include File
#include "robot_select_init.h"

//灯带控制优先级由高到低
enum{
	RGB_LowPower = 0,
	RGB_UserDefined = 1,
	RGB_Default = 2
};

//用户自定义RGB状态
uint8_t userdefine_rgb[4]={ 0 };

static uint8_t RGB_PriorityManagement(void)
{
	if( RobotControlParam.LowPowerFlag ) return RGB_LowPower;

	if( userdefine_rgb[0]==1 ) 
	{
		return RGB_UserDefined;
	}

	return RGB_Default;
}

void RGBControl_task(void* param)
{
	const uint16_t TaskFreq = 5;
	
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//RGB灯带对象
	pRGBLightInterface_t rgb = &UserRGBLight;
	
	//实时状态机
	uint8_t RGB_Sm = RGB_Default;
	
	//灯带初始化与自检
	rgb->init();
	rgb->SetColor(255,0,0);
	vTaskDelay(1000);
	rgb->SetColor(0,255,0);
	vTaskDelay(1000);
	rgb->SetColor(0,0,255);
	vTaskDelay(1000);
	
	while(1)
	{
		//获取RGB状态灯状态
		RGB_Sm = RGB_PriorityManagement();
		
		switch( RGB_Sm )
		{
			case RGB_LowPower:
				//紫灯低频闪烁
				rgb->SetBlink(100,0,128,2000);
				break;
			case RGB_UserDefined:
				//用户自定义
				rgb->SetColorFade(userdefine_rgb[1],userdefine_rgb[2],userdefine_rgb[3]);
				break;
			case RGB_Default:
				//设置灯带默认状态
				rgb->turnoff();
				break;
			default:
				break;
		}	
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}


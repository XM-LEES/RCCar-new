#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern uint8_t rosbuffer;
    extern uint8_t usart1_buffer;
    extern uint8_t rs485_buffer;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart4)
    {
        extern QueueHandle_t g_xQueueROSserial;
        if (g_xQueueROSserial != NULL)
        {
            xQueueSendFromISR(g_xQueueROSserial, &rosbuffer, &xHigherPriorityTaskWoken);
        }
        HAL_UART_Receive_IT(&huart4, &rosbuffer, 1U);
    }
    else if (huart == &huart1)
    {
        HAL_UART_Receive_IT(&huart1, &usart1_buffer, 1U);
    }
    else if (huart == &huart3)
    {
        HAL_UART_Receive_IT(&huart3, &rs485_buffer, 1U);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

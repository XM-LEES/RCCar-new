#include "can.h"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "bsp_can.h"

extern QueueHandle_t g_xQueueAutoRecharge;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t recvbuffer[8] = {0U};
    CAN_RxHeaderTypeDef RxMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMsg, recvbuffer) != HAL_OK)
    {
        return;
    }

    if (RxMsg.IDE == CAN_ID_STD && RxMsg.StdId == 0x182U && g_xQueueAutoRecharge != NULL)
    {
        CANmsgType_t data = {0};
        data.id = RxMsg.StdId;
        memcpy(data.buffer, recvbuffer, 8U);
        xQueueOverwriteFromISR(g_xQueueAutoRecharge, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t recvbuffer[8] = {0U};
    CAN_RxHeaderTypeDef RxMsg;

    (void)recvbuffer;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxMsg, recvbuffer) != HAL_OK)
    {
        return;
    }
}

void Debug_SendServoExt(uint32_t ext_id, uint8_t cmd, uint16_t value)
{
    (void)ext_id;
    (void)cmd;
    (void)value;
}

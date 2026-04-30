#ifndef __APP_RUNTIME_STATE_H
#define __APP_RUNTIME_STATE_H

#include <stdint.h>

typedef struct
{
    float voltage_v;
    uint8_t debug_level;
    uint32_t uart4_tx_busy_count;
    uint32_t uart4_tx_error_count;
    uint32_t usart1_debug_tx_busy_count;
    uint32_t usart1_debug_tx_error_count;
} AppRuntimeState_t;

extern AppRuntimeState_t g_app_runtime_state;

#endif

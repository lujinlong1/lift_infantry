#ifndef TEST_TELEMETRY_USART_H
#define TEST_TELEMETRY_USART_H
#include <stdint.h>
typedef enum {HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2} HAL_StatusTypeDef;
typedef enum {HAL_DMA_STATE_READY=1,HAL_DMA_STATE_BUSY=2} HAL_DMA_StateTypeDef;
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
typedef struct { volatile HAL_DMA_StateTypeDef State; } DMA_HandleTypeDef;
typedef struct { volatile uint32_t gState; DMA_HandleTypeDef *hdmatx; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart1;
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *handle,uint8_t *data,uint16_t length);
#endif

#ifndef RM3100_H
#define RM3100_H

#include "stm32f1xx_hal.h"

typedef struct
{
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *drdy_port;
    uint16_t drdy_pin;
    uint16_t cycle_count;
} RM3100_t;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
} RM3100_Vector_t;

HAL_StatusTypeDef RM3100_Init(SPI_HandleTypeDef *hspi, RM3100_t *sensor,
                              uint16_t cycle_count);
HAL_StatusTypeDef RM3100_StartSingle(SPI_HandleTypeDef *hspi,
                                     const RM3100_t *sensor);
HAL_StatusTypeDef RM3100_WaitReady(const RM3100_t *sensor,
                                   uint32_t timeout_ms);
HAL_StatusTypeDef RM3100_ReadXYZ(SPI_HandleTypeDef *hspi,
                                 const RM3100_t *sensor,
                                 RM3100_Vector_t *vector);

#endif

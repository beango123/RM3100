#include "rm3100.h"

#define RM3100_REG_POLL       0x00U
#define RM3100_REG_CCX        0x04U
#define RM3100_REG_MX         0x24U
#define RM3100_POLL_ALL_AXES  0x70U
#define RM3100_RESULT_BYTES   9U
#define RM3100_SPI_TIMEOUT_MS 20U

static HAL_StatusTypeDef RM3100_Write(SPI_HandleTypeDef *hspi,
                                      const RM3100_t *sensor,
                                      uint8_t reg, const uint8_t *data,
                                      uint16_t length)
{
    HAL_StatusTypeDef status;
    uint8_t address = reg & 0x3FU;

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, &address, 1U, RM3100_SPI_TIMEOUT_MS);
    if (status == HAL_OK && length > 0U)
        status = HAL_SPI_Transmit(hspi, (uint8_t *)data, length,
                                  RM3100_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    return status;
}

static HAL_StatusTypeDef RM3100_Read(SPI_HandleTypeDef *hspi,
                                     const RM3100_t *sensor,
                                     uint8_t reg, uint8_t *data,
                                     uint16_t length)
{
    HAL_StatusTypeDef status;
    uint8_t address = 0x80U | (reg & 0x3FU);
    uint8_t dummy[RM3100_RESULT_BYTES] = {0U};

    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(hspi, &address, 1U, RM3100_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
        status = HAL_SPI_TransmitReceive(hspi, dummy, data, length,
                                        RM3100_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef RM3100_Init(SPI_HandleTypeDef *hspi, RM3100_t *sensor,
                              uint16_t cycle_count)
{
    uint8_t config[6];
    uint8_t readback[6];
    HAL_StatusTypeDef status;

    if (hspi == NULL || sensor == NULL || cycle_count < 1U)
        return HAL_ERROR;

    sensor->cycle_count = cycle_count;
    config[0] = (uint8_t)(cycle_count >> 8);
    config[1] = (uint8_t)cycle_count;
    config[2] = config[0]; config[3] = config[1];
    config[4] = config[0]; config[5] = config[1];

    status = RM3100_Write(hspi, sensor, RM3100_REG_CCX, config, 6U);
    if (status != HAL_OK)
        return status;
    status = RM3100_Read(hspi, sensor, RM3100_REG_CCX, readback, 6U);
    if (status != HAL_OK)
        return status;

    for (uint8_t i = 0U; i < 6U; ++i)
        if (readback[i] != config[i])
            return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef RM3100_StartSingle(SPI_HandleTypeDef *hspi,
                                     const RM3100_t *sensor)
{
    const uint8_t axes = RM3100_POLL_ALL_AXES;
    return RM3100_Write(hspi, sensor, RM3100_REG_POLL, &axes, 1U);
}

HAL_StatusTypeDef RM3100_WaitReady(const RM3100_t *sensor,
                                   uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(sensor->drdy_port, sensor->drdy_pin) != GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

static int32_t RM3100_Signed24(const uint8_t *bytes)
{
    int32_t value = ((int32_t)bytes[0] << 16) |
                    ((int32_t)bytes[1] << 8) | bytes[2];
    if ((value & 0x00800000L) != 0)
        value -= 0x01000000L;
    return value;
}

HAL_StatusTypeDef RM3100_ReadXYZ(SPI_HandleTypeDef *hspi,
                                 const RM3100_t *sensor,
                                 RM3100_Vector_t *vector)
{
    uint8_t bytes[RM3100_RESULT_BYTES];
    HAL_StatusTypeDef status;

    if (vector == NULL)
        return HAL_ERROR;
    status = RM3100_Read(hspi, sensor, RM3100_REG_MX, bytes,
                         RM3100_RESULT_BYTES);
    if (status != HAL_OK)
        return status;
    vector->x = RM3100_Signed24(&bytes[0]);
    vector->y = RM3100_Signed24(&bytes[3]);
    vector->z = RM3100_Signed24(&bytes[6]);
    return HAL_OK;
}

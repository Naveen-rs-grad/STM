#include "dht11.h"

extern TIM_HandleTypeDef htim2;

static void DHT11_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void DHT11_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ---------- Microsecond delay ---------- */

static void DHT11_DelayUs(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    while (__HAL_TIM_GET_COUNTER(&htim2) < us)
    {
    }
}

static uint8_t DHT11_WaitForState(GPIO_PinState state, uint32_t timeout_us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) != state)
    {
        if (__HAL_TIM_GET_COUNTER(&htim2) >= timeout_us)
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t DHT11_Start(void)
{
    /* STM32 controls the line */
    DHT11_SetOutput();

    /* Pull DATA LOW for at least 18 ms */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(20);

    /* Release the line */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

    /* STM32 now listens */
    DHT11_SetInput();

    /* Wait for DHT11 response:
       DHT11 pulls LOW for about 80 us */
    if (!DHT11_WaitForState(GPIO_PIN_RESET, 100))
        return 0;

    /* DHT11 then pulls HIGH for about 80 us */
    if (!DHT11_WaitForState(GPIO_PIN_SET, 100))
        return 0;

    /* DHT11 then starts the data transmission */
    if (!DHT11_WaitForState(GPIO_PIN_RESET, 100))
        return 0;

    return 1;
}

static uint8_t DHT11_ReadBit(uint8_t *bit)
{
    uint32_t high_time;

    /*
     * Each bit starts with approximately 50 us LOW.
     */
    if (!DHT11_WaitForState(GPIO_PIN_SET, 100))
        return 0;

    /*
     * Now measure how long DATA stays HIGH.
     */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
    {
        if (__HAL_TIM_GET_COUNTER(&htim2) > 100)
        {
            return 0;
        }
    }

    high_time = __HAL_TIM_GET_COUNTER(&htim2);

    /*
     * ~26-28 us = 0
     * ~70 us    = 1
     *
     * 50 us is a convenient threshold.
     */
    if (high_time > 50)
        *bit = 1;
    else
        *bit = 0;

    return 1;
}

static uint8_t DHT11_ReadByte(uint8_t *value)
{
    uint8_t bit;

    *value = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (!DHT11_ReadBit(&bit))
            return 0;

        *value <<= 1;
        *value |= bit;
    }

    return 1;
}

uint8_t DHT11_Read(DHT11_Data_t *data)
{
    uint8_t raw[5];

    if (!DHT11_Start())
        return 0;

    /*
     * DHT11 sends 40 bits:
     *
     * raw[0] = Humidity integer
     * raw[1] = Humidity decimal
     * raw[2] = Temperature integer
     * raw[3] = Temperature decimal
     * raw[4] = Checksum
     */

    for (uint8_t i = 0; i < 5; i++)
    {
        if (!DHT11_ReadByte(&raw[i]))
            return 0;
    }

    if ((uint8_t)(raw[0] + raw[1] + raw[2] + raw[3]) != raw[4]) //checksum verification
    {
        return 0;
    }

    data->humidity = raw[0];
    data->temperature = raw[2];

    return 1;
}

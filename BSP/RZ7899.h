#ifndef __RZ7899_H__
#define __RZ7899_H__

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef enum {
    RZ7899_STOP = 0,
    RZ7899_FORWARD,
    RZ7899_REVERSE
} RZ7899_Direction;

typedef enum {
    RZ7899_PWM_NONE = 0,
    RZ7899_PWM_IN1,
    RZ7899_PWM_IN2,
    RZ7899_PWM_BOTH
} RZ7899_PwmPin;

typedef struct {
    GPIO_TypeDef*      IN1_Port;
    uint16_t           IN1_Pin;
    GPIO_TypeDef*      IN2_Port;
    uint16_t           IN2_Pin;
    RZ7899_PwmPin      PwmPin;
    TIM_HandleTypeDef* htim;
    uint32_t           PwmChannel;
    uint32_t           PwmChannel2;
} RZ7899_Config;

typedef struct {
    RZ7899_Config      Config;
    RZ7899_Direction   Direction;
    uint8_t            DutyCycle;
    SemaphoreHandle_t  Lock;
} RZ7899_Handle;

HAL_StatusTypeDef RZ7899_Init(RZ7899_Handle* handle);
HAL_StatusTypeDef RZ7899_SetSpeed(RZ7899_Handle* handle, uint8_t duty);
HAL_StatusTypeDef RZ7899_SetDirection(RZ7899_Handle* handle, RZ7899_Direction dir);
HAL_StatusTypeDef RZ7899_Forward(RZ7899_Handle* handle, uint8_t duty);
HAL_StatusTypeDef RZ7899_Reverse(RZ7899_Handle* handle, uint8_t duty);
HAL_StatusTypeDef RZ7899_Stop(RZ7899_Handle* handle);

#endif

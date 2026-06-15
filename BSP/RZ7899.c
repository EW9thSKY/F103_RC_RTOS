#include "RZ7899.h"

static uint32_t RZ7899_CalcPulse(TIM_HandleTypeDef* htim, uint8_t duty) {
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
    return ((uint32_t)duty * (period + 1)) / 100;
}

static void RZ7899_SetSpeedInternal(RZ7899_Handle* handle, uint8_t duty) {
    handle->DutyCycle = duty;

    if (handle->Config.PwmPin == RZ7899_PWM_NONE) return;

    uint32_t pulse = RZ7899_CalcPulse(handle->Config.htim, duty);

    if (handle->Config.PwmPin == RZ7899_PWM_BOTH) {
        if (handle->Direction == RZ7899_FORWARD) {
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, pulse);
        } else if (handle->Direction == RZ7899_REVERSE) {
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel2, pulse);
        }
        return;
    }

    int pwm_is_active = (handle->Config.PwmPin == RZ7899_PWM_IN1
                         && handle->Direction == RZ7899_FORWARD)
                        || (handle->Config.PwmPin == RZ7899_PWM_IN2
                            && handle->Direction == RZ7899_REVERSE);

    if (pwm_is_active) {
        __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, pulse);
    }
}

static void RZ7899_SetDirectionInternal(RZ7899_Handle* handle, RZ7899_Direction dir) {
    handle->Direction = dir;

    uint32_t pulse = RZ7899_CalcPulse(handle->Config.htim, handle->DutyCycle);

    if (handle->Config.PwmPin == RZ7899_PWM_BOTH) {
        switch (dir) {
        case RZ7899_FORWARD:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel2, 0);
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, pulse);
            break;
        case RZ7899_REVERSE:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel2, pulse);
            break;
        case RZ7899_STOP:
        default:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel2, 0);
            break;
        }
        return;
    }

    switch (dir) {
    case RZ7899_FORWARD:
        switch (handle->Config.PwmPin) {
        case RZ7899_PWM_IN1:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, pulse);
            HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_RESET);
            break;
        case RZ7899_PWM_IN2:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
            HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_SET);
            break;
        default:
            HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_SET);
            break;
        }
        break;

    case RZ7899_REVERSE:
        switch (handle->Config.PwmPin) {
        case RZ7899_PWM_IN1:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
            HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_SET);
            break;
        case RZ7899_PWM_IN2:
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, pulse);
            HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_RESET);
            break;
        default:
            HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_SET);
            break;
        }
        break;

    case RZ7899_STOP:
    default:
        if (handle->Config.PwmPin != RZ7899_PWM_NONE) {
            __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
        }
        HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_RESET);
        break;
    }
}

HAL_StatusTypeDef RZ7899_Init(RZ7899_Handle* handle) {
    if (handle == NULL) return HAL_ERROR;

    GPIO_InitTypeDef GPIO_Init = {0};

    switch (handle->Config.PwmPin) {
    case RZ7899_PWM_BOTH:
        GPIO_Init.Mode = GPIO_MODE_AF_PP;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_Init.Pull = GPIO_NOPULL;

        GPIO_Init.Pin = handle->Config.IN1_Pin;
        HAL_GPIO_Init(handle->Config.IN1_Port, &GPIO_Init);
        GPIO_Init.Pin = handle->Config.IN2_Pin;
        HAL_GPIO_Init(handle->Config.IN2_Port, &GPIO_Init);

        HAL_TIM_PWM_Start(handle->Config.htim, handle->Config.PwmChannel);
        HAL_TIM_PWM_Start(handle->Config.htim, handle->Config.PwmChannel2);
        __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
        __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel2, 0);
        break;

    case RZ7899_PWM_IN1:
        GPIO_Init.Mode = GPIO_MODE_AF_PP;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_Init.Pull = GPIO_NOPULL;
        GPIO_Init.Pin = handle->Config.IN1_Pin;
        HAL_GPIO_Init(handle->Config.IN1_Port, &GPIO_Init);

        GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_Init.Pin = handle->Config.IN2_Pin;
        HAL_GPIO_Init(handle->Config.IN2_Port, &GPIO_Init);

        HAL_TIM_PWM_Start(handle->Config.htim, handle->Config.PwmChannel);
        __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
        break;

    case RZ7899_PWM_IN2:
        GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_Init.Pull = GPIO_NOPULL;
        GPIO_Init.Pin = handle->Config.IN1_Pin;
        HAL_GPIO_Init(handle->Config.IN1_Port, &GPIO_Init);

        GPIO_Init.Mode = GPIO_MODE_AF_PP;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_Init.Pin = handle->Config.IN2_Pin;
        HAL_GPIO_Init(handle->Config.IN2_Port, &GPIO_Init);

        HAL_TIM_PWM_Start(handle->Config.htim, handle->Config.PwmChannel);
        __HAL_TIM_SET_COMPARE(handle->Config.htim, handle->Config.PwmChannel, 0);
        break;

    default:
        GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_Init.Pull = GPIO_NOPULL;

        GPIO_Init.Pin = handle->Config.IN1_Pin;
        HAL_GPIO_Init(handle->Config.IN1_Port, &GPIO_Init);
        GPIO_Init.Pin = handle->Config.IN2_Pin;
        HAL_GPIO_Init(handle->Config.IN2_Port, &GPIO_Init);
        break;
    }

    HAL_GPIO_WritePin(handle->Config.IN1_Port, handle->Config.IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(handle->Config.IN2_Port, handle->Config.IN2_Pin, GPIO_PIN_RESET);

    handle->Direction = RZ7899_STOP;
    handle->DutyCycle = 0;
    handle->Lock = xSemaphoreCreateMutex();

    return HAL_OK;
}

HAL_StatusTypeDef RZ7899_SetSpeed(RZ7899_Handle* handle, uint8_t duty) {
    if (handle == NULL || duty > 100) return HAL_ERROR;

    if (handle->Lock != NULL) xSemaphoreTake(handle->Lock, portMAX_DELAY);
    RZ7899_SetSpeedInternal(handle, duty);
    if (handle->Lock != NULL) xSemaphoreGive(handle->Lock);

    return HAL_OK;
}

HAL_StatusTypeDef RZ7899_SetDirection(RZ7899_Handle* handle, RZ7899_Direction dir) {
    if (handle == NULL) return HAL_ERROR;

    if (handle->Lock != NULL) xSemaphoreTake(handle->Lock, portMAX_DELAY);
    RZ7899_SetDirectionInternal(handle, dir);
    if (handle->Lock != NULL) xSemaphoreGive(handle->Lock);

    return HAL_OK;
}

HAL_StatusTypeDef RZ7899_Forward(RZ7899_Handle* handle, uint8_t duty) {
    if (handle == NULL || duty > 100) return HAL_ERROR;

    if (handle->Lock != NULL) xSemaphoreTake(handle->Lock, portMAX_DELAY);
    RZ7899_SetDirectionInternal(handle, RZ7899_FORWARD);
    RZ7899_SetSpeedInternal(handle, duty);
    if (handle->Lock != NULL) xSemaphoreGive(handle->Lock);

    return HAL_OK;
}

HAL_StatusTypeDef RZ7899_Reverse(RZ7899_Handle* handle, uint8_t duty) {
    if (handle == NULL || duty > 100) return HAL_ERROR;

    if (handle->Lock != NULL) xSemaphoreTake(handle->Lock, portMAX_DELAY);
    RZ7899_SetDirectionInternal(handle, RZ7899_REVERSE);
    RZ7899_SetSpeedInternal(handle, duty);
    if (handle->Lock != NULL) xSemaphoreGive(handle->Lock);

    return HAL_OK;
}

HAL_StatusTypeDef RZ7899_Stop(RZ7899_Handle* handle) {
    if (handle == NULL) return HAL_ERROR;

    if (handle->Lock != NULL) xSemaphoreTake(handle->Lock, portMAX_DELAY);
    RZ7899_SetDirectionInternal(handle, RZ7899_STOP);
    if (handle->Lock != NULL) xSemaphoreGive(handle->Lock);

    return HAL_OK;
}

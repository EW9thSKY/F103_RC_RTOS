/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RZ7899.h"
#include "CarConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern RZ7899_Handle hMotorLF;
extern RZ7899_Handle hMotorRF;
extern RZ7899_Handle hMotorLR;
extern RZ7899_Handle hMotorRR;

typedef enum {
    STATE_DISABLED = 0,
    STATE_ENABLED
} SystemState;

typedef enum {
    DRIVE_STOP = 0,
    DRIVE_FORWARD,
    DRIVE_LEFT,
    DRIVE_RIGHT
} DriveDirection;

volatile uint8_t  g_SystemEnabled = 0;
volatile DriveDirection g_DriveDir = DRIVE_STOP;
/* USER CODE END Variables */
osThreadId CommandTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId SensorTaskHandle;
osMessageQId BarrierdistanceHandle;
osMessageQId TrackingStatusHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void TrackStop(void);
static void TrackDrive(uint8_t leftSpeed, uint8_t rightSpeed);
static uint8_t ButtonPressed(void);
/* USER CODE END FunctionPrototypes */

void StartCommandTask(void const * argument);
void StartChassisTask(void const * argument);
void StartSensorTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Barrierdistance */
  osMessageQDef(Barrierdistance, 1, uint16_t);
  BarrierdistanceHandle = osMessageCreate(osMessageQ(Barrierdistance), NULL);

  /* definition and creation of TrackingStatus */
  osMessageQDef(TrackingStatus, 5, uint16_t);
  TrackingStatusHandle = osMessageCreate(osMessageQ(TrackingStatus), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CommandTask */
  osThreadDef(CommandTask, StartCommandTask, osPriorityNormal, 0, 128);
  CommandTaskHandle = osThreadCreate(osThread(CommandTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityIdle, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityIdle, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartCommandTask */
/**
  * @brief  Function implementing the CommandTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommandTask */
void StartCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartCommandTask */
  SystemState currentState = STATE_DISABLED;
  osEvent event;
  uint16_t sensor;

  for(;;)
  {
    switch (currentState)
    {
    case STATE_DISABLED:
      TrackStop();
      g_DriveDir = DRIVE_STOP;
      g_SystemEnabled = 0;

      if (ButtonPressed())
      {
        g_SystemEnabled = 1;
        currentState = STATE_ENABLED;
      }
      osDelay(20);
      break;

    case STATE_ENABLED:
      event = osMessageGet(TrackingStatusHandle, 20);
      if (event.status == osEventMessage)
      {
        sensor = (uint16_t)event.value.v;

        {
          int8_t  error = 0;
          uint8_t online = 0;

          if (sensor & SENSOR_L2_MASK) { error -= 2; online = 1; }
          if (sensor & SENSOR_L1_MASK) { error -= 1; online = 1; }
          if (sensor & SENSOR_M_MASK)  {             online = 1; }
          if (sensor & SENSOR_R1_MASK) { error += 1; online = 1; }
          if (sensor & SENSOR_R2_MASK) { error += 2; online = 1; }

          if (!online)
          {
            g_DriveDir = DRIVE_FORWARD;
            TrackDrive(TRACK_LOST_SPEED, TRACK_LOST_SPEED);
          }
          else
          {
            int16_t leftSpeed  = (int16_t)TRACK_BASE_SPEED + error * TRACK_KP;
            int16_t rightSpeed = (int16_t)TRACK_BASE_SPEED - error * TRACK_KP;

            if (leftSpeed  < TRACK_MIN_SPEED) leftSpeed  = TRACK_MIN_SPEED;
            if (leftSpeed  > TRACK_MAX_SPEED) leftSpeed  = TRACK_MAX_SPEED;
            if (rightSpeed < TRACK_MIN_SPEED) rightSpeed = TRACK_MIN_SPEED;
            if (rightSpeed > TRACK_MAX_SPEED) rightSpeed = TRACK_MAX_SPEED;

            if      (error < 0) g_DriveDir = DRIVE_LEFT;
            else if (error > 0) g_DriveDir = DRIVE_RIGHT;
            else                g_DriveDir = DRIVE_FORWARD;

            TrackDrive((uint8_t)leftSpeed, (uint8_t)rightSpeed);
          }
        }
      }

      if (ButtonPressed())
      {
        TrackStop();
        currentState = STATE_DISABLED;
      }
      osDelay(10);
      break;
    }
  }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  DriveDirection lastDir = DRIVE_STOP;

  for(;;)
  {
    if (!g_SystemEnabled)
    {
      HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_RESET);
      lastDir = DRIVE_STOP;
      osDelay(50);
      continue;
    }

    if (g_DriveDir != lastDir)
    {
      switch (g_DriveDir)
      {
      case DRIVE_FORWARD:
        HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_RESET);
        break;

      case DRIVE_LEFT:
        HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_RESET);
        break;

      case DRIVE_RIGHT:
        HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_RESET);
        break;

      case DRIVE_STOP:
      default:
        HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_RESET);
        break;
      }
      lastDir = g_DriveDir;
    }
    osDelay(50);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint16_t sensorData;

  for(;;)
  {
    sensorData = 0;

    if (HAL_GPIO_ReadPin(GPIOB, XJ_5_Pin) == GPIO_PIN_RESET)
      sensorData |= SENSOR_L2_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_4_Pin) == GPIO_PIN_RESET)
      sensorData |= SENSOR_L1_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_3_Pin) == GPIO_PIN_RESET)
      sensorData |= SENSOR_M_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_2_Pin) == GPIO_PIN_RESET)
      sensorData |= SENSOR_R1_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_1_Pin) == GPIO_PIN_RESET)
      sensorData |= SENSOR_R2_MASK;

    osMessagePut(TrackingStatusHandle, sensorData, 0);

    osDelay(15);
  }
  /* USER CODE END StartSensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static uint8_t ButtonPressed(void) {
  if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
  {
    osDelay(KEY_DEBOUNCE_MS);
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        osDelay(KEY_HOLD_MS);
      return 1;
    }
  }
  return 0;
}

static void TrackStop(void) {
  RZ7899_Stop(&hMotorLF);
  RZ7899_Stop(&hMotorRF);
  RZ7899_Stop(&hMotorLR);
  RZ7899_Stop(&hMotorRR);
}

static void TrackDrive(uint8_t leftSpeed, uint8_t rightSpeed) {
  RZ7899_SetDirection(&hMotorLF, MOTOR_LF_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetDirection(&hMotorLR, MOTOR_LR_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetDirection(&hMotorRF, MOTOR_RF_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetDirection(&hMotorRR, MOTOR_RR_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetSpeed(&hMotorLF, leftSpeed);
  RZ7899_SetSpeed(&hMotorLR, leftSpeed);
  RZ7899_SetSpeed(&hMotorRF, rightSpeed);
  RZ7899_SetSpeed(&hMotorRR, rightSpeed);
}
/* USER CODE END Application */


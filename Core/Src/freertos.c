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
typedef struct {
    uint16_t freq;     /* 音符频率 (Hz), 0 = 休止符 */
    uint16_t dur_ms;   /* 音符时长 (ms)             */
} MelodyNote;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MELODY_NOTE_COUNT 42
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

/* CommandTask → ChassisTask 驱动指令 (单次写入, 仅 ChassisTask 读取) */
typedef struct {
    DriveDirection dir;
    uint8_t lfSpeed, lrSpeed, rfSpeed, rrSpeed;
} DriveCommand;

volatile SystemState  g_SystemEnabled   = STATE_DISABLED;
volatile DriveCommand g_DriveCmd        = {DRIVE_STOP, 0, 0, 0, 0};
volatile uint16_t     g_BarrierDistance = 0;

/* 兰花草乐谱 (定义见 Application 区) */
extern const MelodyNote MELODY_LANHUACAO[MELODY_NOTE_COUNT];
/* USER CODE END Variables */
osThreadId CommandTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId SensorTaskHandle;
osMessageQId BarrierdistanceHandle;
osMessageQId TrackingStatusHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void TrackStop(void);
static void TrackDrive(uint8_t lfSpeed, uint8_t lrSpeed,
                        uint8_t rfSpeed, uint8_t rrSpeed);
static uint8_t ButtonPressed(void);
static uint16_t SR04_Measure(void);
static void DWT_DelayUs(uint32_t us);
static uint32_t DWT_GetUs(void);
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

  /*
   * CommandTask — 全局状态机 (唯一决策者)
   *   输入: TrackingStatusHandle, BarrierdistanceHandle 队列
   *   输出: g_DriveCmd (驱动指令), g_SystemEnabled, g_BarrierDistance
   *   严禁直接操作硬件 (电机 / LED / 蜂鸣器)
   */
  uint16_t sensor;

  g_SystemEnabled = STATE_DISABLED;

  for(;;)
  {
    switch (g_SystemEnabled)
    {
    /* ---- 系统关闭: 等待按键启动 ---- */
    case STATE_DISABLED:
      g_DriveCmd.dir = DRIVE_STOP;
      g_BarrierDistance = 0;  /* 清除旧障碍数据 */
      if (ButtonPressed())
        g_SystemEnabled = STATE_ENABLED;
      osDelay(20);
      break;

    /* ---- 系统运行: 非阻塞事件驱动 ---- */
    case STATE_ENABLED:
      {
        /*
         * 双队列均非阻塞读取:
         * - 有新数据 → 立即更新 g_DriveCmd / g_BarrierDistance
         * - 无新数据 → 保持上一轮指令, 状态机不卡顿
         * 避障判断在循迹之后运行, 可覆盖为停车 (最高优先级)
         */
        osEvent evt;

        /* ① 非阻塞消费超声波数据 → 更新 g_BarrierDistance */
        evt = osMessageGet(BarrierdistanceHandle, 0);
        if (evt.status == osEventMessage)
        {
          g_BarrierDistance = (uint16_t)evt.value.v;
        }

        /* ② 非阻塞消费循迹数据 → 更新 g_DriveCmd */
        evt = osMessageGet(TrackingStatusHandle, 0);
        if (evt.status == osEventMessage)
        {
          sensor = (uint16_t)evt.value.v & 0x1F;

          /*
           * 加权质心法
           * 传感器布局: L2(-2) L1(-1) M(0) R1(+1) R2(+2)
           */
          int8_t error = 0;
          if (sensor & SENSOR_L2_MASK) error -= 2;
          if (sensor & SENSOR_L1_MASK) error -= 1;
          if (sensor & SENSOR_R1_MASK) error += 1;
          if (sensor & SENSOR_R2_MASK) error += 2;

          if (sensor == 0x00 || error == 0)
          {
            g_DriveCmd.dir = DRIVE_FORWARD;
            uint8_t spd = (sensor == 0x00) ? TRACK_LOST_SPEED
                                           : TRACK_BASE_SPEED;
            g_DriveCmd.lfSpeed = g_DriveCmd.lrSpeed = spd;
            g_DriveCmd.rfSpeed = g_DriveCmd.rrSpeed = spd;
          }
          else if (error < 0)
          {
            g_DriveCmd.dir = DRIVE_LEFT;
            g_DriveCmd.lfSpeed = 0; g_DriveCmd.lrSpeed = 0;
            g_DriveCmd.rfSpeed = TRACK_TURN_SPEED;
            g_DriveCmd.rrSpeed = TRACK_TURN_SPEED;
          }
          else
          {
            g_DriveCmd.dir = DRIVE_RIGHT;
            g_DriveCmd.lfSpeed = TRACK_TURN_SPEED;
            g_DriveCmd.lrSpeed = TRACK_TURN_SPEED;
            g_DriveCmd.rfSpeed = 0; g_DriveCmd.rrSpeed = 0;
          }
        }

        /* ③ 避障优先: 最新距离 ≤ 阈值 → 覆盖为停车 */
        if (g_BarrierDistance > 0
            && g_BarrierDistance <= OBSTACLE_STOP_CM)
        {
          g_DriveCmd.dir      = DRIVE_STOP;
          g_DriveCmd.lfSpeed  = 0;
          g_DriveCmd.lrSpeed  = 0;
          g_DriveCmd.rfSpeed  = 0;
          g_DriveCmd.rrSpeed  = 0;
        }
      }

      if (ButtonPressed())
      {
        g_DriveCmd.dir = DRIVE_STOP;
        g_SystemEnabled = STATE_DISABLED;
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

  /*
   * ChassisTask — 纯执行层 (严禁决策)
   *   输入: g_DriveCmd (由 CommandTask 写入), g_SystemEnabled
   *   执行: 电机 (TrackStop/TrackDrive), LED 方向灯, 蜂鸣器旋律
   */
  DriveCommand  lastCmd = {DRIVE_STOP, 0, 0, 0, 0};
  static uint8_t  melodyIdx = 0;
  static uint32_t melodyElapsedUs = 0;

  for(;;)
  {
    /* ---- 系统关闭: 全停 + 复位旋律 ---- */
    if (!g_SystemEnabled)
    {
      if (lastCmd.dir != DRIVE_STOP) TrackStop();
      HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      lastCmd.dir = DRIVE_STOP;
      melodyIdx = 0;
      melodyElapsedUs = 0;
      osDelay(50);
      continue;
    }

    /*
     * ---- 电机: 单次快照 + 按需更新 ----
     *
     * 关键: 拷贝 g_DriveCmd 到本地变量后再比较和执行,
     * 避免 TOCTOU 竞态 (比较时读到旧值, 执行时已是新值)。
     * 中间传感器 M=1 时方向切换频繁, 此修复尤为关键。
     */
    DriveCommand cmd;
    cmd.dir      = g_DriveCmd.dir;
    cmd.lfSpeed  = g_DriveCmd.lfSpeed;
    cmd.lrSpeed  = g_DriveCmd.lrSpeed;
    cmd.rfSpeed  = g_DriveCmd.rfSpeed;
    cmd.rrSpeed  = g_DriveCmd.rrSpeed;

    if (cmd.dir      != lastCmd.dir      ||
        cmd.lfSpeed  != lastCmd.lfSpeed  ||
        cmd.lrSpeed  != lastCmd.lrSpeed  ||
        cmd.rfSpeed  != lastCmd.rfSpeed  ||
        cmd.rrSpeed  != lastCmd.rrSpeed)
    {
      if (cmd.dir == DRIVE_STOP)
        TrackStop();
      else
        TrackDrive(cmd.lfSpeed, cmd.lrSpeed,
                   cmd.rfSpeed, cmd.rrSpeed);
      lastCmd = cmd;
    }

    /* ---- LED: 方向指示 (使用快照) ---- */
    HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,
      (cmd.dir == DRIVE_FORWARD || cmd.dir == DRIVE_LEFT)
      ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,
      (cmd.dir == DRIVE_FORWARD || cmd.dir == DRIVE_RIGHT)
      ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin,
      (cmd.dir == DRIVE_LEFT)  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin,
      (cmd.dir == DRIVE_RIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* ---- 蜂鸣器: 小星星旋律循环, 避障时静音 ---- */
    if (g_BarrierDistance > 0 && g_BarrierDistance <= OBSTACLE_STOP_CM)
    {
      /* 有障碍 → 静音 + 复位旋律 */
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      melodyIdx = 0;
      melodyElapsedUs = 0;
    }
    else
    {
      const MelodyNote *note = &MELODY_LANHUACAO[melodyIdx];
      uint32_t remain_us = note->dur_ms * 1000UL - melodyElapsedUs;
      uint32_t chunk_us = (remain_us < MELODY_CHUNK_US) ? remain_us
                                                         : MELODY_CHUNK_US;
      uint32_t end_us = DWT_GetUs() + chunk_us;

      if (note->freq > 0)
      {
        /*
         * DWT 周期直接计算半周期 (跳过 µs 换算, 消除两次舍入)
         *   half_ticks = SysClock / (2 × freq)
         *   72MHz 下精度: 440Hz → 81818 ticks (精确 440.00Hz)
         *   SS8050 在 GPIO HIGH 速度下开关边沿 < 50ns, 方波干净
         */
        uint32_t half_ticks = SystemCoreClock / (note->freq * 2UL);
        while (DWT_GetUs() < end_us)
        {
          uint32_t t0 = DWT->CYCCNT;
          HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
          while ((DWT->CYCCNT - t0) < half_ticks);
          t0 = DWT->CYCCNT;
          HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
          while ((DWT->CYCCNT - t0) < half_ticks);
        }
      }
      else
      {
        while (DWT_GetUs() < end_us);  /* 休止符等待 */
      }

      melodyElapsedUs += chunk_us;
      if (melodyElapsedUs >= note->dur_ms * 1000UL)
      {
        melodyElapsedUs = 0;
        melodyIdx++;
        if (melodyIdx >= MELODY_NOTE_COUNT) melodyIdx = 0;
      }
    }
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    osDelay(10);
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

  /*
   * SensorTask — 纯数据采集 (严禁决策 / 执行)
   *   输出: BarrierdistanceHandle (超声波), TrackingStatusHandle (循迹)
   *   DWT 已在 main.c 调度器启动前初始化
   */
  uint16_t sensorData;
  uint16_t distance;

  for(;;)
  {
    /* ---- SR04 超声波测距 ---- */
    distance = SR04_Measure();
    osMessagePut(BarrierdistanceHandle, distance, 0);

    /* ---- 循迹传感器 ---- */
    sensorData = 0;
    if (HAL_GPIO_ReadPin(GPIOB, XJ_5_Pin) == GPIO_PIN_SET)
      sensorData |= SENSOR_L2_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_4_Pin) == GPIO_PIN_SET)
      sensorData |= SENSOR_L1_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_3_Pin) == GPIO_PIN_SET)
      sensorData |= SENSOR_M_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_2_Pin) == GPIO_PIN_SET)
      sensorData |= SENSOR_R1_MASK;
    if (HAL_GPIO_ReadPin(GPIOA, XJ_1_Pin) == GPIO_PIN_SET)
      sensorData |= SENSOR_R2_MASK;
    osMessagePut(TrackingStatusHandle, sensorData, 0);

    osDelay(15);
  }
  /* USER CODE END StartSensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * 小星星 (Twinkle Twinkle Little Star) — 蜂鸣器优化版
 *
 * 简谱 (1=C, 4/4):
 * | 1  1  5  5 | 6  6  5  - | 4  4  3  3 | 2  2  1  - |
 * | 5  5  4  4 | 3  3  2  - | 5  5  4  4 | 3  3  2  - |
 * | 1  1  5  5 | 6  6  5  - | 4  4  3  3 | 2  2  1  - ||
 *
 * 音名映射 (C 大调): 1=262 2=294 3=330 4=349 5=392 6=440
 *
 * 时长: 基础节拍 MELODY_TEMPO_MS, 四分音符=1拍
 */
#define Q  MELODY_TEMPO_MS        /* 四分音符 (1 拍)        */
#define H  (MELODY_TEMPO_MS * 2)  /* 二分音符 (2 拍)        */

const MelodyNote MELODY_LANHUACAO[MELODY_NOTE_COUNT] = {
    /* 第 1 行 */
    {262, Q}, {262, Q}, {392, Q}, {392, Q},    /* | 1  1  5  5 | */
    {440, Q}, {440, Q}, {392, H},                /* | 6  6  5  - | */
    {349, Q}, {349, Q}, {330, Q}, {330, Q},     /* | 4  4  3  3 | */
    {294, Q}, {294, Q}, {262, H},                /* | 2  2  1  - | */
    /* 第 2 行 */
    {392, Q}, {392, Q}, {349, Q}, {349, Q},     /* | 5  5  4  4 | */
    {330, Q}, {330, Q}, {294, H},                /* | 3  3  2  - | */
    {392, Q}, {392, Q}, {349, Q}, {349, Q},     /* | 5  5  4  4 | */
    {330, Q}, {330, Q}, {294, H},                /* | 3  3  2  - | */
    /* 第 3 行 (重复第 1 行) */
    {262, Q}, {262, Q}, {392, Q}, {392, Q},     /* | 1  1  5  5 | */
    {440, Q}, {440, Q}, {392, H},                /* | 6  6  5  - | */
    {349, Q}, {349, Q}, {330, Q}, {330, Q},     /* | 4  4  3  3 | */
    {294, Q}, {294, Q}, {262, H},                /* | 2  2  1  - | */
};

#undef H
#undef Q

/* ---- DWT 微秒级定时工具 (用于 SR04 超声波测距) ---- */
static void DWT_DelayUs(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);
  while ((DWT->CYCCNT - start) < ticks);
}

static uint32_t DWT_GetUs(void) {
  return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}

/* ---- SR04 超声波测距 ---- */
/*
 * TRIG = PA8 (输出), ECHO = PB9 (输入)
 * 时序: TRIG 发 15µs 高脉冲 → 等待 ECHO 高电平 →
 *       测量 ECHO 高电平脉宽 → 距离(cm) = 脉宽(µs) / 58
 * 返回 0 表示超时/无回波
 */
static uint16_t SR04_Measure(void) {
  uint32_t start, timeout;

  /* 1. TRIG 发 15µs 高脉冲 */
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
  DWT_DelayUs(2);
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
  DWT_DelayUs(15);
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);

  /* 2. 等待 ECHO 上升沿 (带超时) */
  timeout = DWT_GetUs() + SR04_TIMEOUT_US;
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
  {
    if (DWT_GetUs() > timeout) return 0;
  }

  /* 3. 测量 ECHO 高电平脉宽 (带超时) */
  start = DWT_GetUs();
  timeout = start + SR04_TIMEOUT_US;
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
  {
    if (DWT_GetUs() > timeout) return 0;
  }
  uint32_t pulse_us = DWT_GetUs() - start;

  /* 4. 转换为厘米 (声速 340m/s, 往返双程) */
  uint16_t dist_cm = (uint16_t)(pulse_us / 58);

  /* 超量程钳位 */
  if (dist_cm > SR04_MAX_DIST_CM) dist_cm = 0;

  return dist_cm;
}

/*
 * 按键消抖状态机 (非阻塞, 含释放确认)
 *   状态 0: IDLE         → 等待按下
 *   状态 1: DEBOUNCE     → 消抖确认
 *   状态 2: HOLD         → 等待释放
 *   状态 3: DEBOUNCE_UP  → 消抖确认释放 → 回到 IDLE
 *   每次完整按下-释放周期返回 1 (仅一次)
 */
static uint8_t ButtonPressed(void) {
  static uint8_t  btnState = 0;
  static uint32_t btnTimer = 0;
  uint8_t isPressed = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)
                       == GPIO_PIN_RESET);
  uint32_t now = DWT_GetUs() / 1000;  /* ms */

  switch (btnState)
  {
  case 0:  /* IDLE */
    if (isPressed) { btnTimer = now; btnState = 1; }
    break;

  case 1:  /* 按下消抖 */
    if (now - btnTimer >= KEY_DEBOUNCE_MS)
    {
      if (isPressed) { btnState = 2; return 1; }  /* 确认按下 */
      else           { btnState = 0; }             /* 噪声, 回到 IDLE */
    }
    break;

  case 2:  /* 等待释放 */
    if (!isPressed) { btnTimer = now; btnState = 3; }
    break;

  case 3:  /* 释放消抖 */
    if (now - btnTimer >= KEY_DEBOUNCE_MS)
    {
      if (!isPressed) { btnState = 0; }  /* 确认释放 → IDLE */
      else            { btnState = 2; }  /* 仍按下, 回到 HOLD */
    }
    break;
  }
  return 0;
}

static void TrackStop(void) {
  RZ7899_Stop(&hMotorLF);
  RZ7899_Stop(&hMotorRF);
  RZ7899_Stop(&hMotorLR);
  RZ7899_Stop(&hMotorRR);
}

static void TrackDrive(uint8_t lfSpeed, uint8_t lrSpeed,
                        uint8_t rfSpeed, uint8_t rrSpeed) {
  RZ7899_SetDirection(&hMotorLF, MOTOR_LF_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetDirection(&hMotorLR, MOTOR_LR_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetDirection(&hMotorRF, MOTOR_RF_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetDirection(&hMotorRR, MOTOR_RR_INVERT ? RZ7899_REVERSE : RZ7899_FORWARD);
  RZ7899_SetSpeed(&hMotorLF, lfSpeed);
  RZ7899_SetSpeed(&hMotorLR, lrSpeed);
  RZ7899_SetSpeed(&hMotorRF, rfSpeed);
  RZ7899_SetSpeed(&hMotorRR, rrSpeed);
}
/* USER CODE END Application */


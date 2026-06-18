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
#include "oled_ssd1306.h"
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
extern TIM_HandleTypeDef htim4;   /* 供 Calib_SetPWM 使用 */
extern I2C_HandleTypeDef hi2c2;   /* 供 OLED 使用            */

typedef enum {
    STATE_DISABLED = 0,
    STATE_CALIBRATING,    /* 上电自校准                        */
    STATE_CAL_FAILED,     /* 校准失败 → LED 频闪               */
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
volatile uint16_t     g_TrackSensors    = 0;  /* 最新循迹位图 (供 OLED) */
volatile uint32_t     g_RunTimeSec      = 0;  /* 运行秒数 (供 OLED 看门狗) */

/* IR 循迹自校准运行时变量 (仅 CommandTask 访问) */
volatile uint16_t     g_CalibPulse;       /* 当前校准 PWM           */
volatile uint8_t      g_CalibFailed;      /* 校准失败标志 (1=失败)   */

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
static uint8_t  ButtonPressed(void);
static uint16_t SR04_Measure(void);
static void     DWT_DelayUs(uint32_t us);
static uint32_t DWT_GetUs(void);
static void     Calib_SetPWM(uint16_t pulse);
static uint8_t  Calib_ReadMidSensors(void);
static uint8_t  Calib_ReadAllSensors(void);
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
   *   状态: DISABLED → CALIBRATING → (CAL_FAILED) → ENABLED
   *   严禁直接操作硬件 (电机 / LED / 蜂鸣器)
   */
  uint16_t sensor;

  g_SystemEnabled = STATE_DISABLED;

  for(;;)
  {
    switch (g_SystemEnabled)
    {
    /* =========================================================== */
    /*  系统关闭: 等待按键                                          */
    /* =========================================================== */
    case STATE_DISABLED:
      g_DriveCmd.dir = DRIVE_STOP;
      g_BarrierDistance = 0;
      g_CalibFailed = 0;
      if (ButtonPressed())
        g_SystemEnabled = STATE_CALIBRATING;
      osDelay(20);
      break;

    /* =========================================================== */
    /*  上电自校准: IR 阈值搜索 (上限 → 下限 → 均值)               */
    /* =========================================================== */
    case STATE_CALIBRATING:
      {
        uint32_t upCoarse, upFine;     /* 上限: ≥1 中间 HIGH    */
        uint32_t loCoarse, loFine;     /* 下限: 全部 5 路 HIGH    */
        uint32_t scanHi, scanLo;

        g_CalibPulse = 65535;
        Calib_SetPWM(g_CalibPulse);
        g_DriveCmd.dir = DRIVE_STOP;

        /* ===================================================== */
        /*  Phase 1: 找上限 (≥1 中间传感器稳定 HIGH)             */
        /* ===================================================== */
        upCoarse = 0;
        while (g_CalibPulse >= CALIB_MIN_PULSE)
        {
          osDelay(CALIB_COARSE_MS);
          if (Calib_ReadMidSensors())
          {
            upCoarse = g_CalibPulse;
            break;
          }
          if (g_CalibPulse < CALIB_COARSE_STEP + CALIB_MIN_PULSE) break;
          g_CalibPulse -= CALIB_COARSE_STEP;
          Calib_SetPWM(g_CalibPulse);
        }
        if (upCoarse == 0) goto calib_fail;  /* 全程无信号 */

        /* 上限精扫: 在 [upCoarse+5%, upCoarse-5%] 间以 1% 步长 */
        scanHi = (upCoarse + CALIB_COARSE_STEP <= 65535)
                  ? upCoarse + CALIB_COARSE_STEP : 65535;
        scanLo = (upCoarse >= CALIB_COARSE_STEP + CALIB_MIN_PULSE)
                  ? upCoarse - CALIB_COARSE_STEP : CALIB_MIN_PULSE;
        upFine = upCoarse;  /* fallback */
        g_CalibPulse = (uint16_t)scanHi;
        while (g_CalibPulse >= scanLo)
        {
          Calib_SetPWM(g_CalibPulse);
          osDelay(CALIB_FINE_MS);
          if (Calib_ReadMidSensors())
          {
            upFine = g_CalibPulse;
            break;
          }
          if (g_CalibPulse < CALIB_FINE_STEP) break;
          g_CalibPulse -= CALIB_FINE_STEP;
        }

        /* ===================================================== */
        /*  Phase 2: 找下限 (全部 5 路传感器稳定 HIGH)             */
        /*  从上限值继续向下扫描                                   */
        /* ===================================================== */
        loCoarse = 0;
        g_CalibPulse = (uint16_t)(upFine > CALIB_COARSE_STEP
                                   ? upFine - CALIB_COARSE_STEP
                                   : CALIB_MIN_PULSE);
        while (g_CalibPulse >= CALIB_MIN_PULSE)
        {
          Calib_SetPWM(g_CalibPulse);
          osDelay(CALIB_COARSE_MS);
          if (Calib_ReadAllSensors())
          {
            loCoarse = g_CalibPulse;
            break;
          }
          if (g_CalibPulse < CALIB_COARSE_STEP + CALIB_MIN_PULSE) break;
          g_CalibPulse -= CALIB_COARSE_STEP;
        }

        if (loCoarse > 0)
        {
          /* 下限精扫 */
          scanHi = (loCoarse + CALIB_COARSE_STEP <= 65535)
                    ? loCoarse + CALIB_COARSE_STEP : 65535;
          scanLo = (loCoarse >= CALIB_COARSE_STEP + CALIB_MIN_PULSE)
                    ? loCoarse - CALIB_COARSE_STEP : CALIB_MIN_PULSE;
          loFine = loCoarse;
          g_CalibPulse = (uint16_t)scanHi;
          while (g_CalibPulse >= scanLo)
          {
            Calib_SetPWM(g_CalibPulse);
            osDelay(CALIB_FINE_MS);
            if (Calib_ReadAllSensors())
            {
              loFine = g_CalibPulse;
              break;
            }
            if (g_CalibPulse < CALIB_FINE_STEP) break;
            g_CalibPulse -= CALIB_FINE_STEP;
          }
        }
        else
        {
          /* 下限未命中: 用最低值作为下限 */
          loFine = CALIB_MIN_PULSE;
        }

        /* ===================================================== */
        /*  Phase 3: 取均值作为最终阈值                           */
        /* ===================================================== */
        {
          uint32_t avg = (upFine + loFine) / 2;
          g_CalibPulse = (uint16_t)avg;
          Calib_SetPWM(g_CalibPulse);
        }
        g_SystemEnabled = STATE_ENABLED;
        break;

calib_fail:
        g_CalibFailed = 1;
        g_SystemEnabled = STATE_CAL_FAILED;
        break;
      }

    /* =========================================================== */
    /*  校准失败: LED 1Hz 频闪 (执行在 ChassisTask)                  */
    /* =========================================================== */
    case STATE_CAL_FAILED:
      g_DriveCmd.dir = DRIVE_STOP;
      if (ButtonPressed())
      {
        g_CalibFailed = 0;
        g_SystemEnabled = STATE_CALIBRATING;  /* 重新校准 */
      }
      osDelay(20);
      break;

    /* =========================================================== */
    /*  系统运行: 循迹 + 避障                                       */
    /* =========================================================== */
    case STATE_ENABLED:
      {
        osEvent evt;

        /* ① 非阻塞消费超声波数据 */
        evt = osMessageGet(BarrierdistanceHandle, 0);
        if (evt.status == osEventMessage)
        {
          g_BarrierDistance = (uint16_t)evt.value.v;
        }

        /* ② 非阻塞消费循迹数据 */
        evt = osMessageGet(TrackingStatusHandle, 0);
        if (evt.status == osEventMessage)
        {
          sensor = (uint16_t)evt.value.v & 0x1F;
          g_TrackSensors = sensor;  /* 快照供 OLED 显示 */

          int8_t error = 0;
          if (sensor & SENSOR_L2_MASK) error -= 2;
          if (sensor & SENSOR_L1_MASK) error -= 1;
          if (sensor & SENSOR_R1_MASK) error += 1;
          if (sensor & SENSOR_R2_MASK) error += 2;

          /*
           * 多级差速转向 (4 级, 含 2 级直行死区)
           *
           *   |error|=0: FORWARD, 内外同速 20          (居中)
           *   |error|=1: FORWARD, 内侧 17 / 外侧 20    (微调, 仍直行)
           *   |error|=2: LEFT/RIGHT, 内侧 10 / 外侧 20 (中度转向)
           *   |error|=3: LEFT/RIGHT, 内侧  2 / 外侧 20 (急转)
           *
           *   后轮 = 前轮 × REAR_RATIO / 100
           *   |error|≤1 归为 FORWARD → 直行状态占比显著增大
           */
          if (sensor == 0x00)
          {
            g_DriveCmd.dir = DRIVE_FORWARD;
            g_DriveCmd.lfSpeed = g_DriveCmd.lrSpeed = TRACK_LOST_SPEED;
            g_DriveCmd.rfSpeed = g_DriveCmd.rrSpeed = TRACK_LOST_SPEED;
          }
          else
          {
            uint8_t absE = (error < 0) ? (uint8_t)(-error)
                                       : (uint8_t)error;
            uint8_t innerF;  /* 内侧前轮 */
            uint8_t outerF;  /* 外侧前轮 */

            if      (absE == 0) { innerF = TRACK_BASE_SPEED;             outerF = TRACK_BASE_SPEED; }
            else if (absE == 1) { innerF = TRACK_BASE_SPEED * 85 / 100; outerF = TRACK_BASE_SPEED; }
            else if (absE == 2) { innerF = TRACK_BASE_SPEED * 50 / 100; outerF = TRACK_BASE_SPEED; }
            else                { innerF = TRACK_BASE_SPEED * 10 / 100; outerF = TRACK_BASE_SPEED; }

            /* 方向: |error|≤1 → 直行; |error|≥2 → 转向 */
            if (absE <= 1)
              g_DriveCmd.dir = DRIVE_FORWARD;
            else
              g_DriveCmd.dir = (error < 0) ? DRIVE_LEFT : DRIVE_RIGHT;

            /* 分配左右: error≤0 → 线偏左, 左侧为内轮 */
            if (error <= 0) {
              g_DriveCmd.lfSpeed = innerF; g_DriveCmd.rfSpeed = outerF;
            } else {
              g_DriveCmd.lfSpeed = outerF; g_DriveCmd.rfSpeed = innerF;
            }
            g_DriveCmd.lrSpeed = (uint8_t)((uint32_t)g_DriveCmd.lfSpeed
                                           * TRACK_REAR_RATIO / 100);
            g_DriveCmd.rrSpeed = (uint8_t)((uint32_t)g_DriveCmd.rfSpeed
                                           * TRACK_REAR_RATIO / 100);
          }
        }

        /* ③ 避障优先: 覆盖为停车 */
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
   *   执行: 电机 / LED / 蜂鸣器 / OLED 显示
   */
  DriveCommand  lastCmd = {DRIVE_STOP, 0, 0, 0, 0};
  static uint8_t  melodyIdx = 0;
  static uint32_t melodyElapsedUs = 0;
  static uint8_t  oledInited = 0;
  static uint32_t lastSecTick = 0;    /* 秒计时基准 (ms)       */
  static uint32_t runSec      = 0;    /* 运行秒数累加器         */

  /* OLED 初始化 (仅一次, 含 I2C 恢复) */
  if (!oledInited) {
    OLED_Init(&hi2c2);
    oledInited = 1;
    lastSecTick = DWT_GetUs() / 1000;
  }

  for(;;)
  {
    /* ---- 运行计时 (每 1000ms 自增, 供 OLED 看门狗) ---- */
    {
      uint32_t now = DWT_GetUs() / 1000;
      if (now - lastSecTick >= 1000) {
        runSec++;
        lastSecTick = now;
        g_RunTimeSec = runSec;
      }
    }

    /* ---- OLED 显示: 全局即时刷新 (不受系统状态限制) ---- */
    OLED_ShowStatus((uint8_t)g_SystemEnabled,
                    g_TrackSensors,
                    g_BarrierDistance,
                    g_CalibPulse,
                    g_CalibFailed,
                    runSec);

    /* ---- 非运行态: 全停 + 复位 ---- */
    if (g_SystemEnabled != STATE_ENABLED)
    {
      if (lastCmd.dir != DRIVE_STOP) TrackStop();

      /* CAL_FAILED → LED 1Hz 频闪; 其他非运行态 → 全灭 */
      if (g_SystemEnabled == STATE_CAL_FAILED)
      {
        uint32_t phase = (DWT_GetUs() / 1000) % 1000;  /* 0-999ms */
        uint8_t  on    = (phase < 500);                  /* 500ms 亮/灭 */
        HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(LED_R_GPIO_Port,  LED_R_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin, GPIO_PIN_RESET);
      }

      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      lastCmd.dir = DRIVE_STOP;
      melodyIdx = 0;
      melodyElapsedUs = 0;
      osDelay(20);
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
      (cmd.dir == DRIVE_FORWARD || cmd.dir == DRIVE_RIGHT)
      ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_L_GPIO_Port,  LED_L_Pin,
      (cmd.dir == DRIVE_FORWARD || cmd.dir == DRIVE_LEFT)
      ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_HR_GPIO_Port, LED_HR_Pin,
      (cmd.dir == DRIVE_RIGHT) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_HL_GPIO_Port, LED_HL_Pin,
      (cmd.dir == DRIVE_LEFT)  ? GPIO_PIN_SET : GPIO_PIN_RESET);

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

/* ---- 循迹 IR 自校准辅助函数 ---- */

/* 设置 IR PWM 占空比 (TIM4_CH3) */
static void Calib_SetPWM(uint16_t pulse) {
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse);
}

/*
 * 多次采样中间三路传感器 (L1|M|R1), 检查是否有任意一路稳定为 HIGH
 * 返回 1: 至少一路 ≥ CALIB_STABLE_RATIO 次 HIGH (稳定检测到黑线)
 * 返回 0: 全部不满足条件
 */
static uint8_t Calib_ReadMidSensors(void) {
  uint8_t cnt[3] = {0, 0, 0};  /* L1, M, R1 */

  for (uint8_t i = 0; i < CALIB_SAMPLE_COUNT; i++)
  {
    if (HAL_GPIO_ReadPin(GPIOA, XJ_4_Pin) == GPIO_PIN_SET) cnt[0]++;  /* L1 */
    if (HAL_GPIO_ReadPin(GPIOA, XJ_3_Pin) == GPIO_PIN_SET) cnt[1]++;  /* M  */
    if (HAL_GPIO_ReadPin(GPIOA, XJ_2_Pin) == GPIO_PIN_SET) cnt[2]++;  /* R1 */
    DWT_DelayUs(500);  /* 采样间隔 500µs, 共 8 次 ≈ 4ms */
  }

  return (cnt[0] >= CALIB_STABLE_RATIO ||
          cnt[1] >= CALIB_STABLE_RATIO ||
          cnt[2] >= CALIB_STABLE_RATIO);
}

/*
 * 多次采样全部 5 路传感器, 检查是否全部稳定为 HIGH
 * 用于确定下限阈值 (亮度太低 → 白色也被误判为黑线)
 */
static uint8_t Calib_ReadAllSensors(void) {
  uint8_t cnt[5] = {0, 0, 0, 0, 0};  /* L2, L1, M, R1, R2 */

  for (uint8_t i = 0; i < CALIB_SAMPLE_COUNT; i++)
  {
    if (HAL_GPIO_ReadPin(GPIOB, XJ_5_Pin) == GPIO_PIN_SET) cnt[0]++;  /* L2 */
    if (HAL_GPIO_ReadPin(GPIOA, XJ_4_Pin) == GPIO_PIN_SET) cnt[1]++;  /* L1 */
    if (HAL_GPIO_ReadPin(GPIOA, XJ_3_Pin) == GPIO_PIN_SET) cnt[2]++;  /* M  */
    if (HAL_GPIO_ReadPin(GPIOA, XJ_2_Pin) == GPIO_PIN_SET) cnt[3]++;  /* R1 */
    if (HAL_GPIO_ReadPin(GPIOA, XJ_1_Pin) == GPIO_PIN_SET) cnt[4]++;  /* R2 */
    DWT_DelayUs(500);
  }

  /* 全部 5 路均 ≥ STABLE_RATIO */
  return (cnt[0] >= CALIB_STABLE_RATIO &&
          cnt[1] >= CALIB_STABLE_RATIO &&
          cnt[2] >= CALIB_STABLE_RATIO &&
          cnt[3] >= CALIB_STABLE_RATIO &&
          cnt[4] >= CALIB_STABLE_RATIO);
}

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


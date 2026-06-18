#ifndef __CAR_CONFIG_H__
#define __CAR_CONFIG_H__

/* ================================================================
 *  CarConfig.h — 车辆全局可调参数
 *  修改数值后重新编译即可生效，无需改动业务代码
 * ================================================================ */

/* 循迹红外供电 ---------------------------------------------------*/
/* IR_PWM (PB8 / TIM4_CH3) 占空比，范围 0~65535                    */
/* 值越大 → 红外越亮 → 循迹灵敏度越高                               */
/* 上电自校准后会覆盖此初始值                                        */
#define IR_PWM_PULSE  65535

/* 循迹自校准参数 -------------------------------------------------*/
#define CALIB_COARSE_STEP  1966   /* 粗扫步长 (65535×3%)           */
#define CALIB_FINE_STEP     196   /* 精扫步长 (65535×0.3%)         */
#define CALIB_COARSE_MS     200   /* 粗扫每步稳定时间 (ms)          */
#define CALIB_FINE_MS        80   /* 精扫每步稳定时间 (ms)          */
#define CALIB_SAMPLE_COUNT    8   /* 每步采样次数                   */
#define CALIB_STABLE_RATIO    6   /* 稳定判定: ≥N 次 HIGH (≤8)     */
#define CALIB_MIN_PULSE     655   /* 最低搜索阈值 (1%)              */
#define CALIB_MID_MASK   0x000E   /* L1|M|R1 中间三路 (bit1-3)     */

/* 循迹差速控制 ---------------------------------------------------*/
#define TRACK_BASE_SPEED        40  /* 基准速度 (外侧轮, 0~100%)    */
#define TRACK_LOST_SPEED        20  /* 丢线搜索速度                  */
#define TRACK_REAR_RATIO        50  /* 后轮速度比例 (前轮的 %)       */

/* 电机物理方向 ---------------------------------------------------*/
/* 1 = 该电机安装方向与逻辑方向相反，需反转                           */
/* 以下为实际测试确认的方向                                           */
#define MOTOR_LF_INVERT  1     /* 左前 (LQ) — PB1/PB0            */
#define MOTOR_RF_INVERT  0     /* 右前 (RQ) — PB4/PB5            */
#define MOTOR_LR_INVERT  1     /* 左后 (LH) — PB3/PB6            */
#define MOTOR_RR_INVERT  1     /* 右后 (RH) — PB7/PA11           */

/* 按键 -----------------------------------------------------------*/
#define KEY_DEBOUNCE_MS  50    /* 消抖延时 (ms)                   */
#define KEY_HOLD_MS      10    /* 等待松手轮询间隔 (ms)            */

/* 循迹传感器位掩码 ------------------------------------------------*/
/* 车头从左到右: PB15, PA4, PA3, PA1, PA0                           */
/* bit = 1 表示检测到黑线 (GPIO 高电平)                              */
#define SENSOR_L2_MASK  0x0001 /* 最左  PB15 / XJ_5              */
#define SENSOR_L1_MASK  0x0002 /* 左中  PA4  / XJ_4              */
#define SENSOR_M_MASK   0x0004 /* 中间  PA3  / XJ_3              */
#define SENSOR_R1_MASK  0x0008 /* 右中  PA1  / XJ_2              */
#define SENSOR_R2_MASK  0x0010 /* 最右  PA0  / XJ_1              */

/* SR04 超声波测距 ---------------------------------------------------*/
/* TRIG=PA8, ECHO=PB9                                                */
#define SR04_TIMEOUT_US   30000  /* 超时时间 (us), 对应 ~5m         */
#define SR04_MAX_DIST_CM  450    /* 最大有效距离 (cm)                */
#define OBSTACLE_STOP_CM  10      /* 避障停车距离 (cm)                */

/* 蜂鸣器音乐 -------------------------------------------------------*/
/* PA2 / BUZZER, 无源蜂鸣器                                          */
#define MELODY_TEMPO_MS   180    /* 基础节拍时长 (ms), 越小越快      */
#define MELODY_CHUNK_US   15000  /* 每次迭代最大发声时间 (us)         */

#endif /* __CAR_CONFIG_H__ */

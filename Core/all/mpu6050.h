#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

/* 7 位 I2C 地址 */
#define MPU6050_ADDR 0x68U     /* AD0 接地 */
#define MPU6050_ADDR_ALT 0x69U /* AD0 接高电平 */

/* 寄存器地址 */
/* 采样率分频：采样率 = 1kHz/(1+值) */
#define MPU6050_SMPLRT_DIV 0x19
/* 数字低通滤波(DLPF)配置：0x03 = 44 Hz */
#define MPU6050_CONFIG 0x1A
/* 陀螺仪量程：0x00 = ±250 dps */
#define MPU6050_GYRO_CONFIG 0x1B
/* 加速度量程：0x00 = ±2 g */
#define MPU6050_ACCEL_CONFIG 0x1C
/* 加速度 X/Y/Z 高字节起点（从这里连续读 6 字节） */
#define MPU6050_ACCEL_XOUT_H 0x3B
/* 温度高字节（连续读 2 字节） */
#define MPU6050_TEMP_OUT_H 0x41
/* 陀螺仪 X/Y/Z 高字节起点（从这里连续读 6 字节） */
#define MPU6050_GYRO_XOUT_H 0x43
/* 电源管理 1：bit7 = 器件复位，bit6 = 睡眠 */
#define MPU6050_PWR_MGMT_1 0x6B
/* 器件 ID 寄存器（读它判断芯片型号） */
#define MPU6050_WHO_AM_I 0x75

/* 共享 MPU60x0 寄存器映射的各芯片的 WHO_AM_I 值。
 * 0x68 : MPU6050 / MPU6000   
 * 0x70 : MPU6500            
 * 0x71 : MPU9250
 * 0x73 : MPU9255
 */
#define MPU6050_ID_MPU6050 0x68U
#define MPU6050_ID_MPU6500 0x70U
#define MPU6050_ID_MPU9250 0x71U
#define MPU6050_ID_MPU9255 0x73U

/* 更换模块时，只需要改下面这一行 */
#define MPU6050_WHO_AM_I_VALUE 0x70U /* 换成 MPU6050 时改为 0x68U */

/* 0 : 严格模式，WHO_AM_I 必须等于 MPU6050_WHO_AM_I_VALUE，否则初始化失败。
 * 1 : 同时接受 MPU60x0 系列的其它 ID（0x68 / 0x70 / 0x71 / 0x73），
 *     避免写错一个数字就把板子卡住。 */
#define MPU6050_ACCEPT_FAMILY_IDS 0

/**
 * @brief  一次读取得到的全部物理量
 * @note   accel 单位 g；gyro 单位 deg/s（已扣零偏）；temp 单位 degC，
 *         pitch/roll/yaw 单位度，由互补滤波得到。
 */
typedef struct
{
        /* 三轴加速度，单位 g（±2 g 量程） */
        float accel_x, accel_y, accel_z;
        /* 三轴角速度，单位 deg/s（已扣掉零偏） */
        float gyro_x, gyro_y, gyro_z;
        /* 芯片温度，单位 degC */
        float temp;
        /* 姿态角（度）：pitch 俯仰 / roll 横滚 / yaw 航向 */
        float pitch, roll, yaw;
} MPU6050_Data;

/* 诊断信息结构，由 MPU6050_Init() 填写，用于在 OLED 上显示，
 * 这样没有调试器也能看出"初始化失败"的原因。 */
typedef struct
{
        uint8_t addr7;          /* 应答的 7 位地址                        */
        uint8_t addr_found;     /* 1 = 0x68 或 0x69 上有器件应答          */
        uint8_t who_am_i;       /* WHO_AM_I 的原始值                      */
        uint8_t id_read_ok;     /* 1 = 读回成功且值不是 00/FF             */
        uint8_t who_am_i_ok;    /* 1 = 与 MPU6050_WHO_AM_I_VALUE 相同     */
        uint8_t read_retry;     /* WHO_AM_I 的读取尝试次数                */
        uint8_t first_err_step; /* 0 = 全部写成功，否则为 1..6            */
        uint8_t step[6];        /* 每个写入步骤：0 = HAL_OK，1 = 出错     */
        uint8_t err_code;       /* Init() 返回时的 hi2c1.ErrorCode        */
        uint8_t init_result;    /* 0 = 成功，1 = 失败                     */
} MPU6050_Diag_t;

/* 最近一次 MPU6050_Init() 的诊断信息 */
extern MPU6050_Diag_t mpu6050_diag;
/* 陀螺仪三轴零偏(deg/s)：由 MPU6050_CalibrateGyro() 写入并在线扣除 */
extern float gyro_bias[3];

const char* MPU6050_ChipName(uint8_t id);  /* 返回 "MPU6050"/"MPU6500" 等 */
uint8_t MPU6050_Init(void);                /* 0 = 成功，1 = 失败 */
void MPU6050_CalibrateGyro(void);          /* 陀螺仪零偏校准，需保持静止 */
void MPU6050_ReadAll(MPU6050_Data* data);  /* 读取全部数据并解算姿态 */
uint8_t MPU6050_GetAddr7(void);            /* 实际使用的 7 位地址 */
uint8_t MPU6050_IsKnownWhoAmI(uint8_t id); /* 是否属于 MPU60x0 系列 */

uint32_t MPU6050_GetReadErrors(void); /* 累计读取失败次数 */

#endif

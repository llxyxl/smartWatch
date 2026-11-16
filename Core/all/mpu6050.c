#include "mpu6050.h"

#include <math.h>
#include <string.h>

#include "i2c.h"

#define ACCEL_SENSITIVITY 16384.0f /* +-2g    : 16384 LSB/g      */
#define GYRO_SENSITIVITY 131.0f    /* +-250dps: 131 LSB/(deg/s)  */
/* 温度换算：T(degC) = RAW / TEMP_SENSITIVITY + TEMP_OFFSET） */
#define TEMP_OFFSET 36.53f
/* 温度灵敏度 LSB/degC */
#define TEMP_SENSITIVITY 340.0f
/* 互补滤波系数，陀螺仪积分权重。ALPHA越大越信陀螺仪 */
#define ALPHA 0.98f

#define MPU6050_I2C_TIMEOUT 100U /* ms */
#define MPU6050_ID_RETRY 3U      /* WHO_AM_I 读取尝试次数 */

MPU6050_Diag_t mpu6050_diag;

static uint8_t s_dev_addr7 = (uint8_t)MPU6050_ADDR; /* 7 位地址 */
/* 上一次读取的时刻(ms)，用于算积分间隔 dt */
static uint32_t s_last_tick = 0U;

/* 陀螺仪积分角度 roll */
static float gyro_angle_x = 0.0f;
/* 陀螺仪积分角度 pitch */
static float gyro_angle_y = 0.0f;
/* 陀螺仪积分角度 yaw */
static float gyro_angle_z = 0.0f;

/* 三轴零偏，校准得到，读取时从原始值里扣掉，全局变量方便调试观察 */
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

/* 读取失败累计 */
static volatile uint32_t s_read_errors = 0U;

/* 返回 0 = 成功，1 = 失败 */
static uint8_t MPU_WriteReg(uint8_t reg, uint8_t value)
{
        if (bsp_i2c_mem_write(s_dev_addr7, reg, &value, 1U,
                              MPU6050_I2C_TIMEOUT) != HAL_OK)
        {
                return 1U;
        }
        return 0U;
}

/* 返回 0 = 成功，1 = 失败 */
static uint8_t MPU_ReadRegs(uint8_t reg, uint8_t* buf, uint16_t len)
{
        if (bsp_i2c_mem_read(s_dev_addr7, reg, buf, len,
                             MPU6050_I2C_TIMEOUT) != HAL_OK)
        {
                return 1U;
        }
        return 0U;
}

/* 把 MPU6050_Init() 的最终结果连同 HAL 错误码一起保存下来，这样调用者可以区分各种错误 */
static uint8_t MPU_DiagFinish(uint8_t result)
{
        mpu6050_diag.err_code    = (uint8_t)(bsp_i2c_last_error() & 0xFFU);
        mpu6050_diag.init_result = result;
        return result;
}

/**
 * @brief  读取累计读取失败次数
 * @retval 从上次初始化/清零至今，MPU_ReadRegs 失败了多少次
 */
uint32_t MPU6050_GetReadErrors(void)
{
        return s_read_errors;
}

/**
 * @brief  判断WHO_AM_I的值是否属于本驱动认识的同系列芯片
 * @param  id WHO_AM_I 读到的原始值
 * @retval 1 = 认识，0 = 不认识
 */
uint8_t MPU6050_IsKnownWhoAmI(uint8_t id)
{
        if ((id == MPU6050_ID_MPU6050) || (id == MPU6050_ID_MPU6500) ||
            (id == MPU6050_ID_MPU9250) || (id == MPU6050_ID_MPU9255))
        {
                return 1U;
        }
        return 0U;
}

/**
 * @brief  把 WHO_AM_I 值翻译成型号名
 */
const char* MPU6050_ChipName(uint8_t id)
{
        switch (id)
        {
                case MPU6050_ID_MPU6050:
                        return "MPU6050";
                case MPU6050_ID_MPU6500:
                        return "MPU6500";
                case MPU6050_ID_MPU9250:
                        return "MPU9250";
                case MPU6050_ID_MPU9255:
                        return "MPU9255";
                default:
                        return "UNKNOWN";
        }
}

/**
 * @brief  取当前使用的 7 位 I2C 地址
 * @retval 0x68（AD0=0）或 0x69（AD0=1），由 MPU6050_Init() 探测结果决定
 */
uint8_t MPU6050_GetAddr7(void)
{
        return s_dev_addr7;
}

/** 
 * @brief MPU6050 初始化。
 * @retval 返回值 0 : 成功，1 : 失败，具体原因在 mpu6050_diag
 */
uint8_t MPU6050_Init(void)
{
        /* 6 条配置寄存器的写入结果：0 = 成功，1 = 失败 */
        uint8_t st[6];
        /* WHO_AM_I 读回来的原始值 */
        uint8_t who = 0x00U;
        /* 1 = 0x68 / 0x69 中至少有一个器件应答 */
        uint8_t addr_found = 0U;
        /* 循环变量：用于探测地址、遍历配置步骤、重试读 ID */
        uint8_t i;

        memset(&mpu6050_diag, 0, sizeof(mpu6050_diag));
        mpu6050_diag.init_result = 1U;
        s_read_errors            = 0U;

        /* 寻找设备：先 0x68 (AD0 = 0)，再 0x69 (AD0 = 1) */
        for (i = 0U; i < 2U; i++)
        {
                /* 第 0 轮试 0x68（AD0=0），第 1 轮试 0x69（AD0=1） */
                uint8_t addr7 = (i == 0U) ? (uint8_t)MPU6050_ADDR
                                          : (uint8_t)MPU6050_ADDR_ALT;

                /* 上电阶段先把总线清干净，因为万一从机处于半传输状态，HAL_I2C_IsDeviceReady() 本身就会一直失败。 */
                bsp_i2c_recover();

                if (bsp_i2c_is_device_ready(addr7, 3U, 20U) != 0U)
                {
                        mpu6050_diag.addr7 = addr7;
                        s_dev_addr7        = addr7;
                        addr_found         = 1U;
                        break;
                }
        }

        mpu6050_diag.addr_found = addr_found;
        if (addr_found == 0U)
        {
                return MPU_DiagFinish(1U); /* 0x68/0x69 上都没有器件应答 */
        }

        /* 配置序列，每一步的结果都会被记录 */
        st[0] = MPU_WriteReg(MPU6050_PWR_MGMT_1, 0x80U);   /* 复位         */
        HAL_Delay(200U);                                   /* 等待启动     */
        st[1] = MPU_WriteReg(MPU6050_PWR_MGMT_1, 0x00U);   /* 唤醒         */
        st[2] = MPU_WriteReg(MPU6050_SMPLRT_DIV, 0x04U);   /* 采样率 1kHz  */
        st[3] = MPU_WriteReg(MPU6050_CONFIG, 0x03U);       /* DLPF 44Hz    */
        st[4] = MPU_WriteReg(MPU6050_ACCEL_CONFIG, 0x00U); /* 量程 +-2g    */
        st[5] = MPU_WriteReg(MPU6050_GYRO_CONFIG, 0x00U);  /* 量程 250dps  */

        for (i = 0U; i < 6U; i++)
        {
                mpu6050_diag.step[i] = st[i];
                if ((st[i] != 0U) && (mpu6050_diag.first_err_step == 0U))
                {
                        mpu6050_diag.first_err_step = (uint8_t)(i + 1U);
                }
        }

        /* 读取 WHO_AM_I，失败可重试 */
        for (i = 0U; i < MPU6050_ID_RETRY; i++)
        {
                mpu6050_diag.read_retry = (uint8_t)(i + 1U);

                if (MPU_ReadRegs(MPU6050_WHO_AM_I, &who, 1U) == 0U)
                {
                        mpu6050_diag.id_read_ok = 1U;
                        break;
                }
                HAL_Delay(20U);
        }

        mpu6050_diag.who_am_i = who;

        /* 总线只回 0x00/0xFF 说明数据并没有真正读回来 */
        if ((who == 0x00U) || (who == 0xFFU))
        {
                mpu6050_diag.id_read_ok = 0U;
        }

        if (mpu6050_diag.id_read_ok == 0U)
        {
                return MPU_DiagFinish(1U); /* 寄存器读不到 */
        }

        /* 芯片返回的 ID 是否等于本驱动期望的值 */
        mpu6050_diag.who_am_i_ok = (uint8_t)(who == (uint8_t)MPU6050_WHO_AM_I_VALUE);

#if (MPU6050_ACCEPT_FAMILY_IDS != 0)
        if ((mpu6050_diag.who_am_i_ok == 0U) && (MPU6050_IsKnownWhoAmI(who) != 0U))
        {
                mpu6050_diag.who_am_i_ok = 1U; /* 同系列芯片，予以容忍 */
        }
#endif

        if (mpu6050_diag.who_am_i_ok == 0U)
        {
                return MPU_DiagFinish(1U); /* 芯片 ID 不符 */
        }

        return MPU_DiagFinish(0U);
}

/*
 * 陀螺仪零偏校准。校准期间板子必须完全静止
 * 200 次采样、每次间隔 5 ms，总时长约 1 s
 * main.c 里在这段前后各喂一次狗，避免 8 s 看门狗被校准拖超时
 * @note 校准结束会把 gyro_angle_x/y/z 清零，避免把静止误差当成初始姿态
 */
void MPU6050_CalibrateGyro(void)
{
        /* 三轴角速度原始值累加和（用 int32 防止 200 次累加溢出） */
        int32_t sum[3] = {0, 0, 0};
        /* GYRO_XOUT_H..GYRO_ZOUT_L 共 6 字节原始数据 */
        uint8_t buf[6];
        /* 真正读成功的样本数 */
        uint16_t samples = 0U;
        /* 采样次数计数（共 200 次，每次间隔 5 ms） */
        int i;

        for (i = 0; i < 200; i++)
        {
                if (MPU_ReadRegs(MPU6050_GYRO_XOUT_H, buf, 6U) != 0U)
                {
                        continue; /* 跳过读失败的样本 */
                }

                sum[0] += (int32_t)(int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
                sum[1] += (int32_t)(int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
                sum[2] += (int32_t)(int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
                samples++;

                HAL_Delay(5U);
        }

        if (samples == 0U)
        {
                return; /* 一次都没读成功：保持上一次的零偏 */
        }

        gyro_bias[0] = (float)sum[0] / (float)samples / GYRO_SENSITIVITY;
        gyro_bias[1] = (float)sum[1] / (float)samples / GYRO_SENSITIVITY;
        gyro_bias[2] = (float)sum[2] / (float)samples / GYRO_SENSITIVITY;

        gyro_angle_x = 0.0f;
        gyro_angle_y = 0.0f;
        gyro_angle_z = 0.0f;
}

/** 
 * 读取加速度计 + 温度 + 陀螺仪，并融合成 pitch / roll / yaw。
 * 读取失败时沿用上一次成功的数据（并累加失败计数）。
 * @param  data 输出结构体（不可为 NULL）；读取失败时写入上一次的值
 */
void MPU6050_ReadAll(MPU6050_Data* data)
{
        /* 上一次成功的数据：读失败时原样返回，让显示/计步不至于跳变 */
        static MPU6050_Data last = {0};

        /* 本次读取的时刻 */
        uint32_t now = HAL_GetTick();
        /* 与上次读取的时间间隔(s)，用于陀螺仪积分 */
        float dt = (float)(now - s_last_tick) / 1000.0f;

        /* ACCEL_XOUT_H..GYRO_ZOUT_L 共 14 字节原始数据 */
        uint8_t buf[14];
        /* 三轴加速度原始值(LSB) */
        int16_t raw_accel_x, raw_accel_y, raw_accel_z;
        /* 温度原始值(LSB) */
        int16_t raw_temp;
        /* 三轴角速度原始值(LSB) */
        int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

        /* 只用加速度计算出的 roll */
        float accel_angle_roll;
        /* 只用加速度计算出的 pitch */
        float accel_angle_pitch;

        if (data == NULL)
        {
                return;
        }

        s_last_tick = now;

        /* dt 限幅：防止第一次调用或长时间丢帧导致积分跳变 */
        if (dt < 0.001f)
        {
                dt = 0.001f;
        }
        if (dt > 0.050f)
        {
                dt = 0.050f;
        }

        if (MPU_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, 14U) != 0U)
        {
                s_read_errors++;
                *data = last; /* 让显示/计步继续用上一次的数据 */
                return;
        }

        raw_accel_x = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
        raw_accel_y = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
        raw_accel_z = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
        raw_temp    = (int16_t)(((uint16_t)buf[6] << 8) | (uint16_t)buf[7]);
        raw_gyro_x  = (int16_t)(((uint16_t)buf[8] << 8) | (uint16_t)buf[9]);
        raw_gyro_y  = (int16_t)(((uint16_t)buf[10] << 8) | (uint16_t)buf[11]);
        raw_gyro_z  = (int16_t)(((uint16_t)buf[12] << 8) | (uint16_t)buf[13]);

        /* 加速度：单位 g */
        data->accel_x = (float)raw_accel_x / ACCEL_SENSITIVITY;
        data->accel_y = (float)raw_accel_y / ACCEL_SENSITIVITY;
        data->accel_z = (float)raw_accel_z / ACCEL_SENSITIVITY;

        /* 温度：单位 degC */
        data->temp = (float)raw_temp / TEMP_SENSITIVITY + TEMP_OFFSET;

        /* 角速度：单位 deg/s，已扣除零偏 */
        data->gyro_x = (float)raw_gyro_x / GYRO_SENSITIVITY - gyro_bias[0];
        data->gyro_y = (float)raw_gyro_y / GYRO_SENSITIVITY - gyro_bias[1];
        data->gyro_z = (float)raw_gyro_z / GYRO_SENSITIVITY - gyro_bias[2];

        /* 姿态解算：加速度计求角度（长期准），陀螺仪积分（短期稳），互补滤波 */
        accel_angle_roll  = atan2f(data->accel_y, data->accel_z) * 57.2958f;
        accel_angle_pitch = atan2f(-data->accel_x,
                                   sqrtf(data->accel_y * data->accel_y +
                                         data->accel_z * data->accel_z)) *
                            57.2958f;

        gyro_angle_x += data->gyro_x * dt;
        gyro_angle_y += data->gyro_y * dt;
        gyro_angle_z += data->gyro_z * dt;

        /* 积分漂移过大时强行与加速度计对齐，避免越走越偏 */
        if (fabsf(gyro_angle_x - accel_angle_roll) > 30.0f)
        {
                gyro_angle_x = accel_angle_roll;
        }
        if (fabsf(gyro_angle_y - accel_angle_pitch) > 30.0f)
        {
                gyro_angle_y = accel_angle_pitch;
        }

        data->roll  = ALPHA * gyro_angle_x + (1.0f - ALPHA) * accel_angle_roll;
        data->pitch = ALPHA * gyro_angle_y + (1.0f - ALPHA) * accel_angle_pitch;
        data->yaw   = gyro_angle_z;

        last = *data;
}

#include "icm42688.h"
#include "i2c.h"
#include "cmsis_os.h"
#include <math.h>

/* 初始化和突发读取参考 STC32G12K128主板DEMOV1.2 的 D9_ICM42688 IIC 例程。 */
#define ICM42688_GYRO_DATA_X1         0x25U
#define ICM42688_PWR_MGMT0            0x4EU
#define ICM42688_GYRO_CONFIG0         0x4FU
#define ICM42688_ACCEL_CONFIG0        0x50U
#define ICM42688_WHO_AM_I             0x75U
#define ICM42688_WHO_AM_I_VALUE       0x47U
#define ICM42688_GYRO_CONFIG_VALUE    0x46U  /* ±500°/s，1 kHz */
#define ICM42688_ACCEL_CONFIG_VALUE   0x46U  /* ±4 g，1 kHz */
#define ICM42688_LOW_NOISE            0x0FU  /* 加速度计和陀螺仪均为低噪声模式 */
#define ICM42688_DPS_PER_COUNT        (500.0f / 32768.0f)
#define ICM42688_I2C_TIMEOUT_MS       10U
#define ICM42688_CALIBRATION_MS       1000U
#define ICM42688_CALIBRATION_STEP_MS  10U
#define ICM42688_FRESH_MS             80U
#define ICM42688_FILTER_TAU_S         0.045f
#define ICM42688_YAW_SIGN             (-1.0f) /* Z 轴向上时，顺时针为车身右转 */

static uint16_t s_address;
static float s_bias_dps;
static float s_filtered_dps;
static volatile float s_yaw_deg;
static volatile uint32_t s_sample_ms;
static volatile ICM42688_Status s_status = ICM42688_STARTING;
static volatile uint32_t s_bus_error;

static bool icm_write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c3, s_address, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, ICM42688_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool icm_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c3, s_address, reg, I2C_MEMADD_SIZE_8BIT,
                            data, len, ICM42688_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool icm_find_address(void)
{
    bool wrong_id = false;
    bool bus_error = false;
    uint32_t hal_errors = 0U;

    for (uint16_t address = 0x69U; address >= 0x68U; address--)
    {
        uint8_t id;
        s_address = address << 1;
        if (icm_read(ICM42688_WHO_AM_I, &id, 1))
        {
            if (id == ICM42688_WHO_AM_I_VALUE) return true;
            wrong_id = true;
        }
        else
        {
            uint32_t error = HAL_I2C_GetError(&hi2c3);
            if (error != HAL_I2C_ERROR_AF)
            {
                bus_error = true;
                hal_errors |= error;
            }
        }
    }

    s_bus_error = hal_errors;
    s_status = wrong_id ? ICM42688_ID_ERROR : (bus_error ? ICM42688_BUS_ERROR : ICM42688_NO_ACK);
    return false;
}

static bool icm_recover_and_probe(void)
{
    switch (I2C3_RecoverBus())
    {
        case I2C3_BUS_IDLE: return icm_find_address();
        case I2C3_BUS_SCL_LOW: s_status = ICM42688_SCL_LOW; break;
        case I2C3_BUS_SDA_LOW: s_status = ICM42688_SDA_LOW; break;
        case I2C3_BUS_BUSY: s_status = ICM42688_BUS_BUSY; break;
        case I2C3_BUS_ERROR:
            s_bus_error = HAL_I2C_GetError(&hi2c3);
            s_status = ICM42688_BUS_ERROR;
            break;
    }
    return false;
}

bool ICM42688_Init(void)
{
    uint8_t power;
    uint8_t gyro_config;
    bool found = false;

    if (s_status == ICM42688_READY) s_status = ICM42688_STARTING;
    if (I2C3_GetBusState() == I2C3_BUS_IDLE)
    {
        found = icm_find_address();
        if (!found && s_status != ICM42688_BUS_ERROR) return false;
    }
    if (!found && !icm_recover_and_probe()) return false;

    if (!icm_write(ICM42688_PWR_MGMT0, 0x00U))
    {
        s_status = ICM42688_CONFIG_ERROR;
        return false;
    }
    osDelay(20);

    if (!icm_write(ICM42688_GYRO_CONFIG0, ICM42688_GYRO_CONFIG_VALUE) ||
        !icm_write(ICM42688_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG_VALUE) ||
        !icm_write(ICM42688_PWR_MGMT0, ICM42688_LOW_NOISE))
    {
        s_status = ICM42688_CONFIG_ERROR;
        return false;
    }
    osDelay(60);

    if (!icm_read(ICM42688_PWR_MGMT0, &power, 1) ||
        !icm_read(ICM42688_GYRO_CONFIG0, &gyro_config, 1) ||
        (power & 0x0FU) != ICM42688_LOW_NOISE || gyro_config != ICM42688_GYRO_CONFIG_VALUE)
    {
        s_status = ICM42688_CONFIG_ERROR;
        return false;
    }
    return true;
}

bool ICM42688_ReadGyroZ(float *rate_dps)
{
    uint8_t raw[6];

    if (!icm_read(ICM42688_GYRO_DATA_X1, raw, sizeof(raw))) return false;
    int16_t counts = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    if (counts == INT16_MIN) return false; /* 传感器无有效角速度时的保留值 */

    *rate_dps = (float)counts * ICM42688_DPS_PER_COUNT;
    return true;
}

bool ICM42688_CalibrateYaw(void)
{
    float previous;
    float angle = 0.0f;
    float min_rate;
    float max_rate;

    if (!ICM42688_ReadGyroZ(&previous))
    {
        s_status = ICM42688_DATA_ERROR;
        return false;
    }
    s_status = ICM42688_CALIBRATING;
    min_rate = previous;
    max_rate = previous;

    uint32_t start_ms = HAL_GetTick();
    uint32_t last_ms = start_ms;
    uint32_t next_tick = osKernelGetTickCount();
    do
    {
        float rate;
        next_tick += ICM42688_CALIBRATION_STEP_MS;
        if (osDelayUntil(next_tick) != osOK) next_tick = osKernelGetTickCount();
        if (!ICM42688_ReadGyroZ(&rate))
        {
            s_status = ICM42688_DATA_ERROR;
            return false;
        }

        uint32_t now_ms = HAL_GetTick();
        angle += 0.5f * (previous + rate) * (float)(now_ms - last_ms) * 0.001f;
        previous = rate;
        last_ms = now_ms;
        if (rate < min_rate) min_rate = rate;
        if (rate > max_rate) max_rate = rate;
    } while (last_ms - start_ms < ICM42688_CALIBRATION_MS);

    /* 静止窗口的角度变化除以窗口时长，得到应从每次读数扣除的零偏。 */
    float bias = angle / ((float)(last_ms - start_ms) * 0.001f);
    if (max_rate - min_rate > 3.0f || fabsf(bias) > 5.0f)
    {
        s_status = ICM42688_NOT_STILL;
        return false;
    }

    s_bias_dps = bias;
    s_filtered_dps = 0.0f;
    s_yaw_deg = 0.0f;
    s_sample_ms = last_ms;
    s_status = ICM42688_READY;
    return true;
}

void ICM42688_UpdateYaw(void)
{
    float raw;
    if (s_status != ICM42688_READY) return;
    bool valid = ICM42688_ReadGyroZ(&raw);

    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - s_sample_ms;
    if (elapsed_ms > ICM42688_FRESH_MS)
    {
        s_status = ICM42688_DATA_ERROR;
        return;
    }
    if (!valid || elapsed_ms == 0U) return;

    float dt = (float)elapsed_ms * 0.001f;
    float rate = (raw - s_bias_dps) * ICM42688_YAW_SIGN;
    float filtered = s_filtered_dps + dt / (ICM42688_FILTER_TAU_S + dt) * (rate - s_filtered_dps);
    float yaw = s_yaw_deg + 0.5f * (s_filtered_dps + filtered) * dt;
    if (yaw < 0.0f) yaw += 360.0f;
    if (yaw >= 360.0f) yaw -= 360.0f;

    s_filtered_dps = filtered;
    s_yaw_deg = yaw;
    s_sample_ms = now_ms;
}

bool ICM42688_GetYaw(float *yaw_deg)
{
    uint32_t sample_ms = s_sample_ms;
    if (s_status != ICM42688_READY || HAL_GetTick() - sample_ms > ICM42688_FRESH_MS) return false;
    *yaw_deg = s_yaw_deg;
    return true;
}

ICM42688_Status ICM42688_GetStatus(void)
{
    return s_status;
}

uint32_t ICM42688_GetBusError(void)
{
    return s_bus_error;
}

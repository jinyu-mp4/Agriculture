#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ICM42688_STARTING,
    ICM42688_BUS_ERROR,
    ICM42688_SCL_LOW,
    ICM42688_SDA_LOW,
    ICM42688_BUS_BUSY,
    ICM42688_NO_ACK,
    ICM42688_ID_ERROR,
    ICM42688_CONFIG_ERROR,
    ICM42688_CALIBRATING,
    ICM42688_NOT_STILL,
    ICM42688_DATA_ERROR,
    ICM42688_READY,
} ICM42688_Status;

/* 单一任务负责 I2C3 初始化、校准和更新。 */
bool ICM42688_Init(void);
bool ICM42688_ReadGyroZ(float *rate_dps);
bool ICM42688_CalibrateYaw(void);
void ICM42688_UpdateYaw(void);

/* 上电校准方向为 0°，向右为正。校准未完成或读数过期时返回 false。 */
bool ICM42688_GetYaw(float *yaw_deg);
ICM42688_Status ICM42688_GetStatus(void);
uint32_t ICM42688_GetBusError(void);

#endif /* ICM42688_H */

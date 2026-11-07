/**
  ******************************************************************************
  * @file    laser.c
  * @brief   单线红外激光测距模块驱动（MINIF / DMP0501C1，串口输出）
  *
  *          串口协议（ASCII，115200 8N1）：
  *            <0x20> 距离(mm) <0x2C> <0x20> 置信度 <0x0A>
  *          有效距离 20~4000mm，置信度 0~100，超范围视为无效。
  *
  *          接收方式：USART2/USART3 单字节中断接收，在
  *          HAL_UART_RxCpltCallback 里逐字节拼帧，不占用任务时间片。
  ******************************************************************************
  */

#include "laser.h"

#define LASER_DIST_MIN   20     /* 最小有效距离(mm) */
#define LASER_DIST_MAX   4000   /* 最大有效距离(mm) */
#define LASER_CONF_MAX   255    /* 置信度上限：实测 0~100，放宽到 255 全范围 */

/* 每个激光模块的接收/解析状态 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t  rx_byte;          /* 单字节 IT 接收缓冲 */

    /* 解析状态机：0 等帧头空格 / 1 距离数字 / 2 等逗号后的空格 / 3 置信度数字 */
    uint8_t  state;
    uint16_t dist;             /* 正在累积的距离 */
    uint8_t  conf;             /* 正在累积的置信度 */

    /* 最新有效结果 */
    volatile uint16_t distance_mm;
    volatile uint8_t  confidence;
    volatile bool     valid;

    /* 诊断计数 */
    volatile uint32_t rx_count;      /* 累计接收字节数 */
    volatile uint32_t frame_count;   /* 累计拼出的完整帧数 */
} LaserDev;

static LaserDev s_laser[LASER_NUM] =
{
    { .huart = &huart2 },   /* LASER_RIGHT 右侧 */
    { .huart = &huart3 },   /* LASER_FRONT 前侧 */
};

/** 把收到的 1 个字节喂给状态机 */
static void laser_parse(LaserDev *d, uint8_t b)
{
    switch (d->state)
    {
        case 0:   /* 等帧头 0x20(空格) */
            if (b == 0x20)
            {
                d->state = 1;
                d->dist  = 0;
                d->conf  = 0;
            }
            break;

        case 1:   /* 距离数字，直到逗号 */
            if (b == 0x2C)
            {
                d->state = 2;
            }
            else if (b >= '0' && b <= '9')
            {
                if (d->dist <= (LASER_DIST_MAX / 10))
                {
                    d->dist = (uint16_t)(d->dist * 10 + (b - '0'));
                }
                else
                {
                    d->dist = LASER_DIST_MAX + 1;   /* 溢出保护：必判无效 */
                }
            }
            else
            {
                d->state = 0;   /* 格式错，复位 */
            }
            break;

        case 2:   /* 逗号后的空格分隔符 */
            if (b == 0x20)
            {
                d->state = 3;
            }
            else
            {
                d->state = 0;   /* 格式错，复位 */
            }
            break;

        case 3:   /* 置信度数字，直到换行（兼容 0x0A / 0x0D 结尾） */
            if (b == 0x0A || b == 0x0D)
            {
                d->frame_count++;
                if (d->dist >= LASER_DIST_MIN && d->dist <= LASER_DIST_MAX &&
                    d->conf <= LASER_CONF_MAX)
                {
                    d->distance_mm = d->dist;
                    d->confidence  = d->conf;
                    d->valid       = true;
                }
                d->state = 0;
            }
            else if (b >= '0' && b <= '9')
            {
                if (d->conf < 200)
                {
                    d->conf = (uint8_t)(d->conf * 10 + (b - '0'));
                }
            }
            else
            {
                d->state = 0;   /* 格式错，复位 */
            }
            break;

        default:
            d->state = 0;
            break;
    }
}

/** 由串口句柄反查激光编号，找不到返回 LASER_NUM */
static LaserID laser_uart_to_id(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) return LASER_RIGHT;
    if (huart == &huart3) return LASER_FRONT;
    return LASER_NUM;
}

void Laser_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &s_laser[LASER_RIGHT].rx_byte, 1);
    HAL_UART_Receive_IT(&huart3, &s_laser[LASER_FRONT].rx_byte, 1);
}

void Laser_FeedByte(UART_HandleTypeDef *huart)
{
    LaserID id = laser_uart_to_id(huart);
    if (id >= LASER_NUM) return;

    s_laser[id].rx_count++;
    laser_parse(&s_laser[id], s_laser[id].rx_byte);
    HAL_UART_Receive_IT(huart, &s_laser[id].rx_byte, 1);
}

void Laser_UART_Error(UART_HandleTypeDef *huart)
{
    LaserID id = laser_uart_to_id(huart);
    if (id >= LASER_NUM) return;

    HAL_UART_AbortReceive_IT(huart);
    HAL_UART_Receive_IT(huart, &s_laser[id].rx_byte, 1);
}

uint16_t Laser_GetDistance(LaserID id)
{
    if (id >= LASER_NUM) return 0;
    return s_laser[id].distance_mm;
}

uint8_t Laser_GetConfidence(LaserID id)
{
    if (id >= LASER_NUM) return 0;
    return s_laser[id].confidence;
}

bool Laser_IsValid(LaserID id)
{
    if (id >= LASER_NUM) return false;
    return s_laser[id].valid;
}

uint32_t Laser_GetRxCount(LaserID id)
{
    if (id >= LASER_NUM) return 0;
    return s_laser[id].rx_count;
}

uint32_t Laser_GetFrameCount(LaserID id)
{
    if (id >= LASER_NUM) return 0;
    return s_laser[id].frame_count;
}

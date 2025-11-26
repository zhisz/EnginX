#ifndef __C620_H
#define __C620_H

#include "interface.h"
#include "tinybus.h"

#define C620_MOTOR_NUM 8  // 电机数量

//-16384~16384 = -20A~20A
#define C620_CURRENT_MAX 16384  //+-20A
#define C620_ANGLE_MAX 8191     // 0~360°

// 转矩常数
#define C620_TORQUE_CONST 0.3f

#define C620_ID_BASE 0x200

typedef struct C620_Config {
    uint32_t motor_id;
    char *motor_ptr_name;  // 共享指针名
    float angle_offset;
    float gear_ratio;                   // 减速比 默认0=19.203 影响速度角度
    float direction;                    // 电机方向（电机角度、输出乘以该系数，设置-1反向）
    INTF_Motor_ModeTypeDef motor_mode;  // 运行模式
    char *can_rx_topic_name;
    char *can_tx_topic_name;
    float *other_feedback_of_angle;  // 其他的角度源(用于替换PID闭环)为空时默认使用电机本身传感器
    float *other_feedback_of_speed;  // 其他的速度源(用于替换PID闭环)为空时默认使用电机本身传感器
    float torque_feed_forward;       // 力矩环前馈参数
    PID_Init_Config_s *speed_pid_config;
    PID_Init_Config_s *angle_pid_config;
    PID_Init_Config_s *torque_pid_config;
} C620_ConfigTypeDef;

typedef struct C620_ResData {
    BusSubscriberHandle_t can_rx_topic;
    BusTopicHandle_t can_tx_topic;
    PIDInstance speed_pid;
    PIDInstance angle_pid;
    PIDInstance torque_pid;
    float *other_feedback_of_angle;
    float *other_feedback_of_speed;
    float gear_ratio;  // 减速比
    int32_t total_rounds;
    float offset_angle;
    int16_t last_ecd;
    float torque_feed_forward;  // 力矩环前馈参数
} C620_ResDataTypeDef;




INTF_Motor_HandleTypeDef *C620_Register(C620_ConfigTypeDef *config);
int16_t C620_Torque2Current(float torque);
#endif
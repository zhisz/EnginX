//
// Created by Ukua on 2023/9/19.
//

#ifndef JUSTFW_INTF_MOTOR_H
#define JUSTFW_INTF_MOTOR_H

#include <stdint.h>

#define RPS2RPM (1.0f / (2.0f * 3.1415926f) / 60.0f)
#define RPM2RPS (60.0f * 2.0f * 3.1415926f)

#define DEG2RAD (3.1415926f / 180.0f)
#define RAD2DEG (180.0f / 3.1415926f)

typedef enum INTF_Motor_Mode {
    MOTOR_MODE_TORQUE = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_ANGLE,
    MOTOR_MODE_MIT
} INTF_Motor_ModeTypeDef;

typedef enum INTF_Motor_State {
    MOTOR_STATE_INIT = 0,  // 待初始化
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_STUCK,  // 堵转
    MOTOR_STATE_ERROR,
    MOTOR_STATE_DISABLE  // 失能
} INTF_Motor_StateTypeDef;

typedef struct INTF_Motor_Handle {
    uint32_t motor_id;
    INTF_Motor_ModeTypeDef motor_mode;    // 运行模式
    INTF_Motor_StateTypeDef motor_state;  // 电机状态
    float target_speed;                   // 单位rad/s
    float real_speed;                     // 单位rad/s
    float target_angle;                   // 单位rad
    float real_angle;                     // 单位rad
    float target_torque;                  // 单位N*m
    float real_torque;                    // 单位N*m
    float min_angle;                      // 最小角度，单位rad
    float max_angle;                      // 最大角度，单位rad
    uint32_t update_time;                 // 电机传感器更新时间，单位ms(HAL_GetTick())

    float direction;  // 电机方向（电机角度、输出乘以该系数，设置-1反向）

    float angle_offset;  // 电机角度偏移 rad

    /**
     * @brief 设置电机模式
     * @param self 电机句柄
     * @param mode 欲设置的电机模式
     */
    void (*set_mode)(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode);  // 设置电机

    /**
     * @brief 设置电机速度
     * @param self 电机句柄
     * @param speed 欲设置的电机速度（rad/s）
     */
    void (*set_speed)(struct INTF_Motor_Handle *self, float speed);  // 设置电机速度

    /**
     * @brief 设置电机角度
     * @param self 电机句柄
     * @param angle 欲设置的电机角度（rad）
     */
    void (*set_angle)(struct INTF_Motor_Handle *self, float angle);  // 设置电机角度

    /**
     * @brief 设置电机力矩
     * @param self 电机句柄
     * @param torque 欲设置的电机力矩（N*m）
     */
    void (*set_torque)(struct INTF_Motor_Handle *self, float torque);  // 设置电机力矩

    /**
     * @brief 重置电机
     * @param self 电机句柄
     */
    void (*reset)(struct INTF_Motor_Handle *self);  // 重置电机

    /**
     * @brief 使能电机
     * @param self 电机句柄
     */
    void (*enable)(struct INTF_Motor_Handle *self);  // 使能电机

    /**
     * @brief 失能电机
     * @param self 电机句柄
     */
    void (*disable)(struct INTF_Motor_Handle *self);  // 失能电机

    void *private_data;  // 电机其他数据
} INTF_Motor_HandleTypeDef;

#endif  // JUSTFW_INTF_MOTOR_H

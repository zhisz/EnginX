//
// Created by liszhi on 25-7-14.
//

#include "chassis.h"
#include <stdio.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "c620.h"

// 轮子中心构成的矩形的大小 单位m
#define Chassis_Width 0.49
#define Chassis_Length 0.35
// 电机的减速比
#define Motor_DECELE_RATIO 1.0f  // 这是3508的
// #define Motor_DECELE_RATIO 19.203f //这是3508的

// 轮子半径 单位m
#define WHEEL_R 0.0763f

RC_ctrl_t *chassis_rc_ctrl;

INTF_Motor_HandleTypeDef *F_RMotor;
INTF_Motor_HandleTypeDef *F_LMotor;
INTF_Motor_HandleTypeDef *B_RMotor;
INTF_Motor_HandleTypeDef *B_LMotor;


void Set_Speed(float speed_x, float speed_y, float speed_w) {
    F_RMotor->set_speed(F_RMotor, ((speed_x - speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO / WHEEL_R));
    F_LMotor->set_speed(F_LMotor, ((speed_x + speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO / WHEEL_R));
    B_LMotor->set_speed(B_LMotor, ((-speed_x + speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO / WHEEL_R));
    B_RMotor->set_speed(B_RMotor, ((-speed_x - speed_y + speed_w * (Chassis_Width + Chassis_Length) * 0.5f) * Motor_DECELE_RATIO / WHEEL_R));
}

void chassis_MainLoop()
{
    float speed_x, speed_y, speed_w;

    while (1)
    {

        while (chassis_rc_ctrl[0].rc.switch_left==1)
        {
            // printf("USB_OK");
            speed_x = chassis_rc_ctrl[0].rc.rocker_l_ / 660.0f * 10;
            speed_y = chassis_rc_ctrl[0].rc.rocker_l1 / 660.0f * 10;
            speed_w = -chassis_rc_ctrl[0].rc.dial / 660.0f * 3 + (chassis_rc_ctrl[0].rc.rocker_r_ / 660.0f * 10);

            Set_Speed(speed_x, speed_y, speed_w);






            extern float INS_angle_N[0];



            // printf("angleXYZ:%f,%f,%f\n",INS_angle_N[0],INS_angle_N[1],INS_angle_N[2]);
            printf("anglex:%f\n",speed_x);
            printf("angley:%f\n",speed_y);
            printf("anglez:%f\n",speed_w);


            vTaskDelay(10);

        }
        vTaskDelay(10);
    }
}

void chassis_Init()
{


    ChassisMotors_Init();
    F_RMotor = pvSharePtr("F_RMotor", sizeof(INTF_Motor_HandleTypeDef));
    F_LMotor = pvSharePtr("F_LMotor", sizeof(INTF_Motor_HandleTypeDef));
    B_RMotor = pvSharePtr("B_RMotor", sizeof(INTF_Motor_HandleTypeDef));
    B_LMotor = pvSharePtr("B_LMotor", sizeof(INTF_Motor_HandleTypeDef));




    chassis_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(chassis_MainLoop, "chassis_MainLoop", 1024, NULL, 240, NULL);
}

////////////////////////////////////////////////////////////////////////////////////////////////
///lift抬升电机配置
void ChassisMotors_Init() {
    PID_Init_Config_s angle_pid = {
        .Kp = 10.0f,       // 提高比例增益，加快响应速度
        .Ki = 4.5f,       // 保持积分增益，维持稳态精度
        .Kd = 0.02f,       // 增大微分增益，抑制抖动
        .MaxOut = 180.0f,
        .DeadBand = 0.11f, // 缩小死区，减少小误差下的抖动
        .Improve = PID_Integral_Limit | PID_OutputFilter,
        .Output_LPF_RC = 0.5f, // 启用低通滤波
};
    PID_Init_Config_s speed_pid = {
        .Kp=0.2f,
        .Ki=0.01f,
        .Kd=0.000f,
        .MaxOut=6.0f,
        .DeadBand = 0.0f,
        .Output_LPF_RC=0.1f,
        .Improve=PID_Integral_Limit | PID_OutputFilter,
        .IntegralLimit=1.0f,
};
    PID_Init_Config_s torque_pid = {
        .Kp=1000.0f,
        .Ki=5000.0f,
        .Kd=0.0f,
        .MaxOut=C620_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve=PID_Integral_Limit,
        .IntegralLimit=500.0f,
};
    C620_ConfigTypeDef config = {
        .motor_id=1,
        .motor_ptr_name="F_RMotor",
        .motor_mode=MOTOR_MODE_SPEED,
        .direction=1.0f,
        .torque_feed_forward = C620_Torque2Current(1.0f),//未测试
        .angle_pid_config=&angle_pid,
        .speed_pid_config=&speed_pid,
        .torque_pid_config=&torque_pid,
        .can_rx_topic_name="/CAN2/RX",
        .can_tx_topic_name="/CAN2/TX",
};
    C620_Register(&config);
    config.motor_id=2,
    config.motor_ptr_name="F_LMotor",
    C620_Register(&config);
    config.motor_id=3,
    config.motor_ptr_name="B_LMotor",
    C620_Register(&config);
    config.motor_id=4,
    config.motor_ptr_name="B_RMotor",
    C620_Register(&config);
}


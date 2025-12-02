//
// Created by Liszhi on 25-7-12.
//

#include "subarms.h"
#include <stdio.h>
#include <tgmath.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "c610.h"
#include "c620.h"
#include "tinybus_intf.h"


RC_ctrl_t *subarms_rc_ctrl;

INTF_Motor_HandleTypeDef *master;
INTF_Motor_HandleTypeDef *slave;
INTF_Motor_HandleTypeDef *test1;


void subarms_MainLoop()
{

    float x1,x2,y1,y2,torque,speed,angle;

    float realtorque,realspeed,realangle,targettorque,targetspeed,targetangle;
    vTaskDelay(10);
    while (1)
    {

        x1 = subarms_rc_ctrl[0].rc.rocker_l_;
        y1 = subarms_rc_ctrl[0].rc.rocker_l1;
        x2 = subarms_rc_ctrl[0].rc.rocker_r_;
        y2 = subarms_rc_ctrl[0].rc.rocker_r1;
        // printf("lx:%f\nly:%f\nrx:%f\nry:%f\n",x1,y1,x2,y2);


        vTaskDelay(100);
        realtorque = master->real_torque;
        realspeed = master->real_speed;
        realangle = master->real_angle;
        targettorque = master->target_torque;
        targetspeed = master->target_speed;
        targetangle = master->target_angle;

        if (subarms_rc_ctrl[0].rc.rocker_l_ >= 600)
        {
            torque +=5;
        }
        if (subarms_rc_ctrl[0].rc.rocker_l_ <= -600)
        {
            torque -=5;
        }
        master->set_speed(master,torque);
        printf("%f,%f,%f,%f,%f,%f\n",realtorque,realspeed,realangle,targettorque,targetspeed,targetangle);
        // master->target_angle=(slave->real_angle);
        // slave->target_angle = (master->real_angle);
        // test1->target_angle = 0;
        //
        // printf("%f\n", master->target_angle);




    }
}

void subarms_Init()
{


    vTaskDelay(10);
    // subarm1_MotorInit();
    // SubArm1_BaseMotor = pvSharePtr("SubArm1_BaseMotor", sizeof(INTF_Motor_HandleTypeDef));
    master = pvSharePtr("master", sizeof(INTF_Motor_HandleTypeDef));
    slave = pvSharePtr("slave", sizeof(INTF_Motor_HandleTypeDef));
    test1 = pvSharePtr("test1", sizeof(INTF_Motor_HandleTypeDef));

    // SubArm1_PitchMotor = pvSharePtr("SubArm1_PitchMotor", sizeof(INTF_Motor_HandleTypeDef));
    // SubArm1_ExtendMotor = pvSharePtr("SubArm1_ExtendMotor", sizeof(INTF_Motor_HandleTypeDef));


    // subarm2_MotorInit();
    // SubArm2_BaseMotor = pvSharePtr("SubArm2_BaseMotor", sizeof(INTF_Motor_HandleTypeDef));
    // SubArm2_BaseMotor->set_angle(SubArm2_BaseMotor,-2.3f); // 设置BaseMotor初始角度为0
    // SubArm2_WristMotor = pvSharePtr("SubArm2_WristMotor", sizeof(INTF_Motor_HandleTypeDef));
    // SubArm2_PitchMotor = pvSharePtr("SubArm2_PitchMotor", sizeof(INTF_Motor_HandleTypeDef));
    // SubArm2_ExtendMotor = pvSharePtr("SubArm2_ExtendMotor", sizeof(INTF_Motor_HandleTypeDef));

    printf("hello world\n");

    // LiftMotor_Init();
    // lift_motor = pvSharePtr("lift_motor", sizeof(INTF_Motor_HandleTypeDef));



    subarms_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(subarms_MainLoop, "subarms_MainLoop", 1024, NULL, 240, NULL);
}

/// subarm1电机配置
///
void subarm1_MotorInit()
{
    // SubArm1_BaseMotor_Init();
    // SubArm1_WristMotor_Init();
    // SubArm1_PitchMotor_Init();
    // SubArm1_ExtendMotor_Init();
}

void SubArm1_BaseMotor_Init() {
    // Odrive_Init();
    Odrive_CAN_ConfigTypedef config = {
        .motor_id =2,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
        .kp = 0.50f,
        .kd = 0.10f,
        .motor_name = "SubArm1_BaseMotor"};
    Odrive_Register(&config);
}

void SubArm1_WristMotor_Init()
{
    PID_Init_Config_s angle_pid = {
        .Kp = 80.0f,
        .Ki = 6.40f,
        .Kd = 8.00f,
        .CoefA = 0.5f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 1.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate | PID_OutputFilter,
    };
    PID_Init_Config_s speed_pid = {
        .Kp = 0.01f,
        .Ki = 0.0f,
        .Kd = 0.00f,
        .MaxOut = 12.0f,
        .DeadBand = 0.0f,
        .IntegralLimit = 6.0f,
        .Improve = PID_Integral_Limit| PID_OutputFilter,
    };
    PID_Init_Config_s torque_pid = {
        .Kp = 1000.0f,
        .Ki = 5000.0f,
        .Kd = 0.0f,
        .MaxOut = C610_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 100.0f,
    };
    C610_ConfigTypeDef config = {
        .motor_id = 2,
        .motor_ptr_name = "SubArm1_WristMotor",
        .motor_mode = MOTOR_MODE_ANGLE,
        .direction = 1.0f,
        .torque_feed_forward = C610_Torque2Current(1.0f), // 未测试
        .angle_pid_config = &angle_pid,
        .speed_pid_config = &speed_pid,
        .torque_pid_config = &torque_pid,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    C610_Register(&config);
}

void SubArm1_PitchMotor_Init()
{
    PID_Init_Config_s angle_pid = {
        .Kp = 60.0f,
        .Ki = 20.0f,
        .Kd = 1.50f,
        .CoefA = 0.5f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate// | PID_OutputFilter,
    };
    PID_Init_Config_s speed_pid = {
        .Kp = 0.05f,
        .Ki = 0.0f,
        .Kd = 0.00f,
        .MaxOut = 12.0f,
        .DeadBand = 0.0f,
        .IntegralLimit = 6.0f,
        .Improve = PID_Integral_Limit| PID_OutputFilter,
    };
    PID_Init_Config_s torque_pid = {
        .Kp = 1000.0f,
        .Ki = 5000.0f,
        .Kd = 0.0f,
        .MaxOut = C610_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 100.0f,
    };
    C610_ConfigTypeDef config = {
        .motor_id = 2,
        .motor_ptr_name = "SubArm1_PitchMotor",
        .motor_mode = MOTOR_MODE_ANGLE,
        .direction = 1.0f,
        .torque_feed_forward = C610_Torque2Current(1.0f), // 未测试
        .angle_pid_config = &angle_pid,
        .speed_pid_config = &speed_pid,
        .torque_pid_config = &torque_pid,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    C610_Register(&config);
}

void SubArm1_ExtendMotor_Init()
{
    PID_Init_Config_s angle_pid = {
        .Kp = 60.0f,
        .Ki = 20.0f,
        .Kd = 1.50f,
        .CoefA = 0.5f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate | PID_OutputFilter,
    };
    PID_Init_Config_s speed_pid = {
        .Kp = 0.01f,
        .Ki = 0.0f,
        .Kd = 0.00f,
        .MaxOut = 12.0f,
        .DeadBand = 0.0f,
        .IntegralLimit = 6.0f,
        .Improve = PID_Integral_Limit| PID_OutputFilter,
    };
    PID_Init_Config_s torque_pid = {
        .Kp = 1000.0f,
        .Ki = 5000.0f,
        .Kd = 0.0f,
        .MaxOut = C610_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 100.0f,
    };
    C610_ConfigTypeDef config = {
        .motor_id = 3,
        .motor_ptr_name = "SubArm1_ExtendMotor",
        .motor_mode = MOTOR_MODE_ANGLE,
        .direction = -1.0f,
        .torque_feed_forward = C610_Torque2Current(1.0f), // 未测试
        .angle_pid_config = &angle_pid,
        .speed_pid_config = &speed_pid,
        .torque_pid_config = &torque_pid,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    C610_Register(&config);

}

/// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// /// ///
/// subarm2电机配置
///
void subarm2_MotorInit()
{
    // SubArm2_BaseMotor_Init();
    // SubArm2_WristMotor_Init();
    // SubArm2_PitchMotor_Init();
    // SubArm2_ExtendMotor_Init();
}

void SubArm2_BaseMotor_Init() {
    // Odrive_Init();
    Odrive_CAN_ConfigTypedef config = {
        .motor_id =3,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
        .kp = 0.50f,
        .kd = 0.10f,
        .motor_name = "SubArm2_BaseMotor"};
    Odrive_Register(&config);
}

void SubArm2_WristMotor_Init()
{
    PID_Init_Config_s angle_pid = {
        .Kp = 80.0f,
        .Ki = 20.0f,
        .Kd = 1.50f,
        .CoefA = 0.5f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate | PID_OutputFilter,
    };
    PID_Init_Config_s speed_pid = {
        .Kp = 0.01f,
        .Ki = 0.0f,
        .Kd = 0.00f,
        .MaxOut = 12.0f,
        .DeadBand = 0.0f,
        .IntegralLimit = 6.0f,
        .Improve = PID_Integral_Limit| PID_OutputFilter,
    };
    PID_Init_Config_s torque_pid = {
        .Kp = 1000.0f,
        .Ki = 5000.0f,
        .Kd = 0.0f,
        .MaxOut = C610_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 100.0f,
    };
    C610_ConfigTypeDef config = {
        .motor_id = 4,
        .motor_ptr_name = "SubArm2_WristMotor",
        .motor_mode = MOTOR_MODE_ANGLE,
        .direction = -1.0f,
        .torque_feed_forward = C610_Torque2Current(1.0f), // 未测试
        .angle_pid_config = &angle_pid,
        .speed_pid_config = &speed_pid,
        .torque_pid_config = &torque_pid,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    C610_Register(&config);
}

void SubArm2_PitchMotor_Init()
{
    PID_Init_Config_s angle_pid = {
        .Kp = 80.0f,
        .Ki = 20.0f,
        .Kd = 1.50f,
        .CoefA = 0.8f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate// | PID_OutputFilter,
    };
    PID_Init_Config_s speed_pid = {
        .Kp = 0.05f,
        .Ki = 0.0f,
        .Kd = 0.00f,
        .MaxOut = 12.0f,
        .DeadBand = 0.0f,
        .IntegralLimit = 6.0f,
        .Improve = PID_Integral_Limit| PID_OutputFilter,
    };
    PID_Init_Config_s torque_pid = {
        .Kp = 1000.0f,
        .Ki = 5000.0f,
        .Kd = 0.0f,
        .MaxOut = C610_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 100.0f,
    };
    C610_ConfigTypeDef config = {
        .motor_id = 5,
        .motor_ptr_name = "SubArm2_PitchMotor",
        .motor_mode = MOTOR_MODE_ANGLE,
        .direction = 1.0f,
        .torque_feed_forward = C610_Torque2Current(1.0f), // 未测试
        .angle_pid_config = &angle_pid,
        .speed_pid_config = &speed_pid,
        .torque_pid_config = &torque_pid,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    C610_Register(&config);

}

void SubArm2_ExtendMotor_Init()
{
    PID_Init_Config_s angle_pid = {
        .Kp = 60.0f,
        .Ki = 20.0f,
        .Kd = 1.50f,
        .CoefA = 0.5f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate | PID_OutputFilter,
    };
    PID_Init_Config_s speed_pid = {
        .Kp = 0.01f,
        .Ki = 0.0f,
        .Kd = 0.00f,
        .MaxOut = 12.0f,
        .DeadBand = 0.0f,
        .IntegralLimit = 6.0f,
        .Improve = PID_Integral_Limit| PID_OutputFilter,
    };
    PID_Init_Config_s torque_pid = {
        .Kp = 1000.0f,
        .Ki = 5000.0f,
        .Kd = 0.0f,
        .MaxOut = C610_CURRENT_MAX,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 100.0f,
    };
    C610_ConfigTypeDef config = {
        .motor_id = 6,
        .motor_ptr_name = "SubArm2_ExtendMotor",
        .motor_mode = MOTOR_MODE_ANGLE,
        .direction = -1.0f,
        .torque_feed_forward = C610_Torque2Current(1.0f), // 未测试
        .angle_pid_config = &angle_pid,
        .speed_pid_config = &speed_pid,
        .torque_pid_config = &torque_pid,
        .can_rx_topic_name = "/CAN1/RX",
        .can_tx_topic_name = "/CAN1/TX",
    };
    C610_Register(&config);

}

////////////////////////////////////////////////////////////////////////////////////////////////
///lift抬升电机配置
void LiftMotor_Init() {
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
            .motor_id=8,
            .motor_ptr_name="lift_motor",
            .motor_mode=MOTOR_MODE_ANGLE,
            .direction=1.0f,
            .torque_feed_forward = C620_Torque2Current(1.0f),//未测试
            .angle_pid_config=&angle_pid,
            .speed_pid_config=&speed_pid,
            .torque_pid_config=&torque_pid,
            .can_rx_topic_name="/CAN2/RX",
            .can_tx_topic_name="/CAN2/TX",
    };
    // C620_Register(&config);
}



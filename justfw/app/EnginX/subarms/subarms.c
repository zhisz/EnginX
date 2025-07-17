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

float angle_min,angle_max;

RC_ctrl_t *subarms_rc_ctrl;

INTF_Motor_HandleTypeDef *SubArm1_BaseMotor;
INTF_Motor_HandleTypeDef *SubArm1_WristMotor;
INTF_Motor_HandleTypeDef *SubArm1_PitchMotor;
INTF_Motor_HandleTypeDef *SubArm1_ExtendMotor;

INTF_Motor_HandleTypeDef *SubArm2_BaseMotor;
INTF_Motor_HandleTypeDef *SubArm2_WristMotor;
INTF_Motor_HandleTypeDef *SubArm2_PitchMotor;
INTF_Motor_HandleTypeDef *SubArm2_ExtendMotor;

INTF_Motor_HandleTypeDef *lift_motor;



extern  int g_dr16_is_connected;

void Min_Calibrate(INTF_Motor_HandleTypeDef* motor) {
    const float CALIBRATE_SPEED = 5.0f;         // 每次移动多少角度
    const int CALIBRATE_DELAY = 30;             // 每次延时多少ms
    const int CALIBRATE_TIMEOUT = 10000;        // 最长校准时间
    const float ANGLE_THRESHOLD = 1.0f;         // 判断停止的最小变化量
    const int STABLE_COUNT_THRESHOLD = 10;      // 连续多次不变就认为撞到限位

    float last_angle = 0.0f;                    // 上一次的角度值
    int stable_count = 0;                       //平稳的次数
    int elapsed = 0;                            // 总的延时时长


    // 1. 向负方向移动，找最小角度（左限位）
    while (elapsed < CALIBRATE_TIMEOUT) {          // 超过设定时间停止自校准
        motor->target_angle -= CALIBRATE_SPEED;     // 向负方向移动
        vTaskDelay(CALIBRATE_DELAY);                // 等待一段时间

        float curr_angle = motor->real_angle;       // 更新当前角度

        if (fabs(curr_angle - last_angle) < ANGLE_THRESHOLD) {  // 判断角度变化值是否小于阈值，注意是变化值，两次真实角度的差值
            stable_count++;                                     // 如果小于阈值，连续稳定的次数加1
        } else {
            stable_count = 0;                                   // 如果大于阈值，重置稳定计数
        }

        if (stable_count >= STABLE_COUNT_THRESHOLD) {           // 如果连续稳定次数超过阈值，认为撞到限位
            angle_min = motor->real_angle;                      // 记录左限位角度
            motor->target_angle = (motor->real_angle + 20.0f);                // 设定目标角度为中间值或指定值
            break;                                              // 撞到限位，退出循环
        }

        last_angle = curr_angle;                                // 更新上一次的角度值
        elapsed += CALIBRATE_DELAY;                             // 累加延时时长
    }



}

void Max_Calibrate(INTF_Motor_HandleTypeDef* motor) {
    const float CALIBRATE_SPEED = 5.0f;     // 校准用的慢速（单位看你项目定的）
    const int CALIBRATE_DELAY = 30;         // 每次运动后的延时(ms)
    const int CALIBRATE_TIMEOUT = 10000;     // 最大校准时间，防死循环(ms)
    const float ANGLE_THRESHOLD = 1.0f;     // 判断是否停止变化的角度阈值
    const int STABLE_COUNT_THRESHOLD = 10;  // 连续几次角度没变，认为撞限位

    float last_angle = 0.0f;//上一次的角度值
    int stable_count = 0;   // 连续稳定的次数
    int elapsed = 0;        //总的延时时长

    // 2. 向正方向移动（找右限位）

    while (elapsed < CALIBRATE_TIMEOUT) {                       // 超过设定时间停止自校准
        motor->target_angle += CALIBRATE_SPEED;                 // 向正方向移动
        vTaskDelay(CALIBRATE_DELAY);                            // 等待一段时间

        float curr_angle = motor->real_angle;                   // 更新当前角度

        if (fabs(curr_angle - last_angle) < ANGLE_THRESHOLD) {  // 判断角度变化值是否小于阈值，注意是变化值，两次真实角度的差值
            stable_count++;                                     // 如果小于阈值，连续稳定的次数加1
        } else {
            stable_count = 0;                                   // 如果大于阈值，重置稳定计数
        }

        if (stable_count >= STABLE_COUNT_THRESHOLD) {           // 如果连续稳定次数超过阈值，认为撞到限位
            angle_max = motor->real_angle;                      // 记录右限位角度
            motor->target_angle = (motor->real_angle - 20.0f);             // 设定目标角度为中间值或指定值
            break;                                              // 撞到限位，退出循环
        }

        last_angle = curr_angle;                                // 更新上一次的角度值
        elapsed += CALIBRATE_DELAY;                             // 累加延时时长
    }

    // 3. 归中或归位




}

bool IsMotorCalibrated = 0;


void CalibrateMotor(INTF_Motor_HandleTypeDef* motor, float desired_angle, int calibrate_count)
{
    bool is_calibrated = false; // 默认是未校准



        for (int a=0;a<=calibrate_count;a++)
        {
            Max_Calibrate(motor);
            Min_Calibrate(motor);


            motor->target_angle= (desired_angle*(angle_max - angle_min)/2.0f + (angle_max + angle_min)/2.0f);        // 设置目标角度为中间值或指定值
        }
        angle_min = 0.0f;
        angle_max = 0.0f;        // 重置角度范围

        IsMotorCalibrated =1;


}


void subarms_MainLoop()
{
    CalibrateMotor(SubArm1_WristMotor,0.0f,4);
    CalibrateMotor(SubArm1_PitchMotor,1.01f,0);
    CalibrateMotor(SubArm1_ExtendMotor,0.0f,0);


    while (1)
    {

        float anglex, angley, anglez, anglew;
        //
        // SubArm1_WristMotor->target_angle=subarms_rc_ctrl[0].rc.rocker_r_/660.0f*150.0f;
        //
        // printf("real:%f,%f,%f,%f,%f\n",SubArm1_WristMotor->real_angle,SubArm1_WristMotor->target_angle,SubArm1_WristMotor->real_speed,angle_max,angle_min);




        while (subarms_rc_ctrl[0].rc.switch_left==2)
        {
            // printf("USB_OK");
            anglex = subarms_rc_ctrl[0].rc.rocker_l_/660.0f*1.5f;
            angley = subarms_rc_ctrl[0].rc.rocker_r_/660.0f*105.0f;
            anglez = subarms_rc_ctrl[0].rc.rocker_l1/660.0f*60.0f;
            anglew = subarms_rc_ctrl[0].rc.rocker_r1/660.0f*180.0f;




            SubArm1_BaseMotor->set_angle(SubArm1_BaseMotor, anglex);

            SubArm1_WristMotor->set_angle(SubArm1_WristMotor,angley);

            if (anglez<0) anglez=0;
            SubArm1_PitchMotor->set_angle(SubArm1_PitchMotor,anglez);

            if (anglew<0) anglew=0;
            SubArm1_ExtendMotor->set_angle(SubArm1_ExtendMotor,anglew);




            SubArm2_BaseMotor->set_angle(SubArm2_BaseMotor, anglex-2.3f);

            SubArm2_WristMotor->set_angle(SubArm2_WristMotor,angley);

            if (anglez<0) anglez=0;
            SubArm2_PitchMotor->set_angle(SubArm2_PitchMotor,anglez);

            if (anglew<0) anglew=0;
            SubArm2_ExtendMotor->set_angle(SubArm2_ExtendMotor,anglew);


            int lift_h = 0;
            if (subarms_rc_ctrl->rc.switch_right == 2) lift_h = 1;
            else if (subarms_rc_ctrl->rc.switch_right == 3) lift_h = 5;
            else if (subarms_rc_ctrl->rc.switch_right == 1) lift_h = 9;;

            lift_motor->set_angle(lift_motor,lift_h);

            extern float INS_angle_N[0];





            // printf("angleXYZ:%f,%f,%f\n",INS_angle_N[0],INS_angle_N[1],INS_angle_N[2]);
            printf("anglex:%f\n",anglex);
            printf("angley:%f\n",angley);
            printf("anglez:%f\n",anglez);
            printf("anglew:%f\n",anglew);


            vTaskDelay(10);

        }
        vTaskDelay(10);
    }
}

void subarms_Init()
{

    subarm1_MotorInit();
    SubArm1_BaseMotor = pvSharePtr("SubArm1_BaseMotor", sizeof(INTF_Motor_HandleTypeDef));
    SubArm1_WristMotor = pvSharePtr("SubArm1_WristMotor", sizeof(INTF_Motor_HandleTypeDef));
    SubArm1_PitchMotor = pvSharePtr("SubArm1_PitchMotor", sizeof(INTF_Motor_HandleTypeDef));
    SubArm1_ExtendMotor = pvSharePtr("SubArm1_ExtendMotor", sizeof(INTF_Motor_HandleTypeDef));


    subarm2_MotorInit();
    SubArm2_BaseMotor = pvSharePtr("SubArm2_BaseMotor", sizeof(INTF_Motor_HandleTypeDef));
    SubArm2_WristMotor = pvSharePtr("SubArm2_WristMotor", sizeof(INTF_Motor_HandleTypeDef));
    SubArm2_PitchMotor = pvSharePtr("SubArm2_PitchMotor", sizeof(INTF_Motor_HandleTypeDef));
    SubArm2_ExtendMotor = pvSharePtr("SubArm2_ExtendMotor", sizeof(INTF_Motor_HandleTypeDef));

    LiftMotor_Init();
    lift_motor = pvSharePtr("lift_motor", sizeof(INTF_Motor_HandleTypeDef));




    subarms_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(subarms_MainLoop, "subarms_MainLoop", 1024, NULL, 240, NULL);
}

/// subarm1电机配置
///
void subarm1_MotorInit()
{
    SubArm1_BaseMotor_Init();
    SubArm1_WristMotor_Init();
    SubArm1_PitchMotor_Init();
    SubArm1_ExtendMotor_Init();
}

void SubArm1_BaseMotor_Init() {
    Odrive_Init();
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
        .motor_id = 1,
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
        .motor_ptr_name = "SubArm1_PitchMotor",
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
    SubArm2_BaseMotor_Init();
    SubArm2_WristMotor_Init();
    SubArm2_PitchMotor_Init();
    SubArm2_ExtendMotor_Init();
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
        .motor_id = 4,
        .motor_ptr_name = "SubArm2_WristMotor",
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

void SubArm2_PitchMotor_Init()
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
        .motor_id = 5,
        .motor_ptr_name = "SubArm2_PitchMotor",
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
    C620_Register(&config);
}



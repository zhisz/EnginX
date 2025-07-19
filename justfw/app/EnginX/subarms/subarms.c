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


///遥控调试小臂角度，该函数主要用于调试，通过遥控是小臂处于特定状态，在打印出关键信息（angle_ratio），缩短调试时间
#define DEADZONE 50         // 摇杆死区
#define DELTA_ANGLE 1.0f    // 每次调整的角度
void SubArm_RC_Control(void)
{
    // 读取摇杆值
    int pitch_input  = subarms_rc_ctrl[0].rc.rocker_l1;  // 左Y
    int wrist_input  = subarms_rc_ctrl[0].rc.rocker_r_;   // 右X
    int extend_input = subarms_rc_ctrl[0].rc.rocker_r1;  // 右Y
    float ratio_pitch,ratio_wrist,ratio_extend;

    // 控制 PitchMotor
    //DEADZONE：死区限制＋最大值限制
    //DELTA_ANGLE：角度++或--，Δ_angle决定速度
    if (pitch_input > DEADZONE && SubArm1_PitchMotor->target_angle < SubArm1_PitchMotor->max_angle) {
        SubArm1_PitchMotor->target_angle += DELTA_ANGLE;
    } else if (pitch_input < -DEADZONE && SubArm1_PitchMotor->target_angle > SubArm1_PitchMotor->min_angle) {
        SubArm1_PitchMotor->target_angle -= DELTA_ANGLE;
    }
    vTaskDelay(10);

    // 控制 WristMotor
    if (wrist_input > DEADZONE && SubArm1_WristMotor->target_angle < SubArm1_WristMotor->max_angle) {
        SubArm1_WristMotor->target_angle += DELTA_ANGLE;
    } else if (wrist_input < -DEADZONE && SubArm1_WristMotor->target_angle > SubArm1_WristMotor->min_angle) {
        SubArm1_WristMotor->target_angle -= DELTA_ANGLE;
    }
    vTaskDelay(10);

    // 控制 ExtendMotor
    if (extend_input > DEADZONE && SubArm1_ExtendMotor->target_angle < (SubArm1_ExtendMotor->max_angle + 75.0f)) {
        SubArm1_ExtendMotor->target_angle += DELTA_ANGLE;
    } else if (extend_input < -DEADZONE && SubArm1_ExtendMotor->target_angle > SubArm1_ExtendMotor->min_angle) {
        SubArm1_ExtendMotor->target_angle -= DELTA_ANGLE;
    }
    vTaskDelay(10);

    // 调试助手，打印对应的角度ratio值，通过解算公式算出对应的angle_ratio值
    ratio_pitch = (SubArm1_PitchMotor->target_angle - (SubArm1_PitchMotor->max_angle+SubArm1_PitchMotor->min_angle)/2)*2/(SubArm1_PitchMotor->max_angle - SubArm1_PitchMotor->min_angle);
    printf("ratio_pitch:%f",ratio_pitch);
    ratio_extend = (SubArm1_ExtendMotor->target_angle - (SubArm1_ExtendMotor->max_angle+SubArm1_ExtendMotor->min_angle)/2)*2/(SubArm1_ExtendMotor->max_angle - SubArm1_ExtendMotor->min_angle);
    printf("ratio_extend:%f",ratio_extend);
    ratio_wrist = (SubArm1_WristMotor->target_angle - (SubArm1_WristMotor->max_angle+SubArm1_WristMotor->min_angle)/2)*2/(SubArm1_WristMotor->max_angle - SubArm1_WristMotor->min_angle);
    printf("ratio_wrist:%f",ratio_wrist);
}


///驱动电机函数，传入电机名，目标角度比例，和移动速度
void Move(INTF_Motor_HandleTypeDef * motor, float position_ratio, float move_speed) {
    const float ANGLE_THRESHOLD = 5.0f;   // 判断到达目标角度的容差
    const int MOVE_DELAY = 30;            // 延时周期 (ms)

    // 安全范围限制：-1.0 ~ 1.0
    if (position_ratio > 1.2f) position_ratio = 1.2f;
    if (position_ratio < -1.2f) position_ratio = -1.2f;

    // 根据比例计算目标角度
    float target_angle = motor->min_angle + (position_ratio + 1.0f) / 2.0f * (motor->max_angle - motor->min_angle); // 通过逆解算得出目标角度值

    while (fabs(motor->real_angle - target_angle) > ANGLE_THRESHOLD) {// 判断是否到达目标角度,因重力，摩擦等外界因素存在，目标角度不可完美到达
        if (motor->real_angle < target_angle) {                      // 如果当前角度小于目标角度
            motor->target_angle += move_speed;                      // 向目标角度移动
            if (motor->target_angle > target_angle) {                 // 如果超过目标角度，则设定为目标角度
                motor->target_angle = target_angle;                   // 确保目标角度不超过设定值
            }
        } else {
            motor->target_angle -= move_speed;                      // 否则反方向向目标角度移动
            if (motor->target_angle < target_angle) {               // 如果超过目标角度，则设定为目标角度
                motor->target_angle = target_angle;                 // 确保目标角度不超过设定值
            }
        }

        vTaskDelay(MOVE_DELAY);                                     // 等待一段时间，再判断是否到位
    }


    motor->target_angle = target_angle; // 最后确保位置精确到达
}


///最小角度校准函数，通过向负方向移动，寻找最小角度（左限位），并记录下来
void Min_Calibrate(INTF_Motor_HandleTypeDef* motor, float CALIBRATE_SPEED) {//最小角度自校准
                                                                            // float CALIBRATE_SPEED 表示每次移动多少角度
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
            motor->min_angle = motor->real_angle;                      // 记录左限位角度
            motor->target_angle = (motor->real_angle + 20.0f);                // 设定目标角度为中间值或指定值
            break;                                              // 撞到限位，退出循环
        }

        last_angle = curr_angle;                                // 更新上一次的角度值
        elapsed += CALIBRATE_DELAY;                             // 累加延时时长
    }



}

///最大角度校准函数，通过向正方向移动，寻找最大角度（右限位），并记录下来
void Max_Calibrate(INTF_Motor_HandleTypeDef* motor,float CALIBRATE_SPEED) {//最大角度自校准
                                                                           // CALIBRATE_SPEED 校准用的速度
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
            motor->max_angle = motor->real_angle;                      // 记录右限位角度
            motor->target_angle = (motor->real_angle - 20.0f);         // 消除堵转效应，往回拨一定的角度值（实测发现，如果不加这个，会导致校准失败）
            break;                                              // 撞到限位，退出循环
        }

        last_angle = curr_angle;                                // 更新上一次的角度值
        elapsed += CALIBRATE_DELAY;                             // 累加延时时长
    }

    // 3. 归中或归位




}

///电机校准函数，用户端调用
//*电机名称，
//*目标角度（映射到-1~1），
//*校准次数(有的电机因结构问题响应过慢，容易被误判为已经校准到极限角度，所以需要多次校准),
//*校准速度（每次校准移动多少角度）
void CalibrateMotor(INTF_Motor_HandleTypeDef* motor, float desired_angle, int calibrate_count,float CALIBRATE_SPEED)
{



        for (int a=0;a<=calibrate_count;a++)      // 循环校准次数
        {
            Max_Calibrate(motor,CALIBRATE_SPEED); // 最大角度校准
            Min_Calibrate(motor,CALIBRATE_SPEED); // 最小角度校准

        }

    motor->target_angle= (desired_angle*(motor->max_angle - motor->min_angle)/2.0f + (motor->max_angle + motor->min_angle)/2.0f);  // 设置目标角度为指定值


}




void subarms_MainLoop()
{
    // CalibrateMotor(SubArm2_WristMotor,-1.0f,3,6.0f);
    // Move(SubArm2_WristMotor,1.00,6.0f);
    // CalibrateMotor(SubArm2_PitchMotor,-1.00f,0,2.0f);
    // CalibrateMotor(SubArm2_ExtendMotor,0.0f,0,6.0f);
    // Move(SubArm2_ExtendMotor,0.6454f,6.0f);
    // Move(SubArm2_PitchMotor,-0.501f,2.0f);


    while (1)
    {

        while (subarms_rc_ctrl[0].rc.switch_left == 3)
        {
            if (subarms_rc_ctrl[0].rc.rocker_l1 <= -600 && subarms_rc_ctrl[0].rc.rocker_l_ <= -600 &&
                subarms_rc_ctrl[0].rc.rocker_r1 <= -600 && subarms_rc_ctrl[0].rc.rocker_r_ >= 600)
            {

                Move(SubArm2_ExtendMotor,1.1500f,2.0f); // 向下伸展,吸取矿石
                vTaskDelay(40);
                Move(SubArm2_ExtendMotor,0.500f,4.0f);// 向上收回，升起矿石
                Move(SubArm2_PitchMotor,-0.188f,0.5f); // 向上微微抬起
                Move(SubArm2_ExtendMotor,-0.114f,4.0f); // 向后收回
                Move(SubArm2_WristMotor,0.128f,2.0f); // 转动
                Move(SubArm2_PitchMotor,-0.558f,4.0f);
                Move(SubArm2_WristMotor,1.008f,2.0f);

                vTaskDelay(10);
            }
            vTaskDelay(10);
        }

        while (subarms_rc_ctrl[0].rc.switch_left==2)
        {
            SubArm_RC_Control();

            if (subarms_rc_ctrl[0].rc.rocker_l1 >=  50)SubArm2_PitchMotor->target_angle += 2.0f;
            if (subarms_rc_ctrl[0].rc.rocker_l1 <= -50)SubArm2_PitchMotor->target_angle -= 2.0f;

            printf("targetangle:%f\n",SubArm2_PitchMotor->target_angle);
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
    // SubArm1_BaseMotor_Init();
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
        .Kp = 80.0f,
        .Ki = 20.0f,
        .Kd = 1.50f,
        .CoefA = 0.8f,
        .CoefB = 0.5f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 5000.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit | PID_ChangingIntegrationRate | PID_OutputFilter,
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
        .motor_id = 7,
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
        .motor_id = 8,
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



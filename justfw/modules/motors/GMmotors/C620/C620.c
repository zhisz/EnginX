#include "C620.h"

#include "GMmotors.h"
#include "motor_manager_config.h"
#include "user_lib.h"

extern GM_BufferTypeDef GM_Buffer[GM_BUFFER_NUM];

INTF_Motor_HandleTypeDef *g_c620_motors[GM_BUFFER_NUM][C620_MOTOR_NUM] = {0};

int16_t C620_Torque2Current(float torque) {
    if (torque > 20 * C620_TORQUE_CONST) {
        return C620_CURRENT_MAX;
    } else if (torque < -20 * C620_TORQUE_CONST) {
        return -C620_CURRENT_MAX;
    } else {
        return (int16_t)roundf(torque / C620_TORQUE_CONST / 20.0f * C620_CURRENT_MAX);
    }
}

float C620_Current2Torque(int16_t current) {
    return current * 20.0f / C620_CURRENT_MAX * C620_TORQUE_CONST;
}

void C620_SetCurrent(uint8_t id, int16_t current, BusTopicHandle_t can_tx_topic) {
    // 限幅
    if (current > C620_CURRENT_MAX) {
        current = C620_CURRENT_MAX;
    } else if (current < -C620_CURRENT_MAX) {
        current = -C620_CURRENT_MAX;
    }

    GM_BufferTypeDef *buffer = NULL;
    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == can_tx_topic) {
            buffer = &GM_Buffer[i];
        }
    }
    if (buffer == NULL) {
        return;
    }
    if (id > 4) {
        buffer->buffer_0x1FF[(id - 5) * 2] = current >> 8;
        buffer->buffer_0x1FF[(id - 5) * 2 + 1] = current;
    } else {
        buffer->buffer_0x200[(id - 1) * 2] = current >> 8;
        buffer->buffer_0x200[(id - 1) * 2 + 1] = current;
    }
}

void C620_CAN_CallBack(void *message, BusSubscriberHandle_t subscriber) {
    INTF_CAN_MessageTypeDef *msg = (INTF_CAN_MessageTypeDef *)message;
    if (msg->id_type == BSP_CAN_ID_EXT || msg->can_id < 0x201 || msg->can_id > 0x208) {
        return;
    }
    uint8_t id = msg->can_id - C620_ID_BASE - 1;

    GM_BufferTypeDef *buffer = NULL;
    uint16_t buffer_id = -1;
    for (int i = 0; i < GM_BUFFER_NUM; i++) {
        if (GM_Buffer[i].can_rx_topic == subscriber->pxTopic) {
            buffer = &GM_Buffer[i];
            buffer_id = i;
        }
    }
    if (buffer == NULL) {
        return;
    }
    if (g_c620_motors[buffer_id][id] == NULL) {
        return;
    }

    INTF_Motor_HandleTypeDef *m = g_c620_motors[buffer_id][id];
    // 确认电机在正常运行
    if (m->motor_state != MOTOR_STATE_RUNNING) {
        m->motor_state = MOTOR_STATE_RUNNING;
    }

    m->update_time = HAL_GetTick();

    C620_ResDataTypeDef *priv = m->private_data;

    int32_t ecd = (int16_t)((int16_t)(msg->data[0] << 8 | msg->data[1]) *
                            sign(m->direction));  // 0~360°
    m->real_speed = LowPassFilter(m->real_speed,
                                  (int16_t)(msg->data[2] << 8 | msg->data[3]) *
                                      m->direction * 2.0f * PI / 60 / priv->gear_ratio,
                                  0.9f);  // RPM->rad/s
    m->real_torque = LowPassFilter(m->real_torque,
                                   C620_Current2Torque(
                                       (int16_t)(msg->data[4] << 8 | msg->data[5])) *
                                       m->direction,
                                   0.9f);  // Nm

    if (ecd - priv->last_ecd > 4096)
        priv->total_rounds--;
    else if (ecd - priv->last_ecd < -4096)
        priv->total_rounds++;
    m->real_angle = LowPassFilter(m->real_angle,
                                  (priv->total_rounds * 2.0f * PI +
                                   ecd * 2.0f * PI / C620_ANGLE_MAX + m->angle_offset) /
                                      priv->gear_ratio,
                                  0.9f);
    priv->last_ecd = ecd;
}

void C620_PIDCalc() {
    for (int j = 0; j < GM_BUFFER_NUM; ++j) {
        for (int i = 0; i < C620_MOTOR_NUM; ++i) {
            INTF_Motor_HandleTypeDef *m = g_c620_motors[j][i];
            if (m == NULL) {
                continue;
            }

            if (m->motor_state != MOTOR_STATE_RUNNING) {
                continue;  // 确认电机在正常运行
            }

            // 电机离线检测
            if (HAL_GetTick() - m->update_time > 30) {
                m->motor_state = MOTOR_STATE_ERROR;
            }

            float PID_out = 0;  // 临时存放PID的输出
            C620_ResDataTypeDef *priv = m->private_data;

            switch (m->motor_mode) {
            case MOTOR_MODE_ANGLE:
                PID_out = m->target_angle;
                break;
            case MOTOR_MODE_SPEED:
                PID_out = m->target_speed;
                break;
            case MOTOR_MODE_TORQUE:
                PID_out = m->target_torque;
                break;
            }

            float angle_measure, speed_measure;
            // 注意：下面的switch没有break，PID逐级计算
            switch (m->motor_mode) {
            case MOTOR_MODE_ANGLE:
                if (priv->other_feedback_of_angle == NULL) {
                    angle_measure = m->real_angle;
                } else {
                    angle_measure = *(priv->other_feedback_of_angle);
                }
                PID_out = PIDCalculate(&(priv->angle_pid), angle_measure, PID_out);
                [[fallthrough]];
            case MOTOR_MODE_SPEED:
                if (priv->other_feedback_of_speed == NULL) {
                    speed_measure = m->real_speed;
                } else {
                    speed_measure = *(priv->other_feedback_of_speed);
                }
                PID_out = PIDCalculate(&(priv->speed_pid), speed_measure, PID_out);
                [[fallthrough]];
            case MOTOR_MODE_TORQUE:
                // 前馈
                PID_out = priv->torque_feed_forward * PID_out +
                          PIDCalculate(&(priv->torque_pid), m->real_torque, PID_out);
                // 限幅，防止爆uint16_t
                PID_out = float_constrain(PID_out, -C620_CURRENT_MAX, C620_CURRENT_MAX);
            }
            C620_SetCurrent(i + 1, (int16_t)roundf(PID_out * m->direction), priv->can_tx_topic);
        }
    }
}

void C620_Setmode_t(struct INTF_Motor_Handle *self, INTF_Motor_ModeTypeDef mode) {
    self->motor_mode = mode;
}

void C620_SetSpeed_t(struct INTF_Motor_Handle *self, float speed) {
    self->target_speed = speed;
}

void C620_SetAngle_t(struct INTF_Motor_Handle *self, float angle) {
    self->target_angle = angle;
}

void C620_SetTorque_t(struct INTF_Motor_Handle *self, float torque) {
    self->target_torque = torque;
}

void C620_Disable_t(struct INTF_Motor_Handle *self) {
    self->motor_state = MOTOR_STATE_DISABLE;
}

void C620_Enable_t(struct INTF_Motor_Handle *self) {
    self->motor_state = MOTOR_STATE_RUNNING;
}

#ifdef C610_USE_MANAGER
#include <stdio.h>

static void apply_control(ITNF_ManagerdMotor_HandleTypedef *manager) {
    INTF_Motor_HandleTypeDef *m = manager->motor;
    C620_ResDataTypeDef *priv = m->private_data;
    priv->angle_pid.Kp = manager->pid_parms.angle_kp;
    priv->angle_pid.Ki = manager->pid_parms.angle_ki;
    priv->angle_pid.Kd = manager->pid_parms.angle_kd;

    priv->speed_pid.Kp = manager->pid_parms.velocity_kp;
    priv->speed_pid.Ki = manager->pid_parms.velocity_ki;
    priv->speed_pid.Kd = manager->pid_parms.velocity_kd;
}

static void print_info(ITNF_ManagerdMotor_HandleTypedef *manager, char *buff, uint16_t len) {
    INTF_Motor_HandleTypeDef *m = manager->motor;
    C620_ResDataTypeDef *priv = m->private_data;
    // printf("ErrorCode:%d\n\r", priv->axis_error);
    // printf("Parms kp:%f,kd:%f\n\r", priv->kp, priv->kd);

    snprintf(buff, len, "Angle: kp=%f ki=%f kd=%f\n\rSpeed: kp=%f ki=%f kd=%f\n\r",
             priv->angle_pid.Kp,
             priv->angle_pid.Ki,
             priv->angle_pid.Kd,
             priv->speed_pid.Kp,
             priv->speed_pid.Ki,
             priv->speed_pid.Kd);
}

#endif

INTF_Motor_HandleTypeDef *C620_Register(C620_ConfigTypeDef *config) {
    INTF_Motor_HandleTypeDef *motor = pvSharePtr(config->motor_ptr_name, sizeof(INTF_Motor_HandleTypeDef));
    motor->motor_id = config->motor_id;
    motor->motor_mode = config->motor_mode;
    motor->motor_state = MOTOR_STATE_INIT;
    motor->target_speed = 0.0f;
    motor->real_speed = 0.0f;
    motor->target_angle = 0.0f;
    motor->real_angle = 0.0f;
    motor->target_torque = 0.0f;
    motor->real_torque = 0.0f;
    motor->direction = config->direction;

    motor->angle_offset = config->angle_offset;

    motor->private_data = JUST_MALLOC(sizeof(C620_ResDataTypeDef));
    memset(motor->private_data, 0, sizeof(C620_ResDataTypeDef));
    C620_ResDataTypeDef *priv = motor->private_data;

    if (config->gear_ratio != 0) {
        priv->gear_ratio = config->gear_ratio;
    } else {
        priv->gear_ratio = 19.203f;
    }

    priv->can_rx_topic = xBusSubscribeFromName(config->can_rx_topic_name, C620_CAN_CallBack);
    priv->can_tx_topic = xBusTopicRegister(config->can_tx_topic_name);

    priv->other_feedback_of_angle = config->other_feedback_of_angle;
    priv->other_feedback_of_speed = config->other_feedback_of_speed;

    priv->torque_feed_forward = config->torque_feed_forward;

    PIDInit(&priv->angle_pid, config->angle_pid_config);
    PIDInit(&priv->speed_pid, config->speed_pid_config);
    PIDInit(&priv->torque_pid, config->torque_pid_config);

    motor->set_mode = C620_Setmode_t;
    motor->set_speed = C620_SetSpeed_t;
    motor->set_angle = C620_SetAngle_t;
    motor->set_torque = C620_SetTorque_t;

    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == priv->can_tx_topic) {
            g_c620_motors[i][motor->motor_id - 1] = motor;
        }
    }

    for (int i = 0; i < GM_BUFFER_NUM; ++i) {
        if (GM_Buffer[i].can_tx_topic == priv->can_tx_topic) {
            if (motor->motor_id > 4) {
                GM_Buffer[i].buffer_0x1FF_not_null = 1;
            } else {
                GM_Buffer[i].buffer_0x200_not_null = 1;
            }
            break;
        }
    }

#ifdef C620_USE_MANAGER
#include "motor_manager.h"
    ITNF_ManagerdMotor_HandleTypedef *managed_motor = ManagedMotor_Create(motor);
    managed_motor->motor_name = config->motor_ptr_name;
    managed_motor->pid_parms.angle_kp = config->angle_pid_config->Kp;
    managed_motor->pid_parms.angle_ki = config->angle_pid_config->Ki;
    managed_motor->pid_parms.angle_kd = config->angle_pid_config->Kd;
    managed_motor->pid_parms.velocity_kp = config->speed_pid_config->Kp;
    managed_motor->pid_parms.velocity_ki = config->speed_pid_config->Ki;
    managed_motor->pid_parms.velocity_kd = config->speed_pid_config->Kd;
    managed_motor->ApplyControl = apply_control;
    managed_motor->MotorInfo = print_info;
#endif

    return motor;
}

void C620_Init()
{


    PID_Init_Config_s angle_pid = {
        .Kp = 20.0f,       // 提高比例增益，加快响应速度
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
        .IntegralLimit= 500.0f,
};
    C620_ConfigTypeDef config = {
        .motor_id=5,
        .motor_ptr_name="test1",
        .motor_mode=MOTOR_MODE_ANGLE,
        .direction=1.0f,
        .torque_feed_forward = C620_Torque2Current(1.0f),//未测试
        .angle_pid_config=&angle_pid,
        .speed_pid_config=&speed_pid,
        .torque_pid_config=&torque_pid,
        .can_rx_topic_name="/CAN1/RX",
        .can_tx_topic_name="/CAN1/TX",
};
    C620_Register(&config);


//     PID_Init_Config_s angle_pid2 = {
//         .Kp = 6.0f,       // 提高比例增益，加快响应速度
//         .Ki = 3.0f,       // 保持积分增益，维持稳态精度
//         .Kd = 0.2f,       // 增大微分增益，抑制抖动
//         .MaxOut = 180.0f,
//         .DeadBand = 0.11f, // 缩小死区，减少小误差下的抖动
//         .Improve = PID_Integral_Limit | PID_OutputFilter,
//         .Output_LPF_RC = 0.1f, // 启用低通滤波
// };
//     PID_Init_Config_s speed_pid2 = {
//         .Kp=0.2f,
//         .Ki=0.01f,
//         .Kd=0.000f,
//         .MaxOut=6.0f,
//         .DeadBand = 0.0f,
//         .Output_LPF_RC=0.1f,
//         .Improve=PID_Integral_Limit | PID_OutputFilter,
//         .IntegralLimit=1.0f,
// };
//     PID_Init_Config_s torque_pid2 = {
//         .Kp=1000.0f,
//         .Ki=5000.0f,
//         .Kd=0.0f,
//         .MaxOut=C620_CURRENT_MAX,
//         .DeadBand = 0.0f,
//         .Improve=PID_Integral_Limit,
//         .IntegralLimit=500.0f,
// };
//     C620_ConfigTypeDef config2 = {
//         .motor_id=33,
//         .motor_ptr_name="_motor",
//         .motor_mode=MOTOR_MODE_ANGLE,
//         .direction=-1.0f,
//         .torque_feed_forward = C620_Torque2Current(1.0f),//未测试
//         .angle_pid_config=&angle_pid2,
//         .speed_pid_config=&speed_pid2,
//         .torque_pid_config=&torque_pid2,
//         .can_rx_topic_name="/CAN1/RX",
//         .can_tx_topic_name="/CAN1/TX",
// };
//     //C620_Register(&config2);
//     config2.motor_id = 2;
//     config2.motor_ptr_name = "lift_motor";
//     config2.direction = 1.0f;
//     //C620_Register(&config2);

}

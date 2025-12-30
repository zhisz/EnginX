//
// Created by CODE01 on 2025/12/30.
//

#include "data_recieve.h"
#include "cmsis_os.h"
#include "crc8_crc16.h"
#include "tinybus.h"

Bus_TopicHandleTypeDef *g_arm_data_rx;
QueueHandle_t arm_data_msg_queue;
float arm_angle[6];


void arm_data_solve(void *message, Bus_TopicHandleTypeDef *topic)
{
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *) message;
    if (msg->len != sizeof(arm_data_M2S_PacketTypeDef)) {
        return;//长度不对丢包
    }
    union arm_data_M2S_Union arm_data_m2s_union;
    memcpy(arm_data_m2s_union.bit_flow, msg->data, sizeof(arm_data_M2S_PacketTypeDef));
    if (arm_data_m2s_union.arm_data_m2s.header == 0xAA) {
        if (verify_CRC16_check_sum(arm_data_m2s_union.bit_flow, sizeof(arm_data_M2S_PacketTypeDef))) {

            memcpy(arm_angle,arm_data_m2s_union.arm_data_m2s.arm_angle,sizeof(arm_angle));


            // xQueueSendFromISR(arm_data_msg_queue,&arm_data_m2s_union.arm_data_m2s,0);
        }
    }

}

void data_receive_loop(){
    while(1){
        //TODO 你的控制代码

        osDelay(1);
    }
}



void data_receive_init(){

    Bus_SubscribeFromName("USB_RX", arm_data_solve);
    // Bus_SubscribeFromName("USB_RX", arm_data_solve);

    arm_data_msg_queue= xQueueCreate(5,sizeof(AutoAim_M2S_PacketTypeDef));

    osThreadDef(data_receive_loopTask, data_receive_loop, osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(data_receive_loopTask), NULL);
}
//启用用户模块，加上这一行程序自动从初始化函数加载用户模块，无需额外的代码
//第一个参数为模块的名字
//第二个参数为模块的初始化函数
// USER_EXPORT(data_receive,data_receive_init);
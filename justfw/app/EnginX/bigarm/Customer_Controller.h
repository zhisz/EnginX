//
// Created by CODE01 on 2025/12/29.
//

#ifndef JUSTFW_CUSTOMER_CONTROLLER_H
#define JUSTFW_CUSTOMER_CONTROLLER_H
#include <stdint.h>

typedef struct Controller_S2M_Packet {
    uint8_t header;            // 包头0xAA
    float arm_angle[6];           // 机械臂角度
    uint16_t checksum;         // 校验位自动算
} __attribute__((packed)) Controller_S2M_PacketTypeDef;

union Controller_S2M_Union {
    Controller_S2M_PacketTypeDef controller_msg;//原始包
    uint8_t bit_flow[sizeof(Controller_S2M_PacketTypeDef)];//联合体会自动将上述结构体转换为字节流
};

void Controller_logic_init();

#endif //JUSTFW_CUSTOMER_CONTROLLER_H
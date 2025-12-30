//
// Created by CODE01 on 2025/12/30.
//

#ifndef JUSTFW_MASTER_ARM_DATA_RECIEVE_H
#define JUSTFW_MASTER_ARM_DATA_RECIEVE_H
#include <stdint.h>


typedef struct arm_data_M2S_Packet {
    uint8_t header;
    float arm_angle[6];
    uint16_t checksum;
} __attribute__((packed)) arm_data_M2S_PacketTypeDef;

union arm_data_M2S_Union {
    arm_data_M2S_PacketTypeDef arm_data_m2s;
    uint8_t bit_flow[sizeof(arm_data_M2S_PacketTypeDef)];
};



void data_receive_init();
#endif //JUSTFW_MASTER_ARM_DATA_RECIEVE_H
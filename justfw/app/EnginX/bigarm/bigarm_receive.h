#pragma once
#include <stdint.h>
#include <stddef.h>
#include "Customer_Controller.h"
#include "task.h"

// 你这帧固定 27 字节
#define CONTROLLER_PKT_LEN (sizeof(Controller_S2M_PacketTypeDef))
#define CONTROLLER_HEADER  (0xAA)

typedef void (*ControllerRx_OnPacket)(const Controller_S2M_PacketTypeDef *pkt, void *user);

typedef struct {
    uint8_t  buf[CONTROLLER_PKT_LEN];
    uint16_t idx;
    uint8_t  syncing; // 0: 等包头, 1: 收数据
} ControllerRxParser;

void ControllerRx_Init(ControllerRxParser *p);

/**
 * @brief 把接收到的字节流喂进解析器
 * @return 本次解析出来的完整有效包数量
 */
size_t ControllerRx_Feed(ControllerRxParser *p,
                         const uint8_t *data, size_t len,
                         ControllerRx_OnPacket cb, void *user);

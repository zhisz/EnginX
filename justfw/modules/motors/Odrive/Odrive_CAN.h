#ifndef __ODRIVE_CAN_H
#define __ODRIVE_CAN_H

#include "Odrive_defination.h"
#include "interface.h"

void Odrive_Init();

INTF_Motor_HandleTypeDef *Odrive_Register(Odrive_CAN_ConfigTypedef *config);

#endif
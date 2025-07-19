//
// Created by Lszhi on 25-7-12.
//

/// 旋转根部	SubArm1_BaseMotor
/// 抬升关节	SubArm1_PitchMotor
/// 手腕转动	SubArm1_WristMotor
/// 前伸控制	SubArm1_ExtendMotor

#include "Odrive_defination.h"
#include "Odrive_CAN.h"


#ifndef SUBARMS_H
#define SUBARMS_H



void subarms_Init();


///Subarm1_BaseMotor配置
void subarm1_MotorInit();
void SubArm1_BaseMotor_Init();
void SubArm1_WristMotor_Init();
void SubArm1_PitchMotor_Init();
void SubArm1_ExtendMotor_Init();

///Subarm1_BaseMotor配置
void subarm2_MotorInit();
void SubArm2_BaseMotor_Init();
void SubArm2_WristMotor_Init();
void SubArm2_PitchMotor_Init();
void SubArm2_ExtendMotor_Init();

///Lift_Motor配置
void LiftMotor_Init();





#endif //SUBARMS_H

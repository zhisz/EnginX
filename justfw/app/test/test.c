#include <stdio.h>
#include "test.h"
#include <tgmath.h>
#include "intf_dr16.h"
#include "intf_motor.h"
#include "shared_ptr_intf.h"
#include "task.h"
#include "tinybus_intf.h"
#include "usbd_cdc_if.h"
#include "BSP_bmi088.h"

RC_ctrl_t *test_logic_rc_ctrl;

INTF_Motor_HandleTypeDef *test_motor;
INTF_Motor_HandleTypeDef *test_motor2;
// INTF_Motor_HandleTypeDef *lift_motor;
INTF_Motor_HandleTypeDef *spin;




extern  int g_dr16_is_connected;

void Test_MainLoop()
{

    while (1)
    {



        while (0)
        {
            // printf("USB_OK");
            float anglex = test_logic_rc_ctrl[0].rc.rocker_l_/660.0f*200.0f;
            float angley = test_logic_rc_ctrl[0].rc.rocker_r1/660.0f*10.0f;

            test_motor2->set_speed(test_motor2,10.0f);


            // if (anglex<0) anglex=0;
            spin->set_angle(spin, anglex);

            printf("anglex:%f",anglex);
            // test_motor->set_angle(test_motor, anglex);

            // lift_motor->set_angle(lift_motor, angley);
            // printf("angley:%f",angley);

            vTaskDelay(10);

        }
        vTaskDelay(10);
    }
}

void Test_Init()
{
    test_motor = pvSharePtr("test_motor", sizeof(INTF_Motor_HandleTypeDef));
    test_motor2 = pvSharePtr("test_motor2", sizeof(INTF_Motor_HandleTypeDef));
    // lift_motor = pvSharePtr("lift_motor", sizeof(INTF_Motor_HandleTypeDef));
    spin = pvSharePtr("spin", sizeof(INTF_Motor_HandleTypeDef));



    test_logic_rc_ctrl = pvSharePtr("DR16", sizeof(RC_ctrl_t));
    xTaskCreate(Test_MainLoop, "Test_MainLoop", 1024, NULL, 240, NULL);
}
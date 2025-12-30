#include "task.h"
#include "bigarm_receive.h"
#include <stdio.h>
#include "shared_ptr_intf.h"



void Controller_MainLoop()
{
    Controller_S2M_PacketTypeDef pkt;
}


void Controller_Init() {



    xTaskCreate(Controller_MainLoop, "Controller_MainLoop", 256, NULL, 5, NULL);

}


#include <stdio.h>
#include <string.h>

#include "main.h"
#include "projectMain.h"
#include "math.h"

// #include "usb_device.h"

// #include "message_buffer.h"
// #include "queue.h"
// #include "cmsis_os.h"



void ProjectMain()
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(1000);
    }
}



#include "device.h"
#include "fault.h"
#include "driver_PWM.h"
#include "driver_GPIO.h"

void fault(int fault_id) 
{   
    PWM_override_OFF();     //keep phases off
}
#include "hal/include/hal_motor.h"
#include "hal/include/hal_drivers.h"

typedef struct
{
	int remainingStep;
	RotationDirection dir;
} HalMotorControl_t;

static HalMotorControl_t g_halMotorControl;

void HalMotor_init(void)
{
}

void HalMotor_run(uint8 step, RotationDirection dir)
{
	UNUSED_VAR(step);
	UNUSED_VAR(dir);

	osal_stop_timerEx(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
	osal_set_event(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
}

void HalMotor_update(void)
{
	if(g_halMotorControl.remainingStep == 0)
	{
		// stop motor
		osal_stop_timerEx(Hal_TaskID, HAL_MOTOR_RUN_EVENT);		// remove from osal timer
	}
	else
	{
		osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT, 100);	/* Schedule event */
	}
	
}

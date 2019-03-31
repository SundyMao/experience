/**************************************************************************************************
  Filename:       hal_led.c
  Revised:        $Date: 2019-03-31 13:43:32(UTC+8) $
  Revision:       $Revision: 29281 $

  Description:    This file contains the interface to the HAL Steering engine Service.
**************************************************************************************************/

#include "hal/include/hal_motor.h"
#include "hal/include/hal_drivers.h"
#include "osal/include/osal.h"

#define MOTOR_PLUSE_INTERVAL 100

typedef enum
{
	MotorPhase_none = 0,
	MotorPhase_a,
	MotorPhase_b,
	MotorPhase_c,
	MotorPhase_d,
	MotorPhase_max
} MotorPhase;

typedef struct
{
	int remainingStep;
	RotationDirection dir;
	MotorPhase phase;
} HalMotorControl_t;

static HalMotorControl_t g_motorControl;

/********************************************************************************
 *									Local Function								*
 *******************************************************************************/
void HalMotor_stop(void);
MotorPhase HalMotor_runOneStep(MotorPhase currentPhase, RotationDirection currentDir);
void HalMotor_update(void);

/***************************************************************************************************
 * @fn      HalMotor_init
 *
 * @brief   initialize motor devices
 *
 * @param   void
 *
 * @return  none
 ***************************************************************************************************/
void HalMotor_init(void)
{
}

/***************************************************************************************************
 * @fn      HalMotor_run
 *
 * @brief   make motor to working
 *
 * @param   step - step to run
 * 			dir - direction to run, clockwise or counter clockwise.
 *
 * @return  none
 ***************************************************************************************************/
void HalMotor_run(int step, RotationDirection dir)
{
	if (step == 0 || dir == RotationDirection_none)
		return;

	if (g_motorControl.dir == dir)
	{
		g_motorControl.remainingStep += step;
	}
	else
	{
		/*stop motor*/
		HalMotor_stop();
		/*reset control message*/
		g_motorControl.dir = dir;
		g_motorControl.phase = MotorPhase_none;
		g_motorControl.remainingStep = step;
	}
	osal_stop_timerEx(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
	osal_set_event(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
}

/***************************************************************************************************
 * @fn      HalMotor_update
 *
 * @brief   Update motor to work
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalMotor_update(void)
{
	if(g_motorControl.remainingStep != 0)
	{
		HalMotor_runOneStep(g_motorControl.phase, g_motorControl.dir);
		g_motorControl.remainingStep -= 1;
	}
	if(g_motorControl.remainingStep != 0)
	{
		osal_start_timerEx(Hal_TaskID, HAL_LED_BLINK_EVENT, MOTOR_PLUSE_INTERVAL);	/* Schedule event */
	}
}

/***************************************************************************************************
 * @fn      HalMotor_stop
 *
 * @brief   Update motor to work
 *
 * @param   none
 *
 * @return  none
 ***************************************************************************************************/
void HalMotor_stop(void)
{
	g_motorControl.dir = RotationDirection_none;
	g_motorControl.remainingStep = 0;
	g_motorControl.phase = MotorPhase_none;
	// TODO: set port to stop motor.
}

/***************************************************************************************************
 * @fn      HalMotor_stop
 *
 * @brief   make motor running just one step.
 *
 * @param   currentPhase - current phase of motor
 * 			currentDir - current rotation of motor.
 *
 * @return  MotorPhase next phase that motor will be.
 ***************************************************************************************************/
MotorPhase HalMotor_runOneStep(MotorPhase currentPhase, RotationDirection currentDir)
{
	//TODO: return next phase status
	MotorPhase nextPhase = MotorPhase_d;
	return nextPhase;
}

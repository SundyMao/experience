/**************************************************************************************************
  Filename:       hal_led.c
  Revised:        $Date: 2019-03-31 13:43:32(UTC+8) $
  Revision:       $Revision: 0 $

  Description:    This file contains the interface to the HAL Steering engine Service.
**************************************************************************************************/

#include <ioCC2530.h>

#include "hal/include/hal_motor.h"
#include "hal/include/hal_drivers.h"
#include "osal/include/osal.h"

/*************************************************************************************************
* P0_0 connect PIN A
* P0_1 connect PIN B
* P0_2 connect PIN C
* P0_3 connect PIN D
*/
#define HalMotor_port P0   //
#define HalMotor_portDir P0DIR 

uint8 g_phaseTableClockwise[]        = { 0x00, 0x09, 0x08, 0x0c, 0x04, 0x06, 0x02, 0x03, 0x01 }; // 半步工作 顺时针 0x00 占位
uint8 g_phaseTableCounterClockwise[] = { 0x00, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08, 0x09 }; // 半步工作 逆时针 0x00 占位


// debug for 1000, actually 10
#define MOTOR_PLUSE_INTERVAL 5000

typedef enum
{
	MotorPhase_none = 0,
	MotorPhase_a,
	MotorPhase_b,
	MotorPhase_c,
	MotorPhase_d,
	MotorPhase_e,
	MotorPhase_f,
	MotorPhase_g,
	MotorPhase_h,
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
	HalMotor_portDir = 0x0F;
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

	bool keepStatus = true;
	if (g_motorControl.dir == dir)
	{
		keepStatus = g_motorControl.remainingStep != 0;
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
		
		keepStatus = false;
	}
	
	if (!keepStatus)
	{
		osal_set_event(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
	}
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
		g_motorControl.phase = HalMotor_runOneStep(g_motorControl.phase, g_motorControl.dir);
		g_motorControl.remainingStep -= 1;
	}
	
	if(g_motorControl.remainingStep != 0)
		osal_start_timerEx(Hal_TaskID, HAL_MOTOR_RUN_EVENT, MOTOR_PLUSE_INTERVAL);	/* Schedule event */
	else
		osal_clear_event(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
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

	// set port with 0x00
	HalMotor_port = g_phaseTableClockwise[0];
	
	osal_stop_timerEx(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
	osal_clear_event(Hal_TaskID, HAL_MOTOR_RUN_EVENT);
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
	if (currentDir == RotationDirection_clockwise)
		HalMotor_port = g_phaseTableClockwise[currentPhase];
	else
		HalMotor_port = g_phaseTableCounterClockwise[currentPhase];

	MotorPhase nextPhase = (MotorPhase)((int)currentPhase + 1);
	if (nextPhase >= MotorPhase_max)
		nextPhase = MotorPhase_a;

	return nextPhase;
}

// example code
/*

uint8 table_l[] = {0x09, 0x08, 0x0c, 0x04, 0x06, 0x02, 0x03, 0x01}; // 鍗婃宸ヤ綔 椤烘椂閽?
uint8 table_r[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0c, 0x08, 0x09}; // 鍗婃宸ヤ綔 閫嗘椂閽?
uint8 step;

void DelayMS(uint msec)
{
	uint8 x, y;
	for (x = msec; x>0; x--)
	{
		for (y = 110; y > 0; y--);
	}
}

void main(void)
{
	P0DIR = 0x0F;
	while(1)
	{
		for (int i = 0; i < 1000; i++)
		{
			if (step > 7)
				step = 0;
			P0 = table_l[step];
			step++;
			DelayMS(10);
		}
		DelayMS(50);
		for (int i = 0; i < 1000; i++)
		{
			if (step > 7)
				step = 0;
			P0 = table_r[step];
			step++;
			DelayMS(10);
		}
		DelayMS(50);
	}
}

*/
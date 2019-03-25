#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "hal/include/hal_board.h"

#define MOTOR_PIN_A P0_0
#define MOTOR_PIN_B P0_1
#define MOTOR_PIN_C P0_2
#define MOTOR_PIN_D P0_3

#define EXCITATION_STATUS_A 0x01
#define EXCITATION_STATUS_B 0x02
#define EXCITATION_STATUS_C 0x04
#define EXCITATION_STATUS_D 0x08

typedef enum
{
	MotorRotation_none = 0,
	MotorRotation_clockwise,
	MotorRotation_counterClockwise
} MotorRotation;

/*
 * Initialize Motor Service.
 */
void HalMotor_init(void);

/*
* Rotate the motor according to the specified steps and directions
*/
void HalMotor_run(uint8 step, MotorRotation rotation);

#ifdef __cplusplus
}
#endif

#endif
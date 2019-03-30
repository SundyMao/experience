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
	RotationDirection_none = 0,
	RotationDirection_clockwise,
	RotationDirection_counterClockwise
} RotationDirection;

/*
 * Initialize Motor Service.
 */
void HalMotor_init(void);

/*
* Rotate the motor according to the specified steps and directions
*/
void HalMotor_run(uint8 step, RotationDirection dir);

#ifdef __cplusplus
}
#endif

#endif
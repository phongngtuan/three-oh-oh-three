/* Header file for library functions */

#ifndef __MOTOR_LIB_H__
#define __MOTOR_LIB_H__

#define RIGHT_ANGLE 8
#include <bsp.h>

enum opt{
	BLOCKED_TURN = 0,
	COUNTER_TURN
};

typedef enum opt opt_t;

extern void motors_init();
extern void RoboMove(tDirection dir, CPU_INT16U seg, CPU_INT16U speed);
extern void RoboTurn(tSide dir, CPU_INT16U segments, opt_t option,CPU_INT16U speed);
extern int RoboStopNow(void);
#endif  //__MOTOR_LIB_H__
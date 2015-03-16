#include "motor_lib.h"

OS_Q IntQ;

CPU_INT08U Left_tgt;
CPU_INT08U Right_tgt;

void IntWheelSensor();

/* *****************************************************************************
*		INITIALIZE THE MOTORS, QUEUE and REGISTER ISR                  *
***************************************************************************** */

void motors_init()
{
	tSide sideR = RIGHT_SIDE;
	tSide sideL = LEFT_SIDE;
	tSensor sensor = SENSOR_A;											
	
	OS_ERR  err;
										// All variable declarations

	OSQCreate((OS_Q *)&IntQ,
              (CPU_CHAR *)"ISR Message Queue",
              (OS_MSG_QTY)10,
              (OS_ERR *)&err);
										// Create a message passing queue

	Left_tgt = 0;
	Right_tgt = 0;	
										
	BSP_MotorsInit();
	BSP_WheelSensorsInit();							// Init functions		
	
	BSP_WheelSensorIntEnable(sideR, sensor, (CPU_FNCT_VOID)IntWheelSensor);
	BSP_WheelSensorIntEnable(sideL, sensor, (CPU_FNCT_VOID)IntWheelSensor);	//Register the ISRs

	return;
}

/* *****************************************************************************
*		THE WHEEL SENSOR ISR                *
***************************************************************************** */
static CPU_INT08U CountL = 0;
static CPU_INT08U CountR = 0;
void IntWheelSensor()
{
        OS_ERR err;

	CPU_INT32U         ulStatusR_A;
	CPU_INT32U         ulStatusL_A;

	

	static CPU_INT08U data = 0;

	ulStatusR_A = GPIOPinIntStatus(RIGHT_IR_SENSOR_A_PORT, DEF_TRUE);
	ulStatusL_A = GPIOPinIntStatus(LEFT_IR_SENSOR_A_PORT, DEF_TRUE);

        if (ulStatusR_A & RIGHT_IR_SENSOR_A_PIN)    //Exp
        {
          GPIOPinIntClear(RIGHT_IR_SENSOR_A_PORT, RIGHT_IR_SENSOR_A_PIN);           /* Clear interrupt.*/
          CountR = CountR + 1;
        }

        if (ulStatusL_A & LEFT_IR_SENSOR_A_PIN)      //Exp
        {
          GPIOPinIntClear(LEFT_IR_SENSOR_A_PORT, LEFT_IR_SENSOR_A_PIN);
          CountL = CountL + 1;
        }

	if((CountL >= Left_tgt) && (CountR >= Right_tgt)){
          data = 0x11;
          Left_tgt = 0;
          Right_tgt = 0;
          CountL = 0;
          CountR = 0;
          
          OSQPost((OS_Q *)&IntQ,
             (CPU_INT08U *)data,
             (OS_MSG_SIZE)sizeof(CPU_INT16U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)&err);
        }
        else if(CountL >= Left_tgt){
          data = 0x10;
          Left_tgt = 0;
          CountL = 0;
          
          OSQPost((OS_Q *)&IntQ,
             (CPU_INT08U *)data,
             (OS_MSG_SIZE)sizeof(CPU_INT16U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)&err);
        }
        else if(CountR >= Right_tgt){
          data = 0x01;
          Right_tgt = 0;
          CountR = 0;
          
          OSQPost((OS_Q *)&IntQ,
             (CPU_INT08U *)data,
             (OS_MSG_SIZE)sizeof(CPU_INT16U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)&err);
        }

     
}


/* *****************************************************************************
*			    ROBOT MOVE FUNCTION		                       *
***************************************************************************** */

void RoboMove(tDirection dir, CPU_INT16U seg, CPU_INT16U speed)
{
	OS_ERR err;
	OS_MSG_SIZE size;
	CPU_TS ts;
        
        CPU_INT08U counts = 0;
        
        Left_tgt = seg;
        Right_tgt = seg;
 		              
        BSP_MotorStop(LEFT_SIDE);
	BSP_MotorStop(RIGHT_SIDE);

	BSP_MotorSpeed(LEFT_SIDE, speed <<8u);
	BSP_MotorSpeed(RIGHT_SIDE,speed <<8u);

	BSP_MotorDir(LEFT_SIDE,dir);
	BSP_MotorDir(RIGHT_SIDE,dir);					        // Set the motor parameters
	
	BSP_MotorRun(LEFT_SIDE);
	BSP_MotorRun(RIGHT_SIDE);						// Start the motors
	
	OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);           // Just a small wait time to let the motors run


	do{

                counts = (CPU_INT08U)OSQPend((OS_Q *)&IntQ,
                                       (OS_TICK)0,                              // wait untill the mesage is recieved : no timeout
                                       (OS_OPT)OS_OPT_PEND_BLOCKING,
                                       (OS_MSG_SIZE *)&size,
                                       (CPU_TS *)&ts,
                                       (OS_ERR *)&err);

				

              	if(counts == 0x11){			
		    BSP_MotorStop(RIGHT_SIDE);
                    BSP_MotorStop(LEFT_SIDE);
                    Left_tgt = 0;
                    Right_tgt = 0;
	        }
                else if(counts == 0x10){			
		    BSP_MotorStop(LEFT_SIDE);
                    Left_tgt = 0;
	        }
                else if(counts == 0x01){			
		    BSP_MotorStop(RIGHT_SIDE);
                    Right_tgt = 0;
	        }
	
	}while((Left_tgt)||(Right_tgt));
		
	return;
}

/* *****************************************************************************
*			    ROBOT TURN FUNCTION		                       *
***************************************************************************** */
void RoboTurn(tSide dir, CPU_INT16U seg, opt_t option,CPU_INT16U speed)
{
	OS_ERR err;
	OS_MSG_SIZE size;
	CPU_TS ts;
	
	CPU_INT16U counts = 0;
    
	Left_tgt = seg;
        Right_tgt = seg;

	BSP_MotorStop(LEFT_SIDE);
	BSP_MotorStop(RIGHT_SIDE);

        BSP_MotorSpeed(LEFT_SIDE, speed <<8u);
	BSP_MotorSpeed(RIGHT_SIDE,speed <<8u);

	switch(dir)
	{
	case LEFT_SIDE :
                BSP_MotorDir(RIGHT_SIDE,FORWARD);
		if(option == COUNTER_TURN){                                                    // Is counter rotation is enables ie. both wheels will move
			BSP_MotorDir(LEFT_SIDE,REVERSE);
			BSP_MotorRun(LEFT_SIDE);
		}
		BSP_MotorRun(RIGHT_SIDE);
		break;
	case RIGHT_SIDE:
                BSP_MotorDir(LEFT_SIDE,FORWARD);
		if(option == COUNTER_TURN){
			BSP_MotorDir(RIGHT_SIDE,REVERSE);
			BSP_MotorRun(RIGHT_SIDE);
		}
		BSP_MotorRun(LEFT_SIDE);
		break;
	default:
		BSP_MotorStop(LEFT_SIDE);
		BSP_MotorStop(RIGHT_SIDE);
		break;
	}

	OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);          //wait for sometime: give time for motors to start running

	do{

                counts = (CPU_INT08U)OSQPend((OS_Q *)&IntQ,
                                       (OS_TICK)0,                              // wait untill the mesage is recieved : no timeout
                                       (OS_OPT)OS_OPT_PEND_BLOCKING,
                                       (OS_MSG_SIZE *)&size,
                                       (CPU_TS *)&ts,
                                       (OS_ERR *)&err);

				

              	if(counts == 0x11){			
		    BSP_MotorStop(RIGHT_SIDE);
                    BSP_MotorStop(LEFT_SIDE);
                    Left_tgt = 0;
                    Right_tgt = 0;
	        }
                else if(counts == 0x10){			
		    BSP_MotorStop(LEFT_SIDE);
                    Left_tgt = 0;
	        }
                else if(counts == 0x01){			
		    BSP_MotorStop(RIGHT_SIDE);
                    Right_tgt = 0;
	        }
	
	}while((Left_tgt)||(Right_tgt));
		
	return;
}

int RoboStopNow(void)
{

	Left_tgt = 0;
	Right_tgt = 0;
        return (CountL+CountR)/2;
}
#include <includes.h>
#include "motor_lib.h"

#define BSP_PushButtonGetStatus(a) (0)

static  OS_TCB       AppTaskStartTCB;
static  OS_TCB       AppTaskRobotControlTCB;
static  OS_TCB       AppTaskMotorControlTCB;
static  OS_TCB       AppTaskTimerControlTCB;
static  OS_TCB       AppTaskInputMonitorTCB;

static  CPU_STK      AppTaskStartStk[APP_TASK_START_STK_SIZE];
static  CPU_STK      AppTaskRobotControlStk[APP_TASK_ROBOT_CONTROL_STK_SIZE];
static  CPU_STK      AppTaskMotorControlStk[APP_TASK_MOTOR_CONTROL_STK_SIZE];
static  CPU_STK      AppTaskTimerControlStk[APP_TASK_TIMER_CONTROL_STK_SIZE];
static  CPU_STK      AppTaskInputMonitorStk[APP_TASK_INPUT_MONITOR_STK_SIZE];

static  void  AppTaskStart        (void  *p_arg);
static  void  AppTaskRobotControl (void  *p_arg);
static  void  AppTaskMotorControl (void  *p_arg);
static  void  AppTaskTimerControl (void);
void  AppTaskInputMonitor (void  *p_arg);
static  void  AppTasksCreate (void);

int main()
{
  OS_ERR  err; 
    
      OSInit(&err);
      
    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                                        
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR ) AppTaskStart,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK |                                                          OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    
    OSStart(&err);  /* give control to uC/OS-III. */

  return 0;
}

static  void  AppTaskStart (void  *p_arg)
{
    CPU_INT32U  clk_freq;
    CPU_INT32U  cnts;
    CPU_INT32U  ulPHYMR0;
    OS_ERR      err;


   (void)&p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);                  /* Enable and Reset the Ethernet Controller.            */
    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

    ulPHYMR0 = EthernetPHYRead(ETH_BASE, PHY_MR0);              /* Power Down PHY                                       */
    EthernetPHYWrite(ETH_BASE, PHY_MR0, ulPHYMR0 | PHY_MR0_PWRDN);
    SysCtlPeripheralDeepSleepDisable(SYSCTL_PERIPH_ETH);


    clk_freq = BSP_CPUClkFreq();                                /* Determine SysTick reference freq.                    */
    cnts     = clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */
    CPU_TS_TmrFreqSet(clk_freq);

#if (OS_CFG_STAT_TASK_EN > 0u)
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

    CPU_IntDisMeasMaxCurReset();
    
    motors_init();

    AppTasksCreate();
    
    while (DEF_ON) {                                            /* Task body, always written as an infinite loop.       */
        OSTimeDlyHMSM(0u, 0u, 1u, 0u,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

    }


}

static  void  AppTaskMotorControl (void  *p_arg)
{
     OS_ERR  err;
     OS_MSG_SIZE size;
     CPU_TS ts;
     
     CPU_INT32U msg_rx;
     CPU_INT08U motor_dir;
     CPU_INT16U motor_seg;
     CPU_INT08U motor_speed;
     
     
     CPU_INT16U msg_tx;
     CPU_INT08U tx_src;
     CPU_INT08U tx_val;
     
     CPU_INT08U state = 0;
         
          
     while(1)
     {
       // Wait for information from the Task queue
       msg_rx = (CPU_INT32U)OSTaskQPend( (OS_TICK)0,                            
                                         (OS_OPT)OS_OPT_PEND_BLOCKING,
                                         (OS_MSG_SIZE *)&size,
                                         (CPU_TS *)&ts,
                                         (OS_ERR *)&err);
       // Extract information                       
       motor_seg = msg_rx;
       motor_speed = (msg_rx >> 16u);
       motor_dir = (msg_rx >> 24u);
       
       
          state = motor_dir;
          
          switch(state)
          {
          case 0: // Move straight
                 RoboMove(FORWARD,motor_seg,(CPU_INT16U)motor_speed);
                 tx_val = 0;
            break;
          case 1: // Move back
                 RoboMove(REVERSE,motor_seg,(CPU_INT16U)motor_speed);
                 tx_val = 1;
            break;
          case 2: // Turn Left
             RoboTurn(LEFT_SIDE,motor_seg,COUNTER_TURN,(CPU_INT16U)motor_speed);
                 tx_val = 2;
            break;
          case 3: // Turn Right
            RoboTurn(RIGHT_SIDE,motor_seg,COUNTER_TURN,(CPU_INT16U)motor_speed);
                 tx_val = 3;
            break;
          default:
                BSP_DisplayClear();
                BSP_DisplayStringDraw("ERROR 0: HALTED",10u,0u);
                BSP_DisplayStringDraw("DO RESET",30u,1u);
                while(1);
            break;
          }
          tx_src = 0;
          
          // Tell the control task that job has been done
          msg_tx = ((tx_src <<8u)|(tx_val));
          OSTaskQPost(&AppTaskRobotControlTCB,
                      (CPU_INT16U *)msg_tx,
                      (OS_MSG_SIZE)sizeof(CPU_INT16U *),
                      (OS_OPT)OS_OPT_POST_FIFO,
                      (OS_ERR *)&err);
         
       }
        
}
static  void  AppTaskTimerControl(void)
{
     OS_ERR  err;
     OS_MSG_SIZE size;
     CPU_TS ts;
     
     CPU_INT16U time;
     
     CPU_INT16U msg_tx;
     CPU_INT08U tx_src;
     CPU_INT08U tx_val;
     
     CPU_INT08U msec =0;
     CPU_INT08U sec =0;
     
     while(1){
    // Wait for the timeout information in the message queue
       time = (CPU_INT16U)OSTaskQPend( (OS_TICK)0,  
                                       (OS_OPT)OS_OPT_PEND_BLOCKING,
                                       (OS_MSG_SIZE *)&size,
                                       (CPU_TS *)&ts,
                                       (OS_ERR *)&err);
       msec = time % 1000;
       sec =  time / 1000;
       
       //sec = 1;
       //msec = 500;
       
       OSTimeDlyHMSM(0u, 0u, sec, msec, OS_OPT_TIME_HMSM_STRICT, &err);
       // Wait until timeout 

       tx_src = 2;
       tx_val = 1;
       
       // After timeout inform the control task that time is over
       msg_tx = ((tx_src << 8u)|(tx_val));
       OSTaskQPost(&AppTaskRobotControlTCB,
                  (CPU_INT16U *)msg_tx,
                  (OS_MSG_SIZE)sizeof(CPU_INT16U *),
                  (OS_OPT)OS_OPT_POST_FIFO,
                  (OS_ERR *)&err);
       
     }
}

static  void  AppTaskInputMonitor (void  *p_arg)
{
    CPU_INT08U ucData;
    OS_ERR     err;
    
    CPU_INT32U msg_tx;
    CPU_INT08U tx_src;
    CPU_INT08U tx_val;

    // The debounced state of the 4 switches. The bit positions
    // correspond to:
    //
    //     0 - Right Push Button
    //     1 - Left  Push Button
    //     2 - Right Bump Sensor
    //     3 - Left  Bump Sensor
    CPU_INT08U  ucSwitches;
    
     CPU_INT08U  ucDelta;
    
    // The vertical counter used to debounce the switches.  The
    // bit positions are the same as g_ucPushButtons.
    CPU_INT08U  ucSwitchesClockA;
    CPU_INT08U  ucSwitchesClockB;


   (void)&p_arg;

       /* Initialize the variables  */
    ucSwitches       = 0x0Fu;
    ucSwitchesClockA =    0u;
    ucSwitchesClockB =    0u;
    
    while (DEF_ON) {
      /* Delay for 5 milliseconds.    */
        OSTimeDlyHMSM(0u, 0u, 0u, 5u, 
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

       /* Read the state of the switches. */
        ucData =  BSP_PushButtonGetStatus(1u) |        /*Right Push Button */
                 (BSP_PushButtonGetStatus(2u) << 1u) | /*Left  Push Button */
                 (BSP_BumpSensorGetStatus(1u) << 2u) | /*Right Bump Sensor*/
                 (BSP_BumpSensorGetStatus(2u) << 3u);  /*Left  Bump Sensor */
                                                                
   /* Determine the switches at a different state than ... */
        ucDelta = ucData ^ ucSwitches;                         
   /* ... the debounced state.                             */

        ucSwitchesClockA ^=  ucSwitchesClockB; 
   /* Increment the clocks by one.                         */
        ucSwitchesClockB  = ~ucSwitchesClockB;

        ucSwitchesClockA &= ucDelta;                           
 /* Reset the clocks corresponding to switches that ...  */

        ucSwitchesClockB &= ucDelta;                            
/* ... have not changed state.                          */

                                                               
 /* Get the new debounced switch state.                  */

        ucSwitches &=    ucSwitchesClockA | ucSwitchesClockB;
        ucSwitches |= (~(ucSwitchesClockA | ucSwitchesClockB)) & ucData;

        ucDelta ^= ucSwitchesClockA | ucSwitchesClockB;         

/* Determine switches that changed debounced state.     */

        if (ucDelta & 0x0Cu) { /* Check if any bump sensor switched state.   */
                    
            tx_src = 1;
                        
            if((ucDelta & 0x08) && (ucDelta & 0x04))
              tx_val = 1;
            else if (ucDelta & 0x08)         //Left one is pressed 
              tx_val = 2;
            else if (ucDelta & 0x04)        //Right one is pressed 
              tx_val = 3;

            msg_tx = ((tx_src << 8u)|(tx_val));
            OSTaskQPost(&AppTaskRobotControlTCB,
                        (CPU_INT16U *)msg_tx,
                        (OS_MSG_SIZE)sizeof(CPU_INT16U *),
                        (OS_OPT)OS_OPT_POST_FIFO,
                        (OS_ERR *)&err);
        }
    }
}

static  void  AppTaskRobotControl (void  *p_arg)
{
     OS_ERR  err;
     OS_MSG_SIZE size;
     CPU_TS ts;
     
     CPU_INT16U msg_rx;
     CPU_INT08U rx_src;
     CPU_INT08U rx_val;                       //Variables to receive message
     
     CPU_INT32U msg_motor;
     CPU_INT08U motor_dir;
     CPU_INT16U motor_seg;
     CPU_INT08U motor_speed;         //Variables to post messages to motor task        
     
     CPU_INT16U msg_timer;             //variables to post messages to the timer

     CPU_INT08U state = 0;
     CPU_INT08U bump_state = 0;
     CPU_INT08U timer_state = 0;
     
     srand(123456);
     
     motor_dir  = 0;
     motor_speed = 80u; 
     motor_seg = 10u;       // Tell the motors to run forward
     msg_motor = ((motor_dir << 24u)|(motor_speed << 16u)|(motor_seg));
     OSTaskQPost(&AppTaskMotorControlTCB,
                 (CPU_INT32U *)msg_motor,
                 (OS_MSG_SIZE)sizeof(CPU_INT32U *),
                 (OS_OPT)OS_OPT_POST_FIFO,
                 (OS_ERR *)&err);           
     
    msg_timer = 1000;      // Generate random timeout
    OSTaskQPost(&AppTaskTimerControlTCB,
               (CPU_INT16U *)msg_timer,
               (OS_MSG_SIZE)sizeof(CPU_INT16U *),
               (OS_OPT)OS_OPT_POST_FIFO,
               (OS_ERR *)&err);          
                            // Tell the timer control task to start running
    
     while(1){
     
     
       msg_rx = (CPU_INT16U)OSTaskQPend((OS_TICK)0,  
                                       (OS_OPT)OS_OPT_PEND_BLOCKING,
                                       (OS_MSG_SIZE *)&size,
                                       (CPU_TS *)&ts,
                                       (OS_ERR *)&err);
       rx_val = msg_rx;
       rx_src = (msg_rx >> 8u);     // Wait for incoming messages
       
       BSP_DisplayClear();
       
       switch(rx_src){     // Resolve Source
       
       case 0: // This is motor control
               // Tell the motor control task to execute next state
         BSP_DisplayStringDraw("receive 0",10u,1u);               
         if(rx_val != 0){            // This means it has completed the turn
           motor_dir  = 0;           // Go Straight
           motor_speed = 80u; 
           motor_seg =15u;
         }
         else{          // This means it has completed a forward or reverse run     
                 
          motor_dir = 3;     // Turn Right 
                
          motor_speed = 50u; 
          motor_seg = 15u;
                     
         }                      
         if(state >=3){
           BSP_DisplayClear();
           
           if(bump_state)
             BSP_DisplayStringDraw("Bumpers",10u,1u);
           else if(timer_state)
             BSP_DisplayStringDraw("Timer",10u,1u);
           else
             BSP_DisplayStringDraw("Distance",10u,1u);
           while(1);
         }
         state++;
         
         msg_motor = ((motor_dir << 24u)|(motor_speed << 16u)|(motor_seg));
         OSTaskQPost(&AppTaskMotorControlTCB,
                   (CPU_INT32U *)msg_motor,
                   (OS_MSG_SIZE)sizeof(CPU_INT32U *),
                   (OS_OPT)OS_OPT_POST_FIFO,
                   (OS_ERR *)&err);
                            // Send subsequent messages to the motor task 
         break;
       case 1: //This is bump sensors
                BSP_DisplayStringDraw("Received 1",10u,1u);               
               //BSP_DisplayStringDraw("Bumpers detected",10u,1u);
               if(bump_state)
                 break;

               RoboStopNow();               // Stop the motors immediately
               bump_state = 1;            // Indicate that obstacle is detected                    
                                          // and distance has not expired
                                          // nor did the timer expire   
         break;
       case 2: //This is timer expiry
               if(!state){
                    RoboStopNow();         // Stop the robot
                    timer_state = 1;       // Indicate that timer has expired                    
               }                           // and distance has not expired
                                          // nor did any obstacle being detected
               break;
       default:
                BSP_DisplayClear();
                BSP_DisplayStringDraw("ERROR 3: HALTED",10u,0u);
                BSP_DisplayStringDraw("DO RESET",30u,1u);
                while(1);
         break;
       }
       
              
     }

}

static  void  AppTasksCreate (void)
{
    OS_ERR  err;

    /* Create Control Task                                  */
    OSTaskCreate((OS_TCB     *)&AppTaskRobotControlTCB,
                 (CPU_CHAR   *)"Main Control Task",
                 (OS_TASK_PTR ) AppTaskRobotControl,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_ROBOT_CONTROL_PRIO,
                 (CPU_STK    *)&AppTaskRobotControlStk[0],
                 (CPU_STK_SIZE) APP_TASK_ROBOT_CONTROL_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_TASK_ROBOT_CONTROL_STK_SIZE,
                 (OS_MSG_QTY  ) 20u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err); 
    
    
    OSTaskCreate((OS_TCB     *)&AppTaskMotorControlTCB,
                   (CPU_CHAR   *)"Motor Control Task",
                   (OS_TASK_PTR ) AppTaskMotorControl,
                   (void       *) 0,
                   (OS_PRIO     ) APP_TASK_MOTOR_CONTROL_PRIO,
                   (CPU_STK    *)&AppTaskMotorControlStk[0],
                   (CPU_STK_SIZE) APP_TASK_MOTOR_CONTROL_STK_SIZE / 10u,
                   (CPU_STK_SIZE) APP_TASK_MOTOR_CONTROL_STK_SIZE,
                   (OS_MSG_QTY  ) 10u,
                   (OS_TICK     ) 0u,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&err);

    /*OSTaskCreate((OS_TCB     *)&AppTaskTimerControlTCB,
                   (CPU_CHAR   *)"Timer Control Task",
                   (OS_TASK_PTR ) AppTaskTimerControl,
                   (void       *) 0,
                   (OS_PRIO     ) APP_TASK_TIMER_CONTROL_PRIO,
                   (CPU_STK    *)&AppTaskTimerControlStk[0],
                   (CPU_STK_SIZE) APP_TASK_TIMER_CONTROL_STK_SIZE / 10u,
                   (CPU_STK_SIZE) APP_TASK_TIMER_CONTROL_STK_SIZE,
                   (OS_MSG_QTY  ) 10u,
                   (OS_TICK     ) 0u,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&err);*/
    
    OSTaskCreate((OS_TCB     *)&AppTaskInputMonitorTCB,
                   (CPU_CHAR   *)"Input Monitor Task",
                   (OS_TASK_PTR ) AppTaskInputMonitor,
                   (void       *) 0,
                   (OS_PRIO     ) APP_TASK_INPUT_MONITOR_PRIO,
                   (CPU_STK    *)&AppTaskInputMonitorStk[0],
                   (CPU_STK_SIZE) APP_TASK_INPUT_MONITOR_STK_SIZE / 10u,
                   (CPU_STK_SIZE) APP_TASK_INPUT_MONITOR_STK_SIZE,
                   (OS_MSG_QTY  ) 10u,
                   (OS_TICK     ) 0u,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&err);  
    
}

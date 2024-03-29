#include <includes.h>
#include "motor_lib.h"
#include "LED.h"

//#define BSP_PushButtonGetStatus(a) (0)
#define MOTOR_SRC 0
#define MONITOR_SRC 1
#define TIMER_SRC 2

#define IDLE 0
#define FINDING 1
#define CIRCLING 2
#define RETURN 3
#define STOP 4

#define STRAIGHT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3

#define SW1 1
#define SW2 2
#define BUMPL 1
#define BUMPR 2
#define MAXIMUM 20
static  OS_TCB       AppTaskStartTCB;
static  OS_TCB       AppTaskRobotControlTCB;
static  OS_TCB       AppTaskMotorControlTCB;
static  OS_TCB       AppTaskTimerControlTCB;
static  OS_TCB       AppTaskInputMonitorTCB;
static  OS_TCB       AppTaskLedControlTCB;

static  CPU_STK      AppTaskStartStk[APP_TASK_START_STK_SIZE];
static  CPU_STK      AppTaskRobotControlStk[APP_TASK_ROBOT_CONTROL_STK_SIZE];
static  CPU_STK      AppTaskMotorControlStk[APP_TASK_MOTOR_CONTROL_STK_SIZE];
static  CPU_STK      AppTaskTimerControlStk[APP_TASK_TIMER_CONTROL_STK_SIZE];
static  CPU_STK      AppTaskInputMonitorStk[APP_TASK_INPUT_MONITOR_STK_SIZE];
static  CPU_STK      AppTaskLedControlStk[APP_TASK_LED_CONTROL_STK_SIZE];

static  void  AppTaskStart        (void  *p_arg);
static  void  AppTaskRobotControl (void  *p_arg);
static  void  AppTaskMotorControl (void  *p_arg);
static  void  AppTaskTimerControl (void);
static void AppTaskLedControl (void *p_arg);
void  AppTaskInputMonitor (void  *p_arg);
static  void  AppTasksCreate (void);
void postToMotor(CPU_INT08U motor_dir ,CPU_INT08U motor_speed, CPU_INT16U motor_seg, OS_ERR* err);
void synchronousMove(CPU_INT08U motor_dir ,CPU_INT08U motor_speed, CPU_INT16U motor_seg, CPU_TS* ts, OS_ERR* err);
void postToTimer(CPU_INT16U msg_timer, OS_ERR* err);


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
    BSP_PushButtonsInit();
    motors_init();
    LEDsInit();
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
     CPU_INT08U sync;
     
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
       sync = msg_rx >> 31u;
       
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
          tx_src = MOTOR_SRC;
          
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

       tx_src = TIMER_SRC;
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
    
   (void)&p_arg;
    int i;
    CPU_INT08U input[4];
    CPU_INT08U counter[4];
    CPU_INT08U state[4];
    
    for(i=0; i<4; i++){
        input[i]=0;
        counter[i]=0;
        state[i]=0;
    }
    
    while (DEF_ON) {
      /* Delay for 5 milliseconds.    */
        OSTimeDlyHMSM(0u, 0u, 0u, 5u, 
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);

       /* Read the state of the switches. */
        input[0] = BSP_PushButtonGetStatus(SW1); //SW1 push button
        input[1] = BSP_PushButtonGetStatus(SW2); //SW2 push button
        input[2] = BSP_BumpSensorGetStatus(BUMPL); //Left bump sensor
        input[3] = BSP_BumpSensorGetStatus(BUMPR); //Right bump sensor
        
        for(i=0; i<4; i++){
            if(input[i]!=state[i])
                counter[i]=0;
            else if(counter[i] < MAXIMUM)
                counter[i]++;
            if(counter[i] >= MAXIMUM)
            {
                counter[i] = 0;
                msg_tx = ((MONITOR_SRC << 8u)|(i));
                OSTaskQPost(&AppTaskRobotControlTCB,
                        (CPU_INT16U *)msg_tx,
                        (OS_MSG_SIZE)sizeof(CPU_INT16U *),
                        (OS_OPT)OS_OPT_POST_FIFO,
                        (OS_ERR *)&err);
            }
        }//end_for
    }
}


static void AppTaskLedControl(void *p_arg)
{
    OS_ERR err;
    OS_MSG_SIZE size;
    CPU_TS ts;
    
    int led_en = 0;
    CPU_INT08U interval = 500;
    LEDsInit();
    while(1){
        if(led_en)
          LED_On(0);
        else
          LED_Off(0);
        led_en = ~led_en;
        OSTimeDlyHMSM(0u, 0u, 0u, interval, OS_OPT_TIME_HMSM_STRICT, &err);
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

    CPU_INT08U motor_speed;
    CPU_INT08U motor_dir;
    CPU_INT16U motor_seg;    

    CPU_INT08U state = IDLE;
    CPU_INT08U corner_count = 0;
    CPU_BOOLEAN turning = true;
    CPU_INT08U bump_state = 0;
    CPU_INT08U timer_state = 0;
    CPU_INT08U distance;
    CPU_INT08U last_distance;
    CPU_INT08U count_debug =0;
    char message[80];
    OSTaskSuspend(&AppTaskLedControlTCB, &err);
    srand(123456);
    while(1){
        msg_rx = (CPU_INT16U)OSTaskQPend((OS_TICK)0,  
                                       (OS_OPT)OS_OPT_PEND_BLOCKING,
                                       (OS_MSG_SIZE *)&size,
                                       (CPU_TS *)&ts,
                                       (OS_ERR *)&err);
        rx_val = msg_rx & 0xff;
        rx_src = (msg_rx >> 8u);
        count_debug++;
        switch(state){
        case IDLE:
            switch(rx_src){
            case MONITOR_SRC:
                if(rx_val == 1){
                    BSP_DisplayClear();
                    BSP_DisplayStringDraw("IDLE",10u,1u);
                    motor_dir  = STRAIGHT;
                    motor_speed = 80u; 
                    motor_seg = 100u;       // Tell the motors to run forward
                    postToMotor(motor_dir, motor_speed, motor_seg, &err);
                    state = FINDING;
                }
            }
            break;
        case FINDING:
            BSP_DisplayClear();
            BSP_DisplayStringDraw("PHONG NGUYEN",10u,1u);
            switch(rx_src){
            case MONITOR_SRC:
                distance = RoboStopNow();
                //DEBUG
                BSP_DisplayClear();
                snprintf(message, 80, "distance is %d", distance);
                BSP_DisplayStringDraw(message,10u,1u);
                //DEBUGEND
                state = CIRCLING;
                break;
            }
            break;
        case CIRCLING:
            OSTaskResume(&AppTaskLedControlTCB, &err);
            postToMotor(LEFT, motor_speed, RIGHT_ANGLE, &err);
            postToMotor(RIGHT, motor_speed, RIGHT_ANGLE, &err);
            postToMotor(RIGHT, motor_speed, RIGHT_ANGLE-1, &err);
            postToMotor(RIGHT, motor_speed, RIGHT_ANGLE, &err);
            postToMotor(RIGHT, motor_speed, RIGHT_ANGLE-1, &err);
            synchronousMove(LEFT, motor_speed, RIGHT_ANGLE, &ts, &err);
            state = STOP;
            break;
        case STOP:
            OSTaskSuspend(&AppTaskLedControlTCB, &err);
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

    OSTaskCreate((OS_TCB     *)&AppTaskTimerControlTCB,
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
                   (OS_ERR     *)&err);    
    
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
    OSTaskCreate((OS_TCB     *)&AppTaskLedControlTCB,
                   (CPU_CHAR   *)"LED Control Task",
                   (OS_TASK_PTR ) AppTaskLedControl,
                   (void       *) 0,
                   (OS_PRIO     ) APP_TASK_LED_CONTROL_PRIO,
                   (CPU_STK    *)&AppTaskLedControlStk[0],
                   (CPU_STK_SIZE) APP_TASK_LED_CONTROL_STK_SIZE / 10u,
                   (CPU_STK_SIZE) APP_TASK_LED_CONTROL_STK_SIZE,
                   (OS_MSG_QTY  ) 10u,
                   (OS_TICK     ) 0u,
                   (void       *) 0,
                   (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                   (OS_ERR     *)&err);
}

void postToMotor(CPU_INT08U motor_dir ,CPU_INT08U motor_speed, CPU_INT16U motor_seg, OS_ERR* err)
{
    CPU_INT32U msg_motor = ((motor_dir << 24u)|(motor_speed << 16u)|(motor_seg));
    OSTaskQPost(&AppTaskMotorControlTCB,
             (CPU_INT32U *)msg_motor,
             (OS_MSG_SIZE)sizeof(CPU_INT32U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)err);
}

void synchronousMove(CPU_INT08U motor_dir ,CPU_INT08U motor_speed, CPU_INT16U motor_seg, CPU_TS* ts, OS_ERR* err)
{
    CPU_INT32U msg_motor = ((motor_dir << 24u)|(motor_speed << 16u)|(motor_seg)|0x1<<31);
    OSTaskQPost(&AppTaskMotorControlTCB,
             (CPU_INT32U *)msg_motor,
             (OS_MSG_SIZE)sizeof(CPU_INT32U *),
             (OS_OPT)OS_OPT_POST_FIFO,
             (OS_ERR *)err);
    OSTaskSemPend(0,
                  OS_OPT_PEND_BLOCKING,
                  ts,
                  err);
}

void postToTimer(CPU_INT16U msg_timer, OS_ERR* err)
{
    OSTaskQPost(&AppTaskTimerControlTCB,
               (CPU_INT16U *)msg_timer,
               (OS_MSG_SIZE)sizeof(CPU_INT16U *),
               (OS_OPT)OS_OPT_POST_FIFO,
               (OS_ERR *)err);
}
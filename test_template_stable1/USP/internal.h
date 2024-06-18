 /*
 * internal.h
 *
 *  Created on: Apr 7, 2021
 *      Author: M3chD09
 */

#ifndef _INTERNAL_H_
#define _INTERNAL_H_

#include "can.h"
#include "usart.h"
#include "tim.h"

#include "SRML.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "BETAFPV.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
	
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8
/* HAL Handlers --------------------------------------------------------------*/
/* RTOS Resources ------------------------------------------------------------*/
/* Task */
extern TaskHandle_t Rx_Referee_Handle;
#if USE_SRML_FS_I6X
extern TaskHandle_t FS_I6X_Handle;
#endif // !USE_SRML_FS_I6X
/* Queues */
extern QueueHandle_t USART_TxPort;
extern QueueHandle_t USART_RxPort;
extern QueueHandle_t CAN1_TxPort;				
extern QueueHandle_t CAN1_RxPort;		
extern QueueHandle_t CAN2_TxPort;
extern QueueHandle_t CAN2_RxPort;

#if USE_SRML_DR16
extern QueueHandle_t DR16_QueueHandle;
#endif
extern QueueHandle_t BETAFPV_QueueHandle;
/* Semaphores */
/* mutex */
#if USE_SRML_DR16
extern SemaphoreHandle_t DR16_mutex;
#endif
extern SemaphoreHandle_t BETAFPV_mutex;
/* Mutexes */
/* Notifications */
/* Other Resources -----------------------------------------------------------*/
#if USE_SRML_VIRTUAL_COM
extern uint8_t VirtualCom_Rx_Buff[VIRTUALCOM_RX_BUFFER_SIZE]; // 虚拟串口缓存区
#endif

#if  USE_SRML_MPU6050
extern mpu_rec_s mpu_receive; //mpu6050传感器
#endif

#if  USE_SRML_DR16
extern DR16_Classdef DR16;		//遥控器DR16类
#endif
extern BETAFPV_Classdef BETAFPV;

#if USE_SRML_FS_I6X
extern FS_I6X_Classdef remote;
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
extern referee_Classdef Referee;//裁判系统类
#endif

extern Motor_C610 m2006;
extern Motor_C620 m3508;
extern Motor_GM6020 gm6020[2];

extern abstractMotor<Motor_C610> absM2006;
extern abstractMotor<Motor_C620> absM3508;
extern abstractMotor<Motor_GM6020> absM6020[2];

extern LkMotorBass lkmotor;

// can通信自回环测试结构体
#define CANTESTID 0x221
#pragma pack(1)
typedef struct _CanTestStructdef
{
	/*发射数据(pack1)*/
	uint16_t test1;	
	uint16_t test2;	
	uint16_t test3; 
	uint16_t test4;	
} CanTestStructdef;
#pragma pack()

extern CanTestStructdef can1def;
extern CanTestStructdef can2def;
extern CAN_COB can1Cob;
extern CAN_COB can2Cob;
/* Exported function declarations --------------------------------------------*/
void Service_Debug_Init(void);
void Service_Communication_Init(void);
void Service_Devices_Init(void);

void User_CAN1_RxCpltCallback(CAN_COB *CAN_RxCOB);
void User_CAN2_RxCpltCallback(CAN_COB *CAN_RxCOB);

extern User_Uart_Callback UART1_RxCpltCallback;
extern User_Uart_Callback UART2_RxCpltCallback;
extern User_Uart_Callback UART3_RxCpltCallback;
extern User_Uart_Callback UART4_RxCpltCallback;
extern User_Uart_Callback UART5_RxCpltCallback;
extern User_Uart_Callback UART6_RxCpltCallback;

#if USE_SRML_DR16
uint32_t DR16_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif
uint32_t BETAFPV_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);

#if USE_SRML_FS_I6X
uint32_t FS_I6X_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
uint32_t Referee_recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#if USE_SRML_VIRTUAL_COM
void User_VirtualComRecCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INTERNAL_H_ */

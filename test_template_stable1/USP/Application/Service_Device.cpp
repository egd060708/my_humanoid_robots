/**
 ******************************************************************************
 * @file   task.cpp
 * @brief  freertos task running file.
 ******************************************************************************
 * @note
 *  - Before running your devices, just do what you want ~ !
 *  - More devices or using other classification is decided by yourself ~ !
 ===============================================================================
								   Task List
 ===============================================================================
 * <table>
 * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
 * <tr><td>              <td>                  <td>                <td>
 * </table>
 *
*/

/* Includes ------------------------------------------------------------------*/
#include "internal.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t DjiMotor_Handle;
TaskHandle_t IMU_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t FS_I6X_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t BETAFPV_Handle;
TaskHandle_t VirtualCom_Handle;
TaskHandle_t Help_Handle;
/* Private function declarations ---------------------------------------------*/
void tskDjiMotor(void *arg);
void tskFS_I6X(void *arg);
void tskIMU(void *arg);
void tskDR16(void *arg);
void Rx_Referee(void *arg);
void tskBETAFPV(void *arg);
void tskVirtualCom(void *arg);
void tskHelp(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */
void Service_Devices_Init(void)
{
	xTaskCreate(tskDjiMotor, "App.Motor", Huge_Stack_Size, NULL, PriorityNormal, &DjiMotor_Handle);
	xTaskCreate(tskHelp, "App.Help", Huge_Stack_Size, NULL, PriorityNormal, &Help_Handle);

#if USE_SRML_MPU6050
	xTaskCreate(tskIMU, "App.IMU", Normal_Stack_Size, NULL, PrioritySuperHigh, &IMU_Handle);
#endif

#if USE_SRML_DR16
	xTaskCreate(tskDR16, "App.DR16", Small_Stack_Size, NULL, PrioritySuperHigh, &DR16_Handle);
#endif
	xTaskCreate(tskBETAFPV, "App.BETAFPV", Small_Stack_Size, NULL, PrioritySuperHigh, &BETAFPV_Handle);

#if USE_SRML_FS_I6X
	xTaskCreate(tskFS_I6X, "App.FS_I6X", Small_Stack_Size, NULL, PrioritySuperHigh, &FS_I6X_Handle);
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
#endif
	
#if USE_SRML_VIRTUAL_COM
//	xTaskCreate(tskVirtualCom, "App.VirtualCom", Normal_Stack_Size, NULL, PriorityHigh, &VirtualCom_Handle);
#endif
}

void tskHelp(void *arg)
{
	/*	pre load for task	*/
	rMiddleware.init(USART_TxPort,1);

	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	UBaseType_t uxHighWaterMark;
    
   // 检测任务使用前的堆栈情况                         
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 10);
		// 检测任务运行之后的堆栈剩余情况          
		humanLeg.State_Judge();
		humanLeg.State_Return();
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	}
}

/**
 * @brief <freertos> 大疆电机控制任务
 */
void tskDjiMotor(void *arg)
{
	/*	pre load for task	*/

	rMiddleware.jointInit();

	// vTaskDelay(10);
  // realjointMotor[HIP_ROLL].init(CAN1_TxPort);
  // realjointMotor[HIP_ROLL].setTorqueConstant(0.07);
  // realjointMotor[HIP_ROLL].setCurrent2ARatio(33.f / 2048.f);
	// vTaskDelay(2);
  // realjointMotor[HIP_ROLL].startMotor();
  // vTaskDelay(10);
  // realjointMotor[HIP_PITCH].init(CAN1_TxPort);
  // realjointMotor[HIP_PITCH].setTorqueConstant(0.22);
  // realjointMotor[HIP_PITCH].setCurrent2ARatio(33.f / 2048.f);
	// vTaskDelay(2);
  // realjointMotor[HIP_PITCH].startMotor();
  // vTaskDelay(10);
  // realjointMotor[HIP_YAW].init(CAN1_TxPort);
  // realjointMotor[HIP_YAW].setTorqueConstant(0.1);
  // realjointMotor[HIP_YAW].setCurrent2ARatio(33.f / 2048.f);
	// vTaskDelay(2);
  // realjointMotor[HIP_YAW].startMotor();
  // vTaskDelay(10);
  // realjointMotor[KNEE].init(CAN2_TxPort);
  // realjointMotor[KNEE].setTorqueConstant(0.22);
  // realjointMotor[KNEE].setCurrent2ARatio(33.f / 2048.f);
	// vTaskDelay(2);
  // realjointMotor[KNEE].startMotor();
  // vTaskDelay(10);
  // realjointMotor[ANKLE].init(CAN2_TxPort);
  // realjointMotor[ANKLE].setTorqueConstant(0.07);
  // realjointMotor[ANKLE].setCurrent2ARatio(33.f / 2048.f);
	// vTaskDelay(2);
  // realjointMotor[ANKLE].startMotor();

	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	UBaseType_t uxHighWaterMark;
    
   // 检测任务使用前的堆栈情况                         
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 10);
		// 检测任务运行之后的堆栈剩余情况          
    
		humanLeg.State_Data_Update();
		// lkmotor.iqCloseControl_Current(0);
		// realjointMotor[0].iqCloseControl_Current(0);
		// realjointMotor[1].iqCloseControl_Current(0);
		// realjointMotor[2].iqCloseControl_Current(0);
		// realjointMotor[3].iqCloseControl_Current(0);
		// realjointMotor[4].iqCloseControl_Current(0);
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );

		/* dji电机库使用示例 */
		// 1. 电机控制
		// m2006.Out = 1000;
		// // 2. 将电机输出数据打包成can消息队列
		// MotorMsgPack(Tx_Buff, m2006, m3508, gm6020);
		// // 3. 发送can队列，根据电机的发射帧id选择需要发送的数据包
		// xQueueSend(CAN1_TxPort,&Tx_Buff.Id200,0);
		// // 也可以2、3合并用MotorCanSend(),但只能发送一个can包,不同can包的电机不能一次性全丢进去，注意加命名空间
		// motor_dji_send::MotorMsgSend(CAN1_TxPort, gm6020);


		// /* 抽象电机库使用示例 */
		// // 1. 电机控制
		// absM2006.setMotorCurrentOut(1000);
		// absM3508.setMotorCurrentOut(1000);
		// absM6020[0].setMotorCurrentOut(1000);
		// absM6020[0].setMotorCurrentOut(1000);
		// // 2. 将电机输出数据打包成can消息队列
		// MotorMsgPack(Tx_Buff, absM2006, absM3508, absM6020);
		// // 3. 发送can队列，根据电机的发射帧id选择需要发送的数据包
		// xQueueSend(CAN1_TxPort,&Tx_Buff.Id200,0);
		// // 也可以2、3合并用MotorCanSend(),但只能发送一个can包，且抽象电机无法用数组发送
		// MotorMsgSend(CAN1_TxPort, absM2006);


	}
}

#if USE_SRML_MPU6050
/**
 * @brief MPU6050读取数据
 */
void tskIMU(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 2);
//		/*	读取MPU6050数据	*/
//		vTaskSuspendAll();		//挂起其他任务，防止被打断
//		taskDISABLE_INTERRUPTS();//关闭中断，若使用中断关闭，请确保SRML定时器的中断不受FreeRTOS管辖
//		dmp_read_data(&mpu_receive);
//		taskENABLE_INTERRUPTS();
//		xTaskResumeAll();
	}
}
#endif

/**
 *	@brief	BETAFPV data receive task
 */
void tskBETAFPV(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	BETAFPV.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		xSemaphoreTake(BETAFPV_mutex, portMAX_DELAY);
		/*	等待数据	*/
		if (xQueueReceive(BETAFPV_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Read Message
			BETAFPV.DataCapture((uint8_t *)Rx_Package.address);
		}
		/*	检测遥控器连接 */
		BETAFPV.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (BETAFPV.GetStatus() != ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */
			xSemaphoreGive(BETAFPV_mutex);
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
		xSemaphoreGive(BETAFPV_mutex);
	}
}

#if USE_SRML_VIRTUAL_COM
void tskVirtualCom(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	/* 测试数据包 */
	uint8_t testbuffer[] = "STM32 Virtual Com test1!\n";
	for(;;)
    {
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);
		VirtualComTransmitData(testbuffer, sizeof(testbuffer));
	}
}
#endif

#if USE_SRML_DR16
/**
 *	@brief	Dr16 data receive task
 */
void tskDR16(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	DR16.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		xSemaphoreTake(DR16_mutex, portMAX_DELAY);
		/*	等待数据	*/
		if (xQueueReceive(DR16_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Read Message
			DR16.DataCapture((DR16_DataPack_Typedef *)Rx_Package.address);
		}
		/*	检测遥控器连接 */
		DR16.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (DR16.GetStatus() != DR16_ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */
			xSemaphoreGive(DR16_mutex);
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
		xSemaphoreGive(DR16_mutex);
	}
}
#endif

#if USE_SRML_REFEREE
/**
 * @brief  接受裁判系统数据
 * @param  None.
 * @return None.
 */
void Rx_Referee(void *arg)
{
	/* Preoad for task */
	USART_COB *referee_pack;
	TickType_t xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *)&referee_pack, portMAX_DELAY) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t *)referee_pack->address, referee_pack->len);
		}
	}
}
#endif

#if USE_SRML_FS_I6X
/**
 *	@brief	FS_I6X data receive task
 */
void tskFS_I6X(void *arg)
{
	/* Cache for Task */
	/* Pre-Load for task */
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		/*	等待数据	*/
		if (ulTaskNotifyTake(pdTRUE, 100) != 0)
		{
			// Read Message
			remote.DataProcess();
		}
		/*	检测遥控器连接 */
		remote.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (remote.GetStatus() != ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */;
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
	}
}
#endif // !USE_SRML_FS_I6X

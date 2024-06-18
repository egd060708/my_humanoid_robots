/**
  ***********************************************************************************
  * @file   : Service_Communication.cpp
  * @brief  : Communication support file.This file provides access ports to interface
  *           with connected devices.
  ***********************************************************************************
                                 ##### Port List #####
  =================================================================================
  |Port Name     Physical-Layer     Data-Link-Layer    Application-Layer    Number
  |————————————————————————————————————————
  |EXAMPLE_Port       CAN1               CAN                CUSTOM            0
  |CAN2_Port          CAN2               CAN                Custom            1
  |EBUG_Port         USART1             Custom              Custom            2
  |USART2_Port       USART2              DBUS               DJI-DR16          3
  *
**/
/* Includes ------------------------------------------------------------------*/
#include "internal.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if USE_SRML_CAN
TaskHandle_t CAN1SendPort_Handle;
TaskHandle_t CAN2SendPort_Handle;
TaskHandle_t CAN1ReceivePort_Handle;
TaskHandle_t CAN2ReceivePort_Handle;
# endif
#if USE_SRML_UART
TaskHandle_t UartTransmitPort_Handle;
TaskHandle_t UartReceivePort_Handle;
#endif
/* Private function declarations ---------------------------------------------*/
#if USE_SRML_CAN
void Task_CAN1Transmit(void *arg);
void Task_CAN2Transmit(void *arg);
void Task_CAN1Receive(void *arg);
void Task_CAN2Receive(void *arg);
# endif
#if USE_SRML_UART
void Task_UsartTransmit(void *arg);
void Task_UsartReceive(void *arg);
#endif
/**
 * @brief  Initialization of communication service
 * @param  None.
 * @return None.
 */
void Service_Communication_Init(void)
{
#if USE_SRML_CAN
  xTaskCreate(Task_CAN1Transmit, "Com.CAN1 TxPort", Small_Stack_Size, NULL, PriorityRealtime, &CAN1SendPort_Handle);
  xTaskCreate(Task_CAN2Transmit, "Com.CAN2 TxPort", Small_Stack_Size, NULL, PriorityRealtime, &CAN2SendPort_Handle);
  xTaskCreate(Task_CAN1Receive, "Com.CAN1 RxPort", Small_Stack_Size, NULL, PriorityRealtime, &CAN1ReceivePort_Handle);
  xTaskCreate(Task_CAN2Receive, "Com.CAN2 RxPort", Small_Stack_Size, NULL, PriorityRealtime, &CAN2ReceivePort_Handle);
# endif
  /* USART Management */
#if USE_SRML_UART
  xTaskCreate(Task_UsartReceive, "Com.Usart RxPort", Small_Stack_Size, NULL, PriorityRealtime, &UartReceivePort_Handle);
  xTaskCreate(Task_UsartTransmit, "Com.Usart TxPort", Small_Stack_Size, NULL, PriorityRealtime, &UartTransmitPort_Handle);
#endif
}

/*----------------------------------------------- CAN Manager ---------------------------------------------*/
/*Task Define ---------------------------*/
#if USE_SRML_CAN
/*Function Prototypes--------------------*/
/**
 * @brief  Tasks for CAN Management.
 * @param  None.
 * @return None.
 */
void Task_CAN1Transmit(void *arg)
{
  /* Cache for Task */
  uint8_t free_can_mailbox;
  CAN_COB CAN_TxMsg;
  /* Pre-Load for task */

  /* Infinite loop */

  for (;;)
  {
    /* CAN1 Send Port */
    if (xQueueReceive(CAN1_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdPASS)
    {
      free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
      /* Avoid the unused warning*/
      UNUSED(&free_can_mailbox);
      CANx_SendData(1, &CAN_TxMsg);
    }
  }
}

/*
 * can2 transmit
 */
void Task_CAN2Transmit(void *arg)
{
  /* Cache for Task */
  uint8_t free_can_mailbox;
  CAN_COB CAN_TxMsg;
  /* Pre-Load for task */

  /* Infinite loop */

  for (;;)
  {
    /* CAN1 Send Port */
    if (xQueueReceive(CAN2_TxPort, &CAN_TxMsg, portMAX_DELAY) == pdPASS)
    {
      free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
      /* Avoid the unused warning*/
      UNUSED(&free_can_mailbox);
      CANx_SendData(2, &CAN_TxMsg);
    }
  }
}

/*
 * can1 receive
 */
void Task_CAN1Receive(void *arg)
{
  CAN_COB CAN_RxCOB;

  /* Infinite loop */
  for (;;)
  {
    /* update motor data from CAN1_RxPort */
    if (xQueueReceive(CAN1_RxPort, &CAN_RxCOB, portMAX_DELAY) == pdPASS)
    {
      lkmotor.update(CAN_RxCOB.ID, CAN_RxCOB.Data);
      // 这里调用自定义的电机can数据更新接口
      // 注意！！！如果接口里函数嵌套、临时变量很多，请考虑该任务栈大小

      // 更新电机数据，如
      // if (m3508.update(CAN_RxCOB.ID, CAN_RxCOB.Data))
      // {
      // }
      // else if (m2006.update(CAN_RxCOB.ID, CAN_RxCOB.Data))
      // {
      // }
      // else
      // {
      //   for (auto &m : absM6020)
      //   {
      //     m.update(CAN_RxCOB.ID, CAN_RxCOB.Data);
      //   }
      // }
    }
  }
}

/*
 * can2 receive
 */
void Task_CAN2Receive(void *arg)
{
  CAN_COB CAN_RxCOB;

  /* Infinite loop */
  for (;;)
  {
    /* update motor data from CAN1_RxPort */
    if (xQueueReceive(CAN2_RxPort, &CAN_RxCOB, portMAX_DELAY) == pdPASS)
    {
			lkmotor.update(CAN_RxCOB.ID, CAN_RxCOB.Data);
      // 这里调用自定义的电机can数据更新接口
      // 注意！！！如果接口里函数嵌套、临时变量很多，请考虑该任务栈大小

      // 更新电机数据，如
      // if (pitchmotor.update(CAN_RxCOB.ID, CAN_RxCOB.Data));
      // else if (yawmotor.update(CAN_RxCOB.ID, CAN_RxCOB.Data));
//      if(CAN_RxCOB.ID == CANTESTID){
//        memcpy((uint8_t *)&can2def, CAN_RxCOB.Data, CAN_RxCOB.DLC);
//      }
    }
  }
}

/**
 * @brief  Callback function in CAN Interrupt
 * @param  None.
 * @return None.
 */
void User_CAN1_RxCpltCallback(CAN_COB *CAN_RxCOB)
{
  BaseType_t xHigherPriorityTaskWoken;
  if (CAN1_RxPort != NULL)
  {
    xQueueSendFromISR(CAN1_RxPort, CAN_RxCOB, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void User_CAN2_RxCpltCallback(CAN_COB *CAN_RxCOB)
{
  BaseType_t xHigherPriorityTaskWoken;
  if (CAN2_RxPort != NULL)
  {
    xQueueSendFromISR(CAN2_RxPort, CAN_RxCOB, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
#endif
/*---------------------------------------------- USART Manager --------------------------------------------*/
/*Task Define ---------------------------*/
#if USE_SRML_UART
/*Function Prototypes--------------------*/
/**
* @brief  Tasks for USART Management.
          Attention:In this version we passing the pointer of data not copying
          data itself and we only have one buff zone, so user need to process
          the data as soon as possible in case of over-write.
* @param  None.
* @return None.
*/
void Task_UsartTransmit(void *arg)
{
  /* Cache for Task */
  USART_COB Usart_TxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for (;;)
  {
    /* Usart Receive Port*/
    if (xQueueReceive(USART_TxPort, &Usart_TxCOB, portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch (Usart_TxCOB.port_num)
      {
      default:
        break;
      }
      /* User Code End Here ---------------------------------*/
      SRML_UART_Transmit_DMA(&Usart_TxCOB);
    }
  }
}

void Task_UsartReceive(void *arg)
{
  /* Cache for Task */
  USART_COB Usart_RxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for (;;)
  {
    /* Usart Receive Port*/
    if (xQueueReceive(USART_RxPort, &Usart_RxCOB, portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      // 注意！！！如果接口里函数嵌套、临时变量很多，请考虑该任务栈大小
      switch (Usart_RxCOB.port_num)
      {
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        break;
      case 6:
        break;
      default:
        break;
      }
      /* User Code End Here ---------------------------------*/
    }
  }
}

template <uint8_t uart_id>
uint32_t User_UART_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);

User_Uart_Callback UART1_RxCpltCallback = User_UART_RxCpltCallback<1>;
User_Uart_Callback UART2_RxCpltCallback = User_UART_RxCpltCallback<2>;
User_Uart_Callback UART3_RxCpltCallback = User_UART_RxCpltCallback<3>;
User_Uart_Callback UART4_RxCpltCallback = User_UART_RxCpltCallback<4>;
User_Uart_Callback UART5_RxCpltCallback = User_UART_RxCpltCallback<5>;
User_Uart_Callback UART6_RxCpltCallback = User_UART_RxCpltCallback<6>;
/**
 * @brief  Callback function in USART Interrupt
 * @param  Recv_Data		接收数组
 *	@param	ReceiveLen	接收数据长度
 * @return None.
 */
template <uint8_t uart_id>
uint32_t User_UART_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  USART_COB Usart_RxCOB;
  BaseType_t xHigherPriorityTaskWoken;
  // Send To UART Receive Queue
  if (USART_RxPort != NULL)
  {
    Usart_RxCOB.port_num = uart_id;
    Usart_RxCOB.len = ReceiveLen;
    Usart_RxCOB.address = Recv_Data;
    xQueueSendFromISR(USART_RxPort, &Usart_RxCOB, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  return 0;
}

uint32_t BETAFPV_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  USART_COB Usart_RxCOB;
  BaseType_t xHigherPriorityTaskWoken;
  // Send To UART Receive Queue
  if (BETAFPV_QueueHandle != NULL)
  {
    Usart_RxCOB.len = ReceiveLen;
    Usart_RxCOB.address = Recv_Data;
    xQueueSendFromISR(BETAFPV_QueueHandle, &Usart_RxCOB, &xHigherPriorityTaskWoken);
  }
  return 0;
}

#if USE_SRML_DR16
uint32_t DR16_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  USART_COB Usart_RxCOB;
  BaseType_t xHigherPriorityTaskWoken;
  // Send To UART Receive Queue
  if (DR16_QueueHandle != NULL)
  {
    Usart_RxCOB.len = ReceiveLen;
    Usart_RxCOB.address = Recv_Data;
    xQueueSendFromISR(DR16_QueueHandle, &Usart_RxCOB, &xHigherPriorityTaskWoken);
  }
  return 0;
}
#endif

#if USE_SRML_FS_I6X
uint32_t FS_I6X_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  if(ReceiveLen >= 25 && FS_I6X_Handle != nullptr)
  {
    BaseType_t xHigherPriorityTaskWoken;
    remote.DataCapture(Recv_Data);
    vTaskNotifyGiveFromISR(FS_I6X_Handle, &xHigherPriorityTaskWoken);
  }
  return 0;
}
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
uint32_t Referee_recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  BaseType_t xHigherPriorityTaskWoken;
  static USART_COB referee_pack;

  if (Rx_Referee_Handle != NULL && ReceiveLen < 256)
  {
    referee_pack.address = Recv_Data;
    referee_pack.len = ReceiveLen;
    xTaskNotifyFromISR(Rx_Referee_Handle, (uint32_t)&referee_pack, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  }
  return 0;
}
#endif

#if USE_SRML_VIRTUAL_COM
void User_VirtualComRecCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
}
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

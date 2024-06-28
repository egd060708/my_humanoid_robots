#include "robot_middleware.h"
#include "internal.h"
#include "Upper_Public.h"

/**
 * @brief 构造函数
 *
 */
robotMiddleware::robotMiddleware(BETAFPV_Classdef *_remote) : remote(_remote)
{
  jointPID[0].SetPIDParam(40, 5, 0, 10, 120);
  jointPID[1].SetPIDParam(30, 5, 0, 8, 100);
  jointPID[2].SetPIDParam(30, 5, 0, 8, 80);
  jointPID[3].SetPIDParam(30, 5, 0, 8, 80);
  jointPID[4].SetPIDParam(40, 5, 0, 10, 120);
  jointPID[0].Out_Max = 20;
  jointPID[1].Out_Max = 20;
  jointPID[2].Out_Max = 20;
  jointPID[3].Out_Max = 20;
  jointPID[4].Out_Max = 20;
}

/**
 * @brief 中间层初始化，传入串口队列以及通信串口号
 *
 * @param _USART_TxPort
 * @param _port_num
 */
void robotMiddleware::init(QueueHandle_t _Usart_TxPort, uint8_t _port_num)
{
  // 串口队列
  Usart_TxPort = _Usart_TxPort;
  // 串口包
  Usart_TxCOB.port_num = _port_num;
  Usart_TxCOB.address = (uint8_t *)&(this->sendSlavePack);
  Usart_TxCOB.len = sizeof(slaveCom_s);
}

/**
 * @brief 电机初始化，设置电机参数并打开电机
 *
 * @param _motor
 */
void robotMiddleware::jointInit()
{
  vTaskDelay(10);
  realjointMotor[HIP_ROLL].init(CAN1_TxPort);
  realjointMotor[HIP_ROLL].setTorqueConstant(0.07);
  realjointMotor[HIP_ROLL].setCurrent2ARatio(33.f / 2048.f);
  vTaskDelay(2);
  realjointMotor[HIP_ROLL].startMotor();
  vTaskDelay(10);
  realjointMotor[HIP_PITCH].init(CAN1_TxPort);
  realjointMotor[HIP_PITCH].setTorqueConstant(0.22);
  realjointMotor[HIP_PITCH].setCurrent2ARatio(33.f / 2048.f);
  vTaskDelay(2);
  realjointMotor[HIP_PITCH].startMotor();
  vTaskDelay(10);
  realjointMotor[HIP_YAW].init(CAN1_TxPort);
  realjointMotor[HIP_YAW].setTorqueConstant(0.1);
  realjointMotor[HIP_YAW].setCurrent2ARatio(33.f / 2048.f);
  vTaskDelay(2);
  realjointMotor[HIP_YAW].startMotor();
  vTaskDelay(10);
  realjointMotor[KNEE].init(CAN2_TxPort);
  realjointMotor[KNEE].setTorqueConstant(0.22);
  realjointMotor[KNEE].setCurrent2ARatio(33.f / 2048.f);
  vTaskDelay(2);
  realjointMotor[KNEE].startMotor();
  vTaskDelay(10);
  realjointMotor[ANKLE].init(CAN2_TxPort);
  realjointMotor[ANKLE].setTorqueConstant(0.07);
  realjointMotor[ANKLE].setCurrent2ARatio(33.f / 2048.f);
  vTaskDelay(2);
  realjointMotor[ANKLE].startMotor();

  // 设置初始化角度
  if (L_LEG == 1)
  {
    offsetAngle[0] = 171.88;
    offsetAngle[1] = 187.40;
    offsetAngle[2] = 115.22;
    offsetAngle[3] = 235.67;
    offsetAngle[4] = 291.66;
    // 设置初始化极性
    direction[0] = 1;
    direction[1] = -1;
    direction[2] = 1;
    direction[3] = 1;
    direction[4] = 1;
  }
}

/**
 * @brief 获取电机起点
 *
 */
void robotMiddleware::jointGetStartAngle()
{
  for (int i = 0; i < 5; i++)
  {
    if (realjointMotor[i].getData().singleAngle != 0 && is_start[i] == false)
    {
      startAngle[i] = realjointMotor[i].getData().singleAngle;
      is_start[i] = true;
    }
  }
}

/**
 * @brief 处理上位机发来的数据
 *
 * @param _recData
 */
void robotMiddleware::processRecHost(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  hostCom_s getHost;
  memcpy(&getHost, Recv_Data, sizeof(ReceiveLen));
  if (getHost.id == HOSTCOM_ID_L)
  {
    // 如果是第一块板子那么直接copy
    recHostPack = getHost;
    host_link_count = 0;
  }
  else if (getHost.id == HOSTCOM_ID_R)
  {
    // 如果是第二块板子那么直接转发
    memcpy(&sendSlavePack, Recv_Data, sizeof(sendSlavePack.jointAngle) + sizeof(sendSlavePack.endPoint));
    sendSlavePack.ctrl_enable = ctrl_enable;
    sendSlaveFromISR();
    host_link_count = 0;
  }
}

/**
 * @brief 发送到上位机
 *
 */
void robotMiddleware::sendHost()
{
  VirtualComTransmitData((uint8_t *)&(this->sendHostPack), sizeof(this->sendHostPack));
}

/**
 * @brief 处理另一个从机发来的数据
 *
 * @param _recData
 */
void robotMiddleware::processRecSlave(slaveCom_s *_recData)
{
  memcpy(&recSlavePack, _recData, sizeof(recSlavePack));
  slave_link_count = 0;
}

/**
 * @brief 发送到另一个从机
 *
 */
void robotMiddleware::sendSlave()
{
  xQueueSend(Usart_TxPort, &Usart_TxCOB, 0);
}

void robotMiddleware::sendSlaveFromISR()
{
  xQueueSendFromISR(Usart_TxPort, &Usart_TxCOB, 0);
}

/**
 * @brief  电机数据更新函数
 * @note
 * @param   接收的CAN包
 * @return
 * @retval  None
 */
void robotMiddleware::Motor_Rec(CAN_COB *CAN_RxMsg)
{
  // for (auto motor : realjointMotor)
  // {
  //   update(CAN_RxMsg->ID, CAN_RxMsg->Data);
  // }
  for (int i = 0; i < 5; i++)
  {
    realjointMotor[i].update(CAN_RxMsg->ID, CAN_RxMsg->Data);
  }
}

/**
 * @brief 执行关节位控
 *
 * @param _angle (radian)
 * @param _time (s)
 */
void robotMiddleware::ctrlActuate(const float _angle[5], float _time)
{
  _time = upper::constrain(_time, 10., 0.1);
  for (int i = 0; i < 5; i++)
  {
    ctrlAngle[i] = _angle[i] / PI * 180 / direction[i] + offsetAngle[i];
    jointPID[i].Target = ctrlAngle[i];
    jointPID[i].Current = realjointMotor[i].getData().singleAngle;
    // if(jointPID[i].Target != lastTarget[i])
    // {
    //   jointPID[i].Out_Max = abs(jointPID[i].Target - lastTarget[i])/_time*60.0;
    // }
    jointPID[i].Adjust();
    lastTarget[i] = jointPID[i].Target;
  }
  realjointMotor[0].speedControl(jointPID[0].Out);
  realjointMotor[1].speedControl(jointPID[1].Out);
  realjointMotor[2].speedControl(jointPID[2].Out);
  realjointMotor[3].speedControl(jointPID[3].Out);
  realjointMotor[4].speedControl(jointPID[4].Out);
}

/**
 * @brief 取消任何控制
 *
 */
void robotMiddleware::noCtrl()
{
  // 全部输出0电流
  // for (auto i : realjointMotor)
  // {
  //   i.setMotorCurrentOut(0);
  //   vTaskDelay(2);
  // }
  // for(int i=0;i<5;i++){
  //   realjointMotor[i].iqCloseControl_Current(0);
  //   // realjointMotor[i].readTotalAngle();
  //   // realjointMotor[i].readPid();
  // }
  realjointMotor[0].iqCloseControl_Current(0);
  realjointMotor[1].iqCloseControl_Current(0);
  realjointMotor[2].iqCloseControl_Current(0);
  realjointMotor[3].iqCloseControl_Current(0);
  realjointMotor[4].iqCloseControl_Current(0);
}

/**
 * @brief 通信连接检查
 *
 */
void robotMiddleware::link_check()
{
  is_host_connet = (host_link_count >= host_link_threshold) ? false : true;
  is_slave_connet = (slave_link_count >= slave_link_threshold) ? false : true;
  host_link_count++;
  slave_link_count++;
  if (host_link_count > 2 * host_link_threshold)
  {
    host_link_count = 2 * host_link_threshold;
  }
  if (slave_link_count > 2 * slave_link_threshold)
  {
    slave_link_count = 2 * slave_link_threshold;
  }
}

/**
 * @brief 根据当前状态判断是否使能控制
 *
 */
void robotMiddleware::judge_ctrl_enable()
{
  // 这里相当于对两个板子做了分化处理
  if (L_LEG == 0)
  {
    // slave
    ctrl_enable = recSlavePack.ctrl_enable;
  }
  else
  {
    if (remote->GetStatus() && remote->getData().SA == SW_UP && remote->getData().SD == SW_UP)
    {
      ctrl_enable = true;
    }
    else
    {
      ctrl_enable = false;
    }
  }
}

/**
 * @brief 返回当前腿的状态(发送出去)
 *
 */
void robotMiddleware::returnDataSend()
{
  if (L_LEG == 0)
  {
    // slave
    sendSlave();
  }
  else
  {
    // 先把自己的发了
    sendHostPack.id = HOSTCOM_ID_L;
    sendHost();
    // 再把另一个板子的数据发过去
    memcpy(&sendHostPack, &recSlavePack, sizeof(sendHostPack.jointAngle) + sizeof(sendHostPack.endPoint));
    sendHostPack.id = HOSTCOM_ID_R;
    sendHost();
  }
}

/**
 * @brief 更新当前腿的状态
 *
 * @param joint
 * @param endPoint
 */
void robotMiddleware::returnStateData(const float joint[5], const float endPoint[6])
{
  if (L_LEG == 0)
  {
    for (int i = 0; i < 6; i++)
    {
      if (i < 5)
      {
        sendSlavePack.jointAngle[i] = joint[i];
      }
      sendSlavePack.endPoint[i] = endPoint[i];
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      if (i < 5)
      {
        sendHostPack.jointAngle[i] = joint[i];
      }
      sendHostPack.endPoint[i] = endPoint[i];
    }
  }
}

#include "robot_middleware.h"
#include "internal.h"

/**
 * @brief 构造函数
 *
 */
robotMiddleware::robotMiddleware(BETAFPV_Classdef *_remote) : remote(_remote)
{
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
void robotMiddleware::jointInit(LkMotorBass _motor[5])
{
  vTaskDelay(10);
  jointMotor[HIP_ROLL].motor.init(CAN1_TxPort);
  jointMotor[HIP_ROLL].motor.setTorqueConstant(0.07);
  jointMotor[HIP_ROLL].motor.setCurrent2ARatio(33.f / 2048.f);
  jointMotor[HIP_ROLL].motor.startMotor();
  vTaskDelay(10);
  jointMotor[HIP_PITCH].motor.init(CAN1_TxPort);
  jointMotor[HIP_PITCH].motor.setTorqueConstant(0.22);
  jointMotor[HIP_PITCH].motor.setCurrent2ARatio(33.f / 2048.f);
  jointMotor[HIP_PITCH].motor.startMotor();
  vTaskDelay(10);
  jointMotor[HIP_YAW].motor.init(CAN1_TxPort);
  jointMotor[HIP_YAW].motor.setTorqueConstant(0.1);
  jointMotor[HIP_YAW].motor.setCurrent2ARatio(33.f / 2048.f);
  jointMotor[HIP_YAW].motor.startMotor();
  vTaskDelay(10);
  jointMotor[KNEE].motor.init(CAN2_TxPort);
  jointMotor[KNEE].motor.setTorqueConstant(0.22);
  jointMotor[KNEE].motor.setCurrent2ARatio(33.f / 2048.f);
  jointMotor[KNEE].motor.startMotor();
  vTaskDelay(10);
  jointMotor[ANKLE].motor.init(CAN2_TxPort);
  jointMotor[ANKLE].motor.setTorqueConstant(0.07);
  jointMotor[ANKLE].motor.setCurrent2ARatio(33.f / 2048.f);
  jointMotor[ANKLE].motor.startMotor();
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
  if(getHost.id == HOSTCOM_ID_L){
    // 如果是第一块板子那么直接copy
    recHostPack = getHost;
		host_link_count = 0;
  }
  else if(getHost.id == HOSTCOM_ID_R){
    // 如果是第二块板子那么直接转发
    memcpy(&sendSlavePack, Recv_Data, sizeof(sendSlavePack.jointAngle)+sizeof(sendSlavePack.endPoint));
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
  for (auto motor : jointMotor)
  {
    motor.update(CAN_RxMsg->ID, CAN_RxMsg->Data);
  }
}

/**
 * @brief 执行关节位控
 *
 * @param _angle
 */
void robotMiddleware::ctrlActuate(const float _angle[5], const float _speed[5])
{
  for (int i = 0; i < 5; i++)
  {
    jointMotor[i].setMotorAngleWithSpeed(_angle[i], _speed[i]);
  }
}

/**
 * @brief 取消任何控制
 *
 */
void robotMiddleware::noCtrl()
{
  // 全部输出0电流
  for (auto i : jointMotor)
  {
    i.setMotorCurrentOut(0);
  }
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
	if(host_link_count > 2*host_link_threshold){
		host_link_count = 2*host_link_threshold;
	}
	if(slave_link_count > 2*slave_link_threshold){
		slave_link_count = 2*slave_link_threshold;
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
    if (is_host_connet && remote->GetStatus() && remote->getData().SA == SW_UP && remote->getData().SD == SW_UP)
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
void robotMiddleware::returnDataSend(){
  if(L_LEG == 0){
    // slave
    sendSlave();
  }
  else{
    // 先把自己的发了
    sendHostPack.id = HOSTCOM_ID_L;
    sendHost();
    // 再把另一个板子的数据发过去
    memcpy(&sendHostPack, &recSlavePack, sizeof(sendHostPack.jointAngle)+sizeof(sendHostPack.endPoint));
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
void robotMiddleware::returnStateData(const float joint[5],const float endPoint[6]){
  if(L_LEG == 0){
    for(int i=0;i<6;i++){
      if(i<5){
        sendSlavePack.jointAngle[i] = joint[i];
      }
      sendSlavePack.endPoint[i] = endPoint[i];
    }
  }
  else{
    for(int i=0;i<6;i++){
      if(i<5){
        sendHostPack.jointAngle[i] = joint[i];
      }
      sendHostPack.endPoint[i] = endPoint[i];
    }
  }
}

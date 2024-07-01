#include "robot_middleware.h"
#include "internal.h"
#include "Upper_Public.h"

/**
 * @brief 构造函数
 *
 */
robotMiddleware::robotMiddleware(BETAFPV_Classdef *_remote) : remote(_remote)
{
  // jointPID[0].SetPIDParam(40, 5, 0, 10, 120);
  // jointPID[1].SetPIDParam(30, 5, 0, 8, 100);
  // jointPID[2].SetPIDParam(30, 5, 0, 8, 80);
  // jointPID[3].SetPIDParam(30, 5, 0, 8, 80);
  // jointPID[4].SetPIDParam(40, 5, 0, 10, 120);
  jointPID[0].SetPIDParam(40, 0, 0, 0, 120);
  jointPID[1].SetPIDParam(30, 0, 0, 0, 100);
  jointPID[2].SetPIDParam(30, 0, 0, 0, 80);
  jointPID[3].SetPIDParam(30, 0, 0, 0, 80);
  jointPID[4].SetPIDParam(40, 0, 0, 0, 120);
  jointPID[0].Out_Max = 20;
  jointPID[1].Out_Max = 20;
  jointPID[2].Out_Max = 20;
  jointPID[3].Out_Max = 20;
  jointPID[4].Out_Max = 20;

  cmd_endPoint[2] = -0.63;
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
  else
  {
    offsetAngle[0] = 89.43;
    offsetAngle[1] = 156.24;
    offsetAngle[2] = 149.67;
    offsetAngle[3] = 269.18;
    offsetAngle[4] = 283.64;
    // 设置初始化极性
    direction[0] = 1;
    direction[1] = -1;
    direction[2] = -1;
    direction[3] = -1;
    direction[4] = -1;
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
  memcpy(&recHostPack, Recv_Data, sizeof(hostCom_rs));
  // 放入自己的cmd里面
  memcpy(cmd_endPoint, &recHostPack.r_EndPoint, sizeof(cmd_endPoint));
  upper::constrain(cmd_endPoint[0], 0.1);
  upper::constrain(cmd_endPoint[1], 0.0001);
  upper::constrain(cmd_endPoint[2], -0.65, -0.58);
  upper::constrain(cmd_endPoint[3], 0.0001);
  upper::constrain(cmd_endPoint[4], 0.0001);
  upper::constrain(cmd_endPoint[5], 0.0001);
  cmd_dt = recHostPack.r_dt;
  // 更新通信检测计数
  host_link_count = 0;
  // 放入发送到从机的数据里面
  memcpy(&sendSlavePack, &recHostPack.l_EndPoint, sizeof(sendSlavePack.endPoint) + sizeof(sendSlavePack.dt));
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
  if (L_LEG == 1)
  {
    memcpy(&recSlavePack, _recData, sizeof(recSlavePack));
    memcpy(cmd_endPoint, &recSlavePack.endPoint, sizeof(cmd_endPoint));
    upper::constrain(cmd_endPoint[0], 0.1);
    upper::constrain(cmd_endPoint[1], 0.0001);
    upper::constrain(cmd_endPoint[2], -0.65, -0.58);
    upper::constrain(cmd_endPoint[3], 0.0001);
    upper::constrain(cmd_endPoint[4], 0.0001);
    upper::constrain(cmd_endPoint[5], 0.0001);
    cmd_dt = recSlavePack.dt;
  }
  else
  {
    memcpy(&recSlavePack, _recData, sizeof(recSlavePack));
  }

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
    if (jointPID[i].Target != lastTarget[i])
    {
      jointPID[i].Out_Max = upper::constrain(abs(jointPID[i].Target - lastTarget[i]) / 360. / _time * 60.0, 100);
    }
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
    if (remote->GetStatus() && remote->getData().SA == SW_UP)
    {
      preCtrl_enable = true;
      if (remote->getData().SD == SW_UP)
      {
        ctrl_enable = true;
        sendHostPack.flag = true;
      }
      else
      {
        ctrl_enable = false;
        sendHostPack.flag = false;
      }
    }
    else
    {
      preCtrl_enable = false;
      ctrl_enable = false;
      sendHostPack.flag = false;
    }
  }
  else
  {
    // slave
    ctrl_enable = recSlavePack.ctrl_enable;
    preCtrl_enable = recSlavePack.preCtrl_enable;
  }
  if (L_LEG == 0)
  {
    sendSlavePack.ctrl_enable = ctrl_enable;
    sendSlavePack.preCtrl_enable = preCtrl_enable;
    sendSlave();
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
    // 前面已经把自己的更新过了，这里就直接加入另一个腿的部分发送即可
    memcpy(&sendHostPack.l_EndPoint, &recSlavePack, sizeof(sendHostPack.l_EndPoint));
    sendHost();
  }
  else
  {
    // slave
    sendSlave();
  }
}

/**
 * @brief 更新当前腿的状态
 *
 * @param joint
 * @param endPoint
 */
void robotMiddleware::returnStateData(const float endPoint[6])
{
  if (L_LEG == 0)
  {
    for (int i = 0; i < 6; i++)
    {
      sendHostPack.r_EndPoint[i] = endPoint[i];
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      sendSlavePack.endPoint[i] = endPoint[i];
    }
  }
  returnDataSend();
}

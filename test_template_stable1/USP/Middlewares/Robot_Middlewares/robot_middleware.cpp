#include "robot_middleware.h"

/**
 * @brief 构造函数
 * 
 */
robotMiddleware::robotMiddleware(){

}

/**
 * @brief 中间层初始化，传入串口队列以及通信串口号
 * 
 * @param _USART_TxPort 
 * @param _port_num 
 */
void robotMiddleware::init(QueueHandle_t _Usart_TxPort, uint8_t _port_num[2]){
  // 串口队列
  Usart_TxPort = _Usart_TxPort;
  // 串口包
  Usart_TxCOB[0].port_num = _port_num[0];
  Usart_TxCOB[0].address = (uint8_t*)&(this->sendHostPack);
  Usart_TxCOB[0].len = sizeof(hostCom_s);
  Usart_TxCOB[1].port_num = _port_num[1];
  Usart_TxCOB[1].address = (uint8_t*)&(this->sendSlavePack);
  Usart_TxCOB[1].len = sizeof(slaveCom_s);
}

/**
 * @brief 电机初始化，设置电机参数并打开电机
 * 
 * @param _motor 
 */
void robotMiddleware::jointInit(LkMotorBass _motor[5]){
  vTaskDelay(10);
  jointMotor[HIP_ROLL].motor.init(CAN1_TxPort);
  jointMotor[HIP_ROLL].motor.setTorqueConstant(0.07);
  jointMotor[HIP_ROLL].motor.setCurrent2ARatio(33.f/2048.f);
  jointMotor[HIP_ROLL].motor.startMotor();
  vTaskDelay(10);
  jointMotor[HIP_PITCH].motor.init(CAN1_TxPort);
  jointMotor[HIP_PITCH].motor.setTorqueConstant(0.22);
  jointMotor[HIP_PITCH].motor.setCurrent2ARatio(33.f/2048.f);
  jointMotor[HIP_PITCH].motor.startMotor();
  vTaskDelay(10);
  jointMotor[HIP_YAW].motor.init(CAN1_TxPort);
  jointMotor[HIP_YAW].motor.setTorqueConstant(0.1);
  jointMotor[HIP_YAW].motor.setCurrent2ARatio(33.f/2048.f);
  jointMotor[HIP_YAW].motor.startMotor();
  vTaskDelay(10);
  jointMotor[KNEE].motor.init(CAN2_TxPort);
  jointMotor[KNEE].motor.setTorqueConstant(0.22);
  jointMotor[KNEE].motor.setCurrent2ARatio(33.f/2048.f);
  jointMotor[KNEE].motor.startMotor();
  vTaskDelay(10);
  jointMotor[ANKLE].motor.init(CAN2_TxPort);
  jointMotor[ANKLE].motor.setTorqueConstant(0.07);
  jointMotor[ANKLE].motor.setCurrent2ARatio(33.f/2048.f);
  jointMotor[ANKLE].motor.startMotor();
}

/**
 * @brief 处理上位机发来的数据
 * 
 * @param _recData 
 */
void robotMiddleware::processRecHost(hostCom_s* _recData){
  memcpy(&recHostPack,_recData,sizeof(recHostPack));
}

/**
 * @brief 发送到上位机
 * 
 */
void robotMiddleware::sendHost(){
  xQueueSend(Usart_TxPort,&Usart_TxCOB,0);
}

/**
 * @brief 处理另一个从机发来的数据
 * 
 * @param _recData 
 */
void robotMiddleware::processRecSlave(slaveCom_s* _recData){
  memcpy(&recSlavePack,_recData,sizeof(recSlavePack));
}

/**
 * @brief 发送到另一个从机
 * 
 */
void robotMiddleware::sendSlave(){
  xQueueSend(Usart_TxPort,&Usart_TxCOB,0);
}


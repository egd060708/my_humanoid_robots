/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MF9025_v2.cpp
  * @author  余俊晖 (2460857175@qq.com)
  * @brief   Code for Motor-MF9025V2 driver
  * @date    2022-10-31
  * @version 1.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    		<th>Description
  * <tr><td>2022-10-31  <td> 1.0     <td>JunhuiYu     <td>Creator
  * </table>
  *
  ==============================================================================
                            How to use this driver
  ==============================================================================
    @note

  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have any
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if USE_SRML_MF9025_V2

#include "Drivers/Devices/MF9025_V2/MF9025_v2.h"

/* define ------------------------------------------------------------------*/
// #define RATIO_RECEIVE_VOLTAGE 0.1f
// #define RATIO_RECEIVE_ANGLE 0.01f
// #define RATIO_ENCODER_TO_ANGLE 360 / 65536.f
// #define RATIO_CURRENT_TO_A 16.5f / 2048.f 
// //16T是0.32f
// #define TORQUE_CONSTANT 0.32f
// 35T是0.81f
// #define TORQUE_CONSTANT 0.81f

/*  没有用空格分开的define，接收的数据处理都是相同的  */
#define ORDER_READ_PID 0x30
#define ORDER_WRITE_PID_TO_RAM 0x31
#define ORDER_WRITE_PID_TO_ROM 0x32

#define ORDER_READ_ACCEL 0x33
#define ORDER_WRITE_ACCEL_TO_RAM 0x34

#define ORDER_READ_ENCODER 0x90

#define ORDER_WRITE_ENCODER_OFFSET 0x91
#define ORDER_WRITE_NOW_ENCODER_AS_OFFSET 0x19

#define ORDER_READ_TOTAL_ANGLE 0x92

#define ORDER_READ_SINGLE_TURN_ANGLE 0x94

#define ORDER_READ_STATE1_AND_ERROR 0x9A //  返回“温度”、“电压”、“错误标志位”
#define ORDER_CLEAN_ERROR 0x9B

#define ORDER_READ_STATE3 0x9D

#define ORDER_READ_STATE2 0x9C
#define ORDER_TORQUE_OPEN_LOOP 0xA0
#define ORDER_TORQUE_CLOSE_LOOP 0xA1
#define ORDER_SPEED_CLOSE_LOOP 0xA2
#define ORDER_ANGLE_CLOSE_LOOP_1 0xA3
#define ORDER_ANGLE_CLOSE_LOOP_2 0xA4
#define ORDER_ANGLE_CLOSE_LOOP_3 0xA5
#define ORDER_ANGLE_CLOSE_LOOP_4 0xA6
#define ORDER_ANGLE_CLOSE_LOOP_5 0xA7
#define ORDER_ANGLE_CLOSE_LOOP_6 0xA8

#define ORDER_CLOSE_MOTOR 0x80 //  返回空值，不用处理
#define ORDER_STOP_MOTOR 0x81  //  返回空值，不用处理
#define ORDER_START_MOTOR 0x88 //  返回空值，不用处理
/* Private function declarations ---------------------------------------------*/
/*
 * @brief 	将电机数据整合入电机can通信数据包
 * @param		&motor_msg			电机can通信数据包
 * @param 	&motor					电机类
 * @return 	motor_msg				电机can通信数据包
 */
CAN_COB &MotorMsgPack(CAN_COB &motor_msg, LkMotorBass &motor)
{
    int16_t constrain_Out = motor.getData().targetCurrent;
    constrain_Out = std_lib::constrain(constrain_Out, static_cast<int16_t>(-motor.MAX_CURRENT()), motor.MAX_CURRENT());

    motor_msg.Data[motor.getData().motorId * 2 - 2] = (constrain_Out >> 8) & 0xff;
    motor_msg.Data[motor.getData().motorId * 2 - 1] = constrain_Out & 0xff;
    return motor_msg;
}
/**
 * @brief 电机类构造函数
 * 
 * @param _id 电机id（1-8）
 */
LkMotorBass::LkMotorBass(uint8_t _id)
{
  dataPool.motorId = _id;
  canSendStruct.ID = getComId();
  canSendStruct.DLC = 8;
}
/**
 * @brief 电机类构造函数（无需认为计算ID号）
 * 
 * @param id1 电机背面序号为 1 的id调节开关是否为ON，是则传入非0数字（最好为1，因为是bool类型，传入非0都会变成1），否则为0
 * @param id2 电机背面序号为 2 的id调节开关是否为ON，是则传入非0数字（最好为1，因为是bool类型，传入非0都会变成1），否则为0
 * @param id3 电机背面序号为 3 的id调节开关是否为ON，是则传入非0数字（最好为1，因为是bool类型，传入非0都会变成1），否则为0
 * 示例：若1-3的id设置开关均为off，则变量构造函数传入MotorMF9025v2Classdef motor(0, 0, 0);
 *      若1为ON，2为OFF，3为ON，则变量构造函数传入MotorMF9025v2Classdef motor(1, 0, 1);
 */
LkMotorBass::LkMotorBass(bool id1, bool id2, bool id3)
{
	dataPool.motorId = 1 + (uint8_t)id1+ 2*(uint8_t)id2 + 4*(uint8_t)id3;
	canSendStruct.ID = getComId();
	canSendStruct.DLC = 8;
}
/**
 * @brief 电机类成员初始化
 * 
 * @param _sendQueue 传入发送队列
 * 不能把传入发送队列放在构造函数中，因为作为全局变量声明，是在队列创建之前的，故只会传入NULL的队列句柄
 */
void LkMotorBass::init(QueueHandle_t _sendQueue)
{
  sendQueue = _sendQueue;
  iqCloseControl(0);
  vTaskDelay(1);
  writeEncoderOffset(dataPool.encoderOffset);
  vTaskDelay(1);
  readPid();
}

/**
 * @brief 写入PID参数到RAM
 * 
 * @param _iqKp 转矩环Kp
 * @param _iqKi 转矩环Ki
 * @param _angleKp 角度环Kp
 * @param _angleKi 角度环Ki
 * @param _speedKp 速度环Kp
 * @param _speedKi 速度环Ki
 */
void LkMotorBass::writePidToRAM(int8_t _iqKp, int8_t _iqKi, int8_t _angleKp, int8_t _angleKi, int8_t _speedKp, int8_t _speedKi)
{
  canSendStruct.Data[0] = ORDER_WRITE_PID_TO_RAM;
  canSendStruct.Data[2] = _angleKp;
  canSendStruct.Data[3] = _angleKi;
  canSendStruct.Data[4] = _speedKp;
  canSendStruct.Data[5] = _speedKi;
  canSendStruct.Data[6] = _iqKp;
  canSendStruct.Data[7] = _iqKi;
  sendData();
}
/**
 * @brief 写入PID参数到ROM
 * 
 * @param _iqKp 转矩环Kp
 * @param _iqKi 转矩环Ki
 * @param _angleKp 角度环Kp
 * @param _angleKi 角度环Ki
 * @param _speedKp 速度环Kp
 * @param _speedKi 速度环Ki
 */
void LkMotorBass::writePidToROM(int8_t _iqKp, int8_t _iqKi, int8_t _angleKp, int8_t _angleKi, int8_t _speedKp, int8_t _speedKi)
{
  canSendStruct.Data[0] = ORDER_WRITE_PID_TO_ROM;
  canSendStruct.Data[2] = _angleKp;
  canSendStruct.Data[3] = _angleKi;
  canSendStruct.Data[4] = _speedKp;
  canSendStruct.Data[5] = _speedKi;
  canSendStruct.Data[6] = _iqKp;
  canSendStruct.Data[7] = _iqKi;
  sendData();
}
/**
 * @brief 写入特定编码器值作为零点
 * 
 * @param _offset 零点编码器值
 */
void LkMotorBass::writeEncoderOffset(uint16_t _offset)
{
  canSendStruct.Data[0] = ORDER_WRITE_ENCODER_OFFSET;
  canSendStruct.Data[6] = _offset & 0xff;
  canSendStruct.Data[7] = (_offset >> 8) & 0xff;
  sendData();
}
/**
 * @brief 将当前编码器值作为零点
 * 
 */
void LkMotorBass::writeNowEncoderAsOffset()
{
  canSendStruct.Data[0] = ORDER_WRITE_NOW_ENCODER_AS_OFFSET;
  sendData();
}
/**
 * @brief 写入加速度到RAM
 * 
 * @param _accel 待写入的加速度，单位为1dps/LSB
 */
void LkMotorBass::writeAccelToRAM(int32_t _accel)
{
  canSendStruct.Data[0] = ORDER_WRITE_ACCEL_TO_RAM;
  canSendStruct.Data[4] = _accel & 0xff;
  canSendStruct.Data[5] = (_accel >> 8) & 0xff;
  canSendStruct.Data[6] = (_accel >> 16) & 0xff;
  canSendStruct.Data[7] = (_accel >> 24) & 0xff;
  sendData();
}
/**
 * @brief 转矩开环控制模式(仅在MS系列电机上实现)
 * 
 * @param _targetTorque 目标扭矩，单位为N*m
 */
void LkMotorBass::iqOpenControl(float _targetTorque)
{
  int16_t targetCurrent = _targetTorque / TORQUE_CONSTANT / RATIO_CURRENT_TO_A;
  canSendStruct.Data[0] = ORDER_TORQUE_OPEN_LOOP;
  canSendStruct.Data[4] = targetCurrent & 0xff;
  canSendStruct.Data[5] = (targetCurrent >> 8) & 0xff;
  sendData();
}
/**
 * @brief 开环力矩控制，输入目标电流值(仅在MS系列电机上实现)
 * 主要用于速度环在MCU上运算时，向电机下发控制量，但是封库时测试用起来效果并不佳
 * @param _targetCurrent 目标电流值，在-2048到2048之间，对应-16.5A到16.5A
 */
void LkMotorBass::iqOpenControl_Current(int16_t _targetCurrent)
{
  _targetCurrent = std_lib::constrain(_targetCurrent, (int16_t)-2048, (int16_t)2048);
  canSendStruct.Data[0] = ORDER_TORQUE_OPEN_LOOP;
  canSendStruct.Data[4] = _targetCurrent & 0xff;
  canSendStruct.Data[5] = (_targetCurrent >> 8) & 0xff;
  sendData();
}
/**
 * @brief 转矩闭环控制模式（仅在MF和MG上实现）
 * 
 * @param _targetTorque 目标扭矩，单位为N*m
 */
void LkMotorBass::iqCloseControl(float _targetTorque)
{
  int16_t targetCurrent = _targetTorque / TORQUE_CONSTANT / RATIO_CURRENT_TO_A;
  canSendStruct.Data[0] = ORDER_TORQUE_CLOSE_LOOP;
  canSendStruct.Data[4] = targetCurrent & 0xff;
  canSendStruct.Data[5] = (targetCurrent >> 8) & 0xff;
  sendData();
}
/**
 * @brief 闭环力矩控制，输入目标电流值（仅在MF和MG上实现）
 * 主要用于速度环在MCU上运算时，向电机下发控制量，但是封库时测试用起来效果并不佳
 * @param _targetCurrent 目标电流值，在-2048到2048之间，对应-16.5A到16.5A
 */
void LkMotorBass::iqCloseControl_Current(int16_t _targetCurrent)
{
  _targetCurrent = std_lib::constrain(_targetCurrent, (int16_t)-2048, (int16_t)2048);
  canSendStruct.Data[0] = ORDER_TORQUE_CLOSE_LOOP;
  canSendStruct.Data[4] = _targetCurrent & 0xff;
  canSendStruct.Data[5] = (_targetCurrent >> 8) & 0xff;
  sendData();
}
/**
 * @brief 速度闭环控制模式
 * 
 * @param _targetRPM 目标转速，单位为RPM
 */
void LkMotorBass::speedControl(float _targetRPM)
{
  int32_t _targetSpeed = _targetRPM * 6 * 100;
  canSendStruct.Data[0] = ORDER_SPEED_CLOSE_LOOP;
  canSendStruct.Data[4] = _targetSpeed & 0xff;
  canSendStruct.Data[5] = (_targetSpeed >> 8) & 0xff;
  canSendStruct.Data[6] = (_targetSpeed >> 16) & 0xff;
  canSendStruct.Data[7] = (_targetSpeed >> 24) & 0xff;
  sendData();
}
/**
 * @brief 总角度闭环模式1
 * 
 * @param _targetAngle 目标总角度，单位为°
 */
void LkMotorBass::angleTotalControl_1(float _targetAngle)
{
  int32_t targetAngle = _targetAngle * 100;
  canSendStruct.Data[0] = ORDER_ANGLE_CLOSE_LOOP_1;
  canSendStruct.Data[4] = targetAngle & 0xff;
  canSendStruct.Data[5] = (targetAngle >> 8) & 0xff;
  canSendStruct.Data[6] = (targetAngle >> 16) & 0xff;
  canSendStruct.Data[7] = (targetAngle >> 24) & 0xff;
  sendData();
}
/**
 * @brief 总角度闭环模式2
 * 
 * @param _targetAngle 目标总角度，单位为°
 * @param _speedLimit 速度限制，单位为RPM
 */
void LkMotorBass::angleTotalControl_2(float _targetAngle, float _speedLimit)
{
  int32_t targetAngle = _targetAngle * 100;
  int16_t speedLimit = _speedLimit * 6;
  canSendStruct.Data[0] = ORDER_ANGLE_CLOSE_LOOP_2;
  canSendStruct.Data[2] = speedLimit & 0xff;
  canSendStruct.Data[3] = (speedLimit >> 8) & 0xff;
  canSendStruct.Data[4] = targetAngle & 0xff;
  canSendStruct.Data[5] = (targetAngle >> 8) & 0xff;
  canSendStruct.Data[6] = (targetAngle >> 16) & 0xff;
  canSendStruct.Data[7] = (targetAngle >> 24) & 0xff;
  sendData();
}
/**
 * @brief 单圈角度闭环模式1
 * 
 * @param _targetAngle 目标总角度，单位为°
 */
void LkMotorBass::angleSingleControl_1(float _targetAngle)
{
  int16_t targetAngle = _targetAngle * 100;
  canSendStruct.Data[0] = ORDER_ANGLE_CLOSE_LOOP_3;
  canSendStruct.Data[4] = targetAngle & 0xff;
  canSendStruct.Data[5] = (targetAngle >> 8) & 0xff;
  sendData();
}
/**
 * @brief 单圈角度闭环模式2
 * 
 * @param _targetAngle 目标总角度，单位为°
 * @param _speedLimit 速度限制，单位为RPM
 */
void LkMotorBass::angleSingleControl_2(float _targetAngle, float _speedLimit)
{
  int16_t speedLimit = _speedLimit * 6;
  int16_t targetAngle = _targetAngle * 100;
  canSendStruct.Data[0] = ORDER_ANGLE_CLOSE_LOOP_4;
  canSendStruct.Data[2] = speedLimit & 0xff;
  canSendStruct.Data[3] = (speedLimit >> 8) & 0xff;
  canSendStruct.Data[4] = targetAngle & 0xff;
  canSendStruct.Data[5] = (targetAngle >> 8) & 0xff;
  sendData();
}
/**
 * @brief 增量式角度闭环模式1
 * 
 * @param _targetAngle 目标总角度，单位为°
 */
void LkMotorBass::angleIncrementControl_1(float _targetAngle)
{
  int32_t targetAngle = _targetAngle * 100;
  canSendStruct.Data[0] = ORDER_ANGLE_CLOSE_LOOP_5;
  canSendStruct.Data[4] = targetAngle & 0xff;
  canSendStruct.Data[5] = (targetAngle >> 8) & 0xff;
  canSendStruct.Data[6] = (targetAngle >> 16) & 0xff;
  canSendStruct.Data[7] = (targetAngle >> 24) & 0xff;
  sendData();
}
/**
 * @brief 增量式角度闭环模式2
 * 
 * @param _targetAngle 目标总角度，单位为°
 * @param _speedLimit 速度限制，单位为RPM
 */
void LkMotorBass::angleIncrementControl_2(float _targetAngle, float _speedLimit)
{
  int32_t targetAngle = _targetAngle * 100;
  int16_t speedLimit = _speedLimit * 6;
  canSendStruct.Data[0] = ORDER_ANGLE_CLOSE_LOOP_6;
  canSendStruct.Data[2] = speedLimit & 0xff;
  canSendStruct.Data[3] = (speedLimit >> 8) & 0xff;
  canSendStruct.Data[4] = targetAngle & 0xff;
  canSendStruct.Data[5] = (targetAngle >> 8) & 0xff;
  canSendStruct.Data[6] = (targetAngle >> 16) & 0xff;
  canSendStruct.Data[7] = (targetAngle >> 24) & 0xff;
  sendData();
}
/**
 * @brief 设置目标电流（多电机模式使用）
 * 
 * @param _current 目标电流，在±2000
 */
void LkMotorBass::setTatgetCurrent(int16_t _current)
{
  _current = std_lib::constrain(_current, static_cast<int16_t>(-1 * MAX_CURRENT()), MAX_CURRENT());
  dataPool.targetCurrent = _current;
};
/**
 * @brief 清除电机错误标志位
 * 
 */
void LkMotorBass::cleanErrorState()
{
  canSendStruct.Data[0] = ORDER_CLEAN_ERROR;
  sendData();
};
/**
 * @brief 关闭电机
 * 
 */
void LkMotorBass::closeMotor()
{
  canSendStruct.Data[0] = ORDER_CLOSE_MOTOR;
  sendData();
};
/**
 * @brief 停止电机
 * 
 */
void LkMotorBass::stopMotor()
{
  canSendStruct.Data[0] = ORDER_STOP_MOTOR;
  sendData();
};
/**
 * @brief 开启电机
 * 
 */
void LkMotorBass::startMotor()
{
  canSendStruct.Data[0] = ORDER_START_MOTOR;
  sendData();
};
/**
 * @brief 电机回调数据包处理函数
 * 
 * @param canRecID 接收CAN的ID
 * @param can_rx_data can接收数组
 */
bool LkMotorBass::update(uint32_t canRecID, uint8_t can_rx_data[])
{
  if(canRecID != getComId())
    return 0;
  switch (can_rx_data[0])
  {
  case ORDER_READ_STATE2:
  case ORDER_TORQUE_CLOSE_LOOP:
  case ORDER_SPEED_CLOSE_LOOP:
  case ORDER_ANGLE_CLOSE_LOOP_1:
  case ORDER_ANGLE_CLOSE_LOOP_2:
  case ORDER_ANGLE_CLOSE_LOOP_3:
  case ORDER_ANGLE_CLOSE_LOOP_4:
  case ORDER_ANGLE_CLOSE_LOOP_5:
  case ORDER_ANGLE_CLOSE_LOOP_6:
    updateMotorState2(can_rx_data);
    break;
  case ORDER_READ_TOTAL_ANGLE:
    updateTotalAngle(can_rx_data);
    break;
  case ORDER_READ_SINGLE_TURN_ANGLE:
    updateSingleAngle(can_rx_data);
    break;
  case ORDER_READ_ACCEL:
  case ORDER_WRITE_ACCEL_TO_RAM:
    updateAccel(can_rx_data);
    break;
  case ORDER_READ_ENCODER:
    updateEncoder(can_rx_data);
    break;
  case ORDER_READ_PID:
  case ORDER_WRITE_PID_TO_RAM:
  case ORDER_WRITE_PID_TO_ROM:
    updatePid(can_rx_data);
    break;
  case ORDER_WRITE_ENCODER_OFFSET:
  case ORDER_WRITE_NOW_ENCODER_AS_OFFSET:
    updateEncoderOffset(can_rx_data);
    break;
  case ORDER_READ_STATE1_AND_ERROR:
  case ORDER_CLEAN_ERROR:
    updateMotorState1(can_rx_data);
    break;
  case ORDER_READ_STATE3:
    updateMotorState3(can_rx_data);
    break;
  default:
    break;
  }
  return 1;
}

void LkMotorBass::setTorqueConstant(float _val)
{
  this->TORQUE_CONSTANT = _val;
}

void LkMotorBass::setCurrent2ARatio(float _val)
{
  this->RATIO_CURRENT_TO_A = _val;
}

void LkMotorBass::updateMotorState2(uint8_t can_rx_data[])
{
  dataPool.temperature = can_rx_data[1];
  dataPool.current = *reinterpret_cast<int16_t *>(&can_rx_data[2]) * RATIO_CURRENT_TO_A;
  dataPool.speed = *reinterpret_cast<int16_t *>(&can_rx_data[4]) * 60 / 360.f;
  dataPool.encoder = *reinterpret_cast<int16_t *>(&can_rx_data[6]);

  if (angle_offset_state)
  {
    last_angle = dataPool.singleAngle;
    dataPool.singleAngle = dataPool.encoder * RATIO_ENCODER_TO_ANGLE;
    if (dataPool.singleAngle - last_angle > 180)
    {
      dataPool.round_cnt--;
      dataPool.totalAngleLocal += dataPool.singleAngle - last_angle - 360;
    }
    else if (dataPool.singleAngle - last_angle < -180)
    {
      dataPool.round_cnt++;
      dataPool.totalAngleLocal += dataPool.singleAngle - last_angle + 360;
    }
    else
    {
      dataPool.totalAngleLocal += dataPool.singleAngle - last_angle;
    }
  }
  else
  {
    angle_offset_state = true;
    dataPool.singleAngle = dataPool.encoder * RATIO_ENCODER_TO_ANGLE;
  }
}

void LkMotorBass::updateAccel(uint8_t can_rx_data[])
{
  dataPool.accel = *reinterpret_cast<int32_t *>(&can_rx_data[4]);
}

void LkMotorBass::updateEncoder(uint8_t can_rx_data[])
{
  dataPool.encoder = *reinterpret_cast<int16_t *>(&can_rx_data[2]);
  dataPool.encoderRaw = *reinterpret_cast<int16_t *>(&can_rx_data[4]);
  dataPool.encoderOffset = *reinterpret_cast<int16_t *>(&can_rx_data[6]);
}

void LkMotorBass::updateEncoderOffset(uint8_t can_rx_data[])
{
  dataPool.encoderOffset = *reinterpret_cast<int16_t *>(&can_rx_data[6]);
}

void LkMotorBass::updateSingleAngle(uint8_t can_rx_data[])
{
  uint8_t temp[4] = {can_rx_data[4], can_rx_data[5], can_rx_data[6], can_rx_data[7]};
  //dataPool.singleAngle = *reinterpret_cast<int32_t *>(&can_rx_data[4]) * RATIO_RECEIVE_ANGLE; //会进入硬件中断HardFault_Handler,原因未知
  dataPool.singleAngle = *reinterpret_cast<int32_t *>(temp) * RATIO_RECEIVE_ANGLE;
}

void LkMotorBass::updateTotalAngle(uint8_t can_rx_data[])
{
  dataPool.totalAngle = *reinterpret_cast<int32_t *>(&can_rx_data[1]) * RATIO_RECEIVE_ANGLE;
}

void LkMotorBass::updatePid(uint8_t can_rx_data[])
{
  dataPool.anglePidKp = can_rx_data[2];
  dataPool.anglePidKi = can_rx_data[3];
  dataPool.speedPidKp = can_rx_data[4];
  dataPool.speedPidKi = can_rx_data[5];
  dataPool.iqPidKp = can_rx_data[6];
  dataPool.iqPidKi = can_rx_data[7];
};

void LkMotorBass::updateMotorState1(uint8_t can_rx_data[])
{
  dataPool.temperature = can_rx_data[1];
  dataPool.voltage = *reinterpret_cast<uint16_t *>(&can_rx_data[3]) * RATIO_RECEIVE_VOLTAGE;
  dataPool.errorState = can_rx_data[7];
}

void LkMotorBass::updateMotorState3(uint8_t can_rx_data[])
{
  dataPool.temperature = can_rx_data[1];
  dataPool.iA = *reinterpret_cast<int16_t *>(&can_rx_data[2]) / 64.f;
  dataPool.iB = *reinterpret_cast<int16_t *>(&can_rx_data[4]) / 64.f;
  dataPool.iC = *reinterpret_cast<int16_t *>(&can_rx_data[6]) / 64.f;
}
/**
 * @brief 读取pid参数
 * 
 */
void LkMotorBass::readPid()
{
  canSendStruct.Data[0] = ORDER_READ_PID;
  sendData();
};
/**
 * @brief 读取加速度
 * 
 */
void LkMotorBass::readAccel()
{
  canSendStruct.Data[0] = ORDER_READ_ACCEL;
  sendData();
};
/**
 * @brief 读取编码器值
 * 
 */
void LkMotorBass::readEncoder()
{
  canSendStruct.Data[0] = ORDER_READ_ENCODER;
  sendData();
};
/**
 * @brief 读取总已转角度
 * 
 */
void LkMotorBass::readTotalAngle()
{
  canSendStruct.Data[0] = ORDER_READ_TOTAL_ANGLE;
  sendData();
};
/**
 * @brief 读取单圈角度
 * 
 */
void LkMotorBass::readSingleAngle()
{
  canSendStruct.Data[0] = ORDER_READ_SINGLE_TURN_ANGLE;
  sendData();
};
/**
 * @brief 读取电机标志位1与错误标志位
 * 
 */
void LkMotorBass::readMotorState1_errorState()
{
  canSendStruct.Data[0] = ORDER_READ_STATE1_AND_ERROR;
  sendData();
};
/**
 * @brief 读取电机标志位2
 * 
 */
void LkMotorBass::readMotorState2()
{
  canSendStruct.Data[0] = ORDER_READ_STATE2;
  sendData();
};
/**
 * @brief 读取电机标志位3
 * 
 */
void LkMotorBass::readMotorState3()
{
  canSendStruct.Data[0] = ORDER_READ_STATE3;
  sendData();
};
/* function prototypes -------------------------------------------------------*/
#endif /* USE_SRML_MF9025_V2 */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

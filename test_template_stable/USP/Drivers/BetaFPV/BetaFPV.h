#pragma once

#ifdef __cplusplus

#include "srml_std_lib.h"
#include "Drivers/Devices/Remote_Public.h"

#define CRC8POLY 0xD5
#define Ignore_Limit 0.05 /*<! 线性死区,手柄或鼠标的归一化后的绝对值小于此值时自动视为0 */

/**
  @brief BETAFPV数据包内容
*/
#pragma pack(1)
typedef struct _BETAFPV_BaseData_Typedef
{
  uint8_t device_addr : 8;
  uint8_t frame_size : 8;
  uint8_t type : 8;
  uint16_t ch0 : 11; //表示该元素占用内存为11位
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
} BETAFPV_BaseData_Typedef;
#pragma pack()

#pragma pack(1)
typedef struct _BETAFPV_Data_Typedef
{
  float RX_Norm, RY_Norm, LX_Norm, LY_Norm; /*<! 两个摇杆四个方向归一化的值*/
  uint8_t SA,SB,SC,SD;/*<! 四个拨档开关值*/
} BETAFPV_Data_Typedef;
#pragma pack()

class BETAFPV_Classdef
{
private:
  uint32_t dt = 0;
  uint32_t last_check_time;                 /*<! 上一次在线检测时间*/
  LinkageStatus_Typedef Status;             /*<! 连接状态 */
  BETAFPV_BaseData_Typedef BaseData;        /*<! 原始数据包*/
  BETAFPV_Data_Typedef Data;                /*<! 处理后的数据包*/

  uint8_t _lut[256];                            /*<! CRC8校验数组 */
  void crc8init(uint8_t poly);                  /*<! CRC8初始化 */
  uint8_t crc8calc(uint8_t *data, uint8_t len); /*<! CRC8计算 */

public:
  BETAFPV_Classdef()
  {
    crc8init(CRC8POLY); /*初始化CRC*/
  }
  void DataCapture(uint8_t *captureData);

  /*返回基础数据*/
  BETAFPV_BaseData_Typedef &getBaseData(void);
  /*返回处理过后的数据*/
  BETAFPV_Data_Typedef &getData(void);

  /*连接状态相关操作*/
  void Check_Link(uint32_t current_check_time);
  LinkageStatus_Typedef GetStatus(void);
};

inline void BETAFPV_Classdef::DataCapture(uint8_t *captureData)
{

  //来自代码段2
  uint8_t len = captureData[1];
  //来自代码段2
  uint8_t inCrc = captureData[2 + len - 1];
  uint8_t crc = crc8calc(&captureData[2], len - 1);
  /*设置在线，开始再次检测连接*/
  last_check_time = 0;
  Status = ESTABLISHED;
  if (inCrc == crc && captureData[2] == 22){
    BaseData = *(BETAFPV_BaseData_Typedef *)captureData;
  }
  
  /*通道值归一化处理*/
  Data.RX_Norm = std_lib::DeadZone_Process((float)(BaseData.ch0 - 992) / 820.f, -Ignore_Limit, Ignore_Limit, 0);
  Data.RY_Norm = std_lib::DeadZone_Process((float)(BaseData.ch1 - 992) / 820.f, -Ignore_Limit, Ignore_Limit, 0);
  Data.LY_Norm = std_lib::DeadZone_Process((float)(BaseData.ch2 - 992) / 820.f, -Ignore_Limit, Ignore_Limit, 0);
  Data.LX_Norm = std_lib::DeadZone_Process((float)(BaseData.ch3 - 992) / 820.f, -Ignore_Limit, Ignore_Limit, 0);

  /*拨档值处理*/
  switch(BaseData.ch5){
    case 0x0BF:
      Data.SB = SW_DOWN;
      break;
    case 0x3E5:
      Data.SB = SW_MID;
      break;
    case 0x700:
      Data.SB = SW_UP;
      break;
    default:
      Data.SB = SW_NONE;
      break;
  }

  switch(BaseData.ch6){
    case 0x0BF:
      Data.SC = SW_DOWN;
      break;
    case 0x3E5:
      Data.SC = SW_MID;
      break;
    case 0x700:
      Data.SC = SW_UP;
      break;
    default:
      Data.SC = SW_NONE;
      break;
  }

  switch(BaseData.ch4){
    case 0x0BF:
      Data.SA = SW_DOWN;
      break;
    case 0x700:
      Data.SA = SW_UP;
      break;
    default:
      Data.SA = SW_NONE;
      break;
  }

  switch(BaseData.ch7){
    case 0x0BF:
      Data.SD = SW_DOWN;
      break;
    case 0x700:
      Data.SD = SW_UP;
      break;
    default:
      Data.SD = SW_NONE;
      break;
  }
}

/**
 * @brief crc8初始化
 *
 * @param poly
 */
inline void BETAFPV_Classdef::crc8init(uint8_t poly)
{
  for (int idx = 0; idx < 256; ++idx)
  {
    uint8_t crc = idx;
    for (int shift = 0; shift < 8; ++shift)
    {
      crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
    }
    _lut[idx] = crc & 0xff;
  }
}

/**
 * @brief crc8计算
 *
 * @param data 要进行crc校验的数据
 * @param len  输入数据的长度
 * @return uint8_t
 */
inline uint8_t BETAFPV_Classdef::crc8calc(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    crc = _lut[crc ^ *data++];
  }
  return crc;
}

/**
 * @brief  连接确认，更新设备的连接状态。每100ms内没有调用DataCapture()将
 *         进入离线模式。
 * @param  current_check_time 当前系统时间（毫秒）.
 * @return None
 */
inline void BETAFPV_Classdef::Check_Link(uint32_t current_check_time)
{
  /*开始检测*/
  if (last_check_time == 0)
  {
    last_check_time = current_check_time;
  }
  else
  {
    dt = current_check_time - last_check_time;
    if (dt > 100)
    {
      /*时钟计时溢出*/
      if (dt > 1000)
        last_check_time = 0;
      /*每100ms不置位就认为掉线*/
      else
        Status = LOST;

      /*重新开始检测*/
      last_check_time = 0;
    }
    else
    {
    }
  }
}

/**
 * @brief   得到DR16成员变量status的值，常用于判断DR16是否在线
 * @param   None
 * @return  LOST         离线
 * @return  ESTABLISHED  在线
 */
inline LinkageStatus_Typedef BETAFPV_Classdef::GetStatus(void)
{
  return Status;
}

/**
 * @brief 返回处理后的数据
 * 
 * @return BETAFPV_Data_Typedef& 
 */
inline BETAFPV_Data_Typedef &BETAFPV_Classdef::getData(void)
{
  return Data;
}

/**
 * @brief 返回原始数据
 * 
 * @return BETAFPV_BaseData_Typedef& 
 */
inline BETAFPV_BaseData_Typedef &BETAFPV_Classdef::getBaseData(void)
{
  return BaseData;
}

#endif
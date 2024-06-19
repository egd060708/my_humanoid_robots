#pragma once
#include "internal.h"

enum jointID
{
  HIP_ROLL = 0,
  HIP_PITCH = 1,
  HIP_YAW = 2,
  KNEE = 3,
	ANKLE = 4
};

/*与上位机通信*/
#pragma pack(1)
typedef struct _hostCom_s{
	float jointAngle[5];
	float endPoint[6];
}hostCom_s;
#pragma pack()

/*与另一个从机通信*/
#pragma pack(1)
typedef struct _slaveCom_s{
	float jointAngle[5];
}slaveCom_s;
#pragma pack()

class robotMiddleware{
private:

public:
	robotMiddleware();
	void init(QueueHandle_t _Usart_TxPort, uint8_t _port_num[2]);

	// float jointAngle_c[5]; // 当前电机角度
	// float jointAngle_t[5]; // 目标电机角度
	// float endPoint_c[6]; // 当前末端位姿
	// float endPoint_t[6]; // 目标末端位姿
	abstractMotor<LkMotorBass> jointMotor[5] = {LkMotorBass(1),LkMotorBass(2),LkMotorBass(3),LkMotorBass(4),LkMotorBass(5)};//单腿五个驱动关节
	void jointInit(LkMotorBass _motor[5]);

	hostCom_s sendHostPack;// 发送上位机数据
	hostCom_s recHostPack;// 接收上位机数据
	void processRecHost(hostCom_s* _recData);
	void sendHost();

	slaveCom_s sendSlavePack;//发送从机的数据
	slaveCom_s recSlavePack;//接收从机的数据
	void processRecSlave(slaveCom_s* _recData);
	void sendSlave();

	QueueHandle_t Usart_TxPort;
	USART_COB Usart_RxCOB[2];
  USART_COB Usart_TxCOB[2];
};

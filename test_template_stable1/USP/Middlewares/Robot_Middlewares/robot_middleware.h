#pragma once
#include <FreeRTOS.h>
#include <queue.h>
#include "SRML.h"
#include "BetaFPV.h"

// 判断是左腿还是右腿，预编译中间层内容
#define L_LEG 0

const uint16_t host_link_threshold = 40;
const uint16_t slave_link_threshold = 40;
#define HOSTCOM_ID_L 0x16
#define HOSTCOM_ID_R 0x17

enum jointID
{
	HIP_ROLL = 0,
	HIP_PITCH = 2,
	HIP_YAW = 1,
	KNEE = 3,
	ANKLE = 4
};

/*与上位机通信*/
#pragma pack(1)
typedef struct _hostCom_s
{
	float jointAngle[5];
	float endPoint[6];
	float dt;
	uint8_t id;
} hostCom_s;
#pragma pack()

/*与另一个从机通信*/
#pragma pack(1)
typedef struct _slaveCom_s
{
	float jointAngle[5];
	float endPoint[6];
	bool ctrl_enable = false;
} slaveCom_s;
#pragma pack()

class robotMiddleware
{
public:
	BETAFPV_Classdef *remote; //遥控器类型指针

public:
	robotMiddleware(BETAFPV_Classdef *_remote);
	void init(QueueHandle_t _Usart_TxPort, uint8_t _port_num);

	// abstractMotor<LkMotorBass> jointMotor[5] = {LkMotorBass(1), LkMotorBass(2), LkMotorBass(3), LkMotorBass(4), LkMotorBass(1)}; //单腿五个驱动关节
#if L_LEG
	LkMotorBass realjointMotor[5] = {LkMotorBass(1),LkMotorBass(3),LkMotorBass(2),LkMotorBass(4),LkMotorBass(1)};
#else
	LkMotorBass realjointMotor[5] = {LkMotorBass(6),LkMotorBass(3),LkMotorBass(2),LkMotorBass(4),LkMotorBass(5)};
#endif
	float offsetAngle[5] = {0};
	float startAngle[5] = {0};
	float direction[5] = {0};
	float ctrlAngle[5] = {0};
	bool is_start[5] = {false};
	myPID jointPID[5];
	float lastTarget[5] = {0};
	void jointInit();
	void jointGetStartAngle();

	hostCom_s sendHostPack; // 发送上位机数据
	hostCom_s recHostPack;	// 接收上位机数据
	void processRecHost(uint8_t *Recv_Data, uint16_t ReceiveLen);
	void sendHost();
	bool is_host_connet = false;
	uint16_t host_link_count = host_link_threshold; //初始化认为没有连接，设置为阈值

	slaveCom_s sendSlavePack; //发送从机的数据
	slaveCom_s recSlavePack;	//接收从机的数据
	void processRecSlave(slaveCom_s *_recData);
	void sendSlave();
	void sendSlaveFromISR();
	bool is_slave_connet = false;
	uint16_t slave_link_count = slave_link_threshold; //初始化认为没有连接，设置为阈值

	void returnDataSend(); // 返回数据函数
	void returnStateData(const float joint[5], const float endPoint[6]);

	QueueHandle_t Usart_TxPort;
	USART_COB Usart_RxCOB;
	USART_COB Usart_TxCOB;

	void ctrlActuate(const float _angle[5], float _time);
	void noCtrl();
	void Motor_Rec(CAN_COB *CAN_RxMsg);
	void link_check();

	bool ctrl_enable = false; // 多重判断归结到一个标志位指示是否能进行控制
	void judge_ctrl_enable(); // 根据当前状态判断是否能够使能控制
};

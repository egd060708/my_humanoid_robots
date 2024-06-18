#pragma once
#include "arm_def.h"
#include "internal.h"

/*机械臂控制器*/
template<uint8_t jNum>
class Arm_Controller_s
{
public:
	myPID Motor_pid[jNum];
	myMatrices<float> target_angle = myMatrices<float>(1, jNum);//目标关节角度
	myMatrices<float> current_angle = myMatrices<float>(1, jNum);//当前关节角度
	myMatrices<float> targetSpace = myMatrices<float>(4, 4);//目标齐次矩阵
	myMatrices<float> oriWorkSpace = myMatrices<float>(4, 4);//默认位型复位
	bool is_reset = false;//复位标志位
//public:
	Arm_Controller_s(rOrder _order)
	{
		arm_dof.set_eularOrder(_order);
	}
	void PID_params(uint8_t _num, float _p, float _i, float _d, float _imax, float _omax);
	Arm_Def_s<jNum> arm_dof;
	void setOriginAngle(const float _oangle[jNum]);//设置初始关节位型，赋予目标值和当前值
	void updateAngle_c(const float _cangle[jNum]);//更新观测值
	void updatePos_t(float _droll, float _dpitch, float _dyaw, float _dx, float _dy, float _dz);//末端运动算子
	void actuate();//控制器执行
	void setReset(bool _reset)
	{
		is_reset = _reset;
	}
	float motor_out[jNum];
};

/**
 * @brief pid参数设置
 * 
 * @tparam jNum 
 * @param _num 
 * @param _p 
 * @param _i 
 * @param _d 
 * @param _imax 
 * @param _omax 
 * @param _omin 
 */
template<uint8_t jNum>
inline void Arm_Controller_s<jNum>::PID_params(uint8_t _num, float _p, float _i, float _d, float _imax, float _omax)
{
	Motor_pid[_num].SetPIDParam(_p, _i, _d, _imax, _omax);
}

/**
 * @brief 设置初始位型
 * 
 * @tparam jNum 
 * @param _oangle 
 */
template<uint8_t jNum>
inline void Arm_Controller_s<jNum>::setOriginAngle(const float _oangle[jNum])
{
	arm_dof.update_Space_c(_oangle);
	//arm_dof.update_Space_t(arm_dof.workArray_c.getArray());//定义初始位型
	arm_dof.update_Space_t(arm_dof.workSpace_c);
	target_angle = arm_dof.jointSpace_t;
	oriWorkSpace = arm_dof.workSpace_t;
	arm_dof.workArray_c.print();
	arm_dof.jointSpace_t.print();
}

/**
 * @brief 更新当前关节变量
 * 
 * @tparam jNum 
 * @param _cangle 
 */
template<uint8_t jNum>
inline void Arm_Controller_s<jNum>::updateAngle_c(const float _cangle[jNum])
{
	current_angle.setArray(_cangle, jNum);
	arm_dof.update_Space_c(_cangle);
}

/**
 * @brief 更新目标位姿
 * 
 * @tparam jNum 
 * @param _droll 
 * @param _dpitch 
 * @param _dyaw 
 * @param _dx 
 * @param _dy 
 * @param _dz 
 */
template<uint8_t jNum>
inline void Arm_Controller_s<jNum>::updatePos_t(float _droll, float _dpitch, float _dyaw, float _dx, float _dy,float _dz)
{
	myMatrices<float> Tpitch(4);
	myMatrices<float> Tyaw(4);
	myMatrices<float> Troll(4);
	myMatrices<float> Tx(4);
	myMatrices<float> Ty(4);
	myMatrices<float> Tz(4);
	const float Apitch[4 * 4] = { cosf(_dpitch), 0,sinf(_dpitch),0,
											  0, 1,			   0,0,
								 -sinf(_dpitch), 0,cosf(_dpitch),0,
											  0, 0,			   0,1 };
	const float Ayaw[4 * 4] = { cosf(_dyaw),-sinf(_dyaw),0, 0,
								sinf(_dyaw), cosf(_dyaw),0, 0,
										  0,		   0,1, 0,
										  0,		   0,0, 1 };
	const float Aroll[4 * 4] = { 1,			0,			    0, 0,
								 0,cosf(_droll),-sinf(_droll), 0,
								 0,sinf(_droll), cosf(_droll), 0,
								 0,			0,			    0, 1 };
	const float Ax[4 * 4] = { 1,0,0,_dx,
							  0,1,0,  0,
							  0,0,1,  0,
							  0,0,0,  1 };
	const float Ay[4 * 4] = { 1,0,0,  0,
							  0,1,0,_dy,
							  0,0,1,  0,
							  0,0,0,  1 };
	const float Az[4 * 4] = { 1,0,0,  0,
							  0,1,0,  0,
							  0,0,1,_dz,
							  0,0,0,  1 };
	Tpitch.setArray(Apitch, 4 * 4);
	Tyaw.setArray(Ayaw, 4 * 4);
	Troll.setArray(Aroll, 4 * 4);
	Tx.setArray(Ax, 4 * 4);
	Ty.setArray(Ay, 4 * 4);
	Tz.setArray(Az, 4 * 4);
	myMatrices<float> workSpace_t = arm_dof.workSpace_t;
	workSpace_t = workSpace_t * Tpitch;
	workSpace_t = workSpace_t * Tyaw;
	workSpace_t = workSpace_t * Troll;
	workSpace_t = workSpace_t * Tx;
	workSpace_t = workSpace_t * Ty;
	workSpace_t = workSpace_t * Tz;
	if (is_reset)
	{
		workSpace_t = oriWorkSpace;
		workSpace_t.print();
	}
	arm_dof.update_Space_t(workSpace_t);
	target_angle = arm_dof.jointSpace_t;
}

/**
 * @brief 关节电机执行
 * 
 * @tparam jNum 
 */
template<uint8_t jNum>
inline void Arm_Controller_s<jNum>::actuate()
{
	for (int i = 0; i < jNum; i++)
	{
		Motor_pid[i].Target = target_angle.getElement(0, i);
		Motor_pid[i].Current = current_angle.getElement(0, i);
		motor_out[i] = Motor_pid[i].Adjust();
	}
}

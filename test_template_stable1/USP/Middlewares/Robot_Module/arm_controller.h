#pragma once
#include "arm_def.h"

/*机械臂控制器*/
template <uint8_t jNum>
class Arm_Controller_s
{
public:
	myPID Motor_pid[jNum];
	myMatrices target_angle = myMatrices(1, jNum);	//目标关节角度
	myMatrices current_angle = myMatrices(1, jNum); //当前关节角度
	myMatrices targetSpace = myMatrices(4, 4);			//目标齐次矩阵
	myMatrices oriWorkSpace = myMatrices(4, 4);			//默认位型复位
	bool is_reset = false;																				//复位标志位
																																// public:
	Arm_Controller_s(rOrder _order)
	{
		arm_dof.set_eularOrder(_order);
	}
	void PID_params(uint8_t _num, float _p, float _i, float _d, float _imax, float _omax);
	Arm_Def_s<jNum> arm_dof;
	void setOriginAngle(const float _oangle[jNum]); //设置初始关节位型，赋予目标值和当前值
	void updateAngle_c(const float _cangle[jNum]);
	void updateAngle_t(const float _tangle[jNum]);																								//更新观测值
	void updatePos_dt(float _droll, float _dpitch, float _dyaw, float _dx, float _dy, float _dz); //末端运动算子
	void updatePos_t(float _roll, float _pitch, float _yaw, float _x, float _y, float _z);				//末端运动目标
	void resetPos_t(); // 重置末端位置
	void updatePos_t(float _endPoint[6]);
	void actuate();			//控制器执行
	bool is_reaching(); //判断是否能够响应目标值
	
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
template <uint8_t jNum>
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
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::setOriginAngle(const float _oangle[jNum])
{
	arm_dof.update_Angle_c(_oangle);
	// arm_dof.update_Space_t(arm_dof.workArray_c.getArray());//定义初始位型
	arm_dof.update_Space_t(arm_dof.workSpace_c);
	target_angle = arm_dof.jointSpace_t;
	oriWorkSpace = arm_dof.workSpace_t;
	// arm_dof.workArray_c.print();
	// arm_dof.jointSpace_t.print();
}

/**
 * @brief 更新当前关节变量
 *
 * @tparam jNum
 * @param _cangle
 */
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::updateAngle_c(const float _cangle[jNum])
{
	current_angle.setArray(_cangle, jNum);
	arm_dof.update_Angle_c(_cangle);
}

/**
 * @brief 更新目标关节变量
 *
 * @tparam jNum
 * @param _tangle
 */
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::updateAngle_t(const float _tangle[jNum])
{
	target_angle.setArray(_tangle, jNum);
	arm_dof.update_Angle_t(_tangle);
}

/**
 * @brief 更新目标位姿（增量式）
 *
 * @tparam jNum
 * @param _droll
 * @param _dpitch
 * @param _dyaw
 * @param _dx
 * @param _dy
 * @param _dz
 */
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::updatePos_dt(float _droll, float _dpitch, float _dyaw, float _dx, float _dy, float _dz)
{
	myMatrices Tpitch(4);
	myMatrices Tyaw(4);
	myMatrices Troll(4);
	myMatrices Td(4);
	Tpitch.rotateY_T(_dpitch);
	Tyaw.rotateZ_T(_dyaw);
	Troll.rotateX_T(_droll);
	Td.transform_T(_dx,_dy,_dz);
	myMatrices workSpace_t = arm_dof.workSpace_t;
	workSpace_t = workSpace_t * Td;
	workSpace_t = workSpace_t * Tpitch;
	workSpace_t = workSpace_t * Tyaw;
	workSpace_t = workSpace_t * Troll;
	
	arm_dof.update_Space_t(workSpace_t);
	target_angle = arm_dof.jointSpace_t;
}

/**
 * @brief 更新目标位姿（绝对式）
 *
 * @tparam jNum
 * @param _roll
 * @param _pitch
 * @param _yaw
 * @param _x
 * @param _y
 * @param _z
 */
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::updatePos_t(float _roll, float _pitch, float _yaw, float _x, float _y, float _z)
{
	myMatrices Tpitch(4);
	myMatrices Tyaw(4);
	myMatrices Troll(4);
	myMatrices Td(4);
	float roll = upper::constrain(_roll, arm_dof.workArrayCs.getElement(0, 0), arm_dof.workArrayCs.getElement(0, 1));
	float pitch = upper::constrain(_pitch, arm_dof.workArrayCs.getElement(1, 0), arm_dof.workArrayCs.getElement(1, 1));
	float yaw = upper::constrain(_yaw, arm_dof.workArrayCs.getElement(2, 0), arm_dof.workArrayCs.getElement(2, 1));
	float x = upper::constrain(_x, arm_dof.workArrayCs.getElement(3, 0), arm_dof.workArrayCs.getElement(3, 1));
	float y = upper::constrain(_y, arm_dof.workArrayCs.getElement(4, 0), arm_dof.workArrayCs.getElement(4, 1));
	float z = upper::constrain(_z, arm_dof.workArrayCs.getElement(5, 0), arm_dof.workArrayCs.getElement(5, 1));
	Tpitch.rotateY_T(pitch);
	Tyaw.rotateZ_T(yaw);
	Troll.rotateX_T(roll);
	Td.transform_T(x,y,z);
	myMatrices workSpace_t(4);
	workSpace_t = Td;
	workSpace_t = workSpace_t * Troll;
	workSpace_t = workSpace_t * Tpitch;
	workSpace_t = workSpace_t * Tyaw;
	
	arm_dof.update_Space_t(workSpace_t);
	target_angle = arm_dof.jointSpace_t;
}

template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::updatePos_t(float _endPoint[6])
{
	myMatrices Tpitch(4);
	myMatrices Tyaw(4);
	myMatrices Troll(4);
	myMatrices Td(4);
	float x = upper::constrain(_endPoint[0], arm_dof.workArrayCs.getElement(0, 0), arm_dof.workArrayCs.getElement(0, 1));
	float y = upper::constrain(_endPoint[1], arm_dof.workArrayCs.getElement(1, 0), arm_dof.workArrayCs.getElement(1, 1));
	float z = upper::constrain(_endPoint[2], arm_dof.workArrayCs.getElement(2, 0), arm_dof.workArrayCs.getElement(2, 1));
	float roll = upper::constrain(_endPoint[3], arm_dof.workArrayCs.getElement(3, 0), arm_dof.workArrayCs.getElement(3, 1));
	float pitch = upper::constrain(_endPoint[4], arm_dof.workArrayCs.getElement(4, 0), arm_dof.workArrayCs.getElement(4, 1));
	float yaw = upper::constrain(_endPoint[5], arm_dof.workArrayCs.getElement(5, 0), arm_dof.workArrayCs.getElement(5, 1));
	Tpitch.rotateY_T(pitch);
	Tyaw.rotateZ_T(yaw);
	Troll.rotateX_T(roll);
	Td.transform_T(x,y,z);
	myMatrices workSpace_t(4);
	workSpace_t = Td;
	workSpace_t = workSpace_t * Troll;
	workSpace_t = workSpace_t * Tyaw;
	workSpace_t = workSpace_t * Tpitch;
	
	arm_dof.update_Space_t(workSpace_t);
	target_angle = arm_dof.jointSpace_t;
}

/**
 * @brief 重置机器人初始状态
 * 
 * @tparam jNum 
 */
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::resetPos_t(){
	arm_dof.update_Space_t(oriWorkSpace);
	target_angle = arm_dof.jointSpace_t;
}

/**
 * @brief 关节电机执行
 *
 * @tparam jNum
 */
template <uint8_t jNum>
inline void Arm_Controller_s<jNum>::actuate()
{
	for (int i = 0; i < jNum; i++)
	{
		Motor_pid[i].Target = target_angle.getElement(0, i);
		Motor_pid[i].Current = current_angle.getElement(0, i);
		motor_out[i] = Motor_pid[i].Adjust();
	}
}

template <uint8_t jNum>
inline bool Arm_Controller_s<jNum>::is_reaching()
{
	for (int i = 0; i < jNum; i++)
	{
		if (abs(target_angle.getElement(0, i) - current_angle.getElement(0, i)) > 0.1)
		{
			return false;
		}
	}
	return true;
}

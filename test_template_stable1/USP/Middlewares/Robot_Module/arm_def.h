#pragma once
#include <iostream>
#include "myMatrices.h"
#include "math.h"
#include "Upper_Public.h"

using namespace mM; //矩阵库命名空间

enum class rOrder
{
	XYZ,
	XZY,
	YXZ,
	YZX,
	ZXY,
	ZYX
};

const float zero_check = 0.001f;

template <uint8_t jNum>
class Arm_Def_s
{
private:
#pragma pack(1)
	struct T_s
	{
		float T[16];
	};
#pragma pack()

#pragma pack(1)
	struct A_s
	{
		float A[6];
	};
#pragma pack()

#pragma pack(1)
	struct J_s
	{
		float J[jNum];
	};
#pragma pack()
	// 主要用于debug观测变量
	T_s workSpace_ts;
	T_s workSpace_cs;
	A_s workArray_ts;
	A_s workArray_cs;
	J_s jointSpace_ts;
	J_s jointSpace_cs;

public:
	myMatrices DH_params = myMatrices(jNum, 5);		 // DH参数存储矩阵(4个基本参数+1个关节offset)
	myMatrices jointSpace_t = myMatrices(1, jNum); //目标关节空间(输出给机器人，减去offset)
	myMatrices jointSpace_c = myMatrices(1, jNum); //当前关节空间(提供给坐标运算，加上offset)
	myMatrices workSpace_t = myMatrices(4);				 //目标工作空间(各个坐标系的位姿）
	myMatrices workSpace_c = myMatrices(4);				 //当前工作空间
	myMatrices workArray_t = myMatrices(1, 6);		 //工作空间向量形式
	myMatrices workArray_c = myMatrices(1, 6);
	myMatrices preSpaceCali = myMatrices(4);	 //预姿态校准矩阵
	myMatrices lastSpaceCali = myMatrices(4);	 //末端姿态校准矩阵
	myMatrices jointCs = myMatrices(jNum, 2);	 //关节目标约束
	myMatrices workArrayCs = myMatrices(6, 2); //目标向量约束
	myMatrices fk(const float _angle[jNum]);					 //正运动学解算
	myMatrices ik();																	 //逆运动学解算
	myMatrices T2A(myMatrices _T);						 //齐次变换矩阵转成六维向量
	rOrder order_c;
	// public:
	Arm_Def_s() {}
	void set_eularOrder(rOrder _order) { order_c = _order; };
	void import_DH(uint8_t _num, float _alpha, float _a, float _d, float _theta, float _offset); //设置DH参数（改进型）
	void update_Angle_c(const float _angle[jNum]);																							 //更新当前关节角
	void update_Angle_t(const float _angle[jNum]);																							 //更新目标关节角
	void update_Space_t(const float _endSpace[6]);																							 //更新目标空间(末端采用三维坐标+欧拉角向量表示）
	void update_Space_t(myMatrices _T);																									 //更新目标空间(纯旋转矩阵表示）
	void set_spaceCali(const float _preSpaceCali[4 * 4], const float _lastSpaceCali[4 * 4]);		 //更新姿态校准矩阵
	void set_jointCs(const float _acs[2 * jNum]);																								 //设置关节空间目标约束
	void set_workArrayCs(const float _wcs[2 * 6]);																							 //设置空间向量目标约束
};

/**
 * @brief 通过改进型DH参数定义机械臂
 *
 * @tparam jNum
 * @param _num
 * @param _alpha
 * @param _a
 * @param _d
 * @param _theta
 * @param _offset
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::import_DH(uint8_t _num, float _alpha, float _a, float _d, float _theta, float _offset)
{
	if (_num > 0)
	{
		float importValue[5] = {_alpha, _a, _d, _theta, _offset};
		DH_params.setRowArray(importValue, _num - 1);
	}
}

/**
 * @brief 更新当前关节空间
 *
 * @tparam jNum
 * @param _angle
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::update_Angle_c(const float _angle[jNum])
{
	//拷贝关节空间值,并更新DH参数矩阵
	for (int i = 0; i < jNum; i++)
	{
		jointSpace_c.setElement(0, i, _angle[i]);
		jointSpace_cs.J[i] = _angle[i];
		DH_params.setElement(i, 3, jointSpace_c.getElement(0, i));
	}
	//正运动学解算
	workSpace_c = fk(jointSpace_c.getArray());
	workArray_c = T2A(workSpace_c);
	memcpy(&workArray_cs, workArray_c.getArray(), sizeof(workArray_cs));
	memcpy(&workSpace_cs, workSpace_c.getArray(), sizeof(workSpace_cs));
}

/**
 * @brief 更新目标关节空间
 *
 * @tparam jNum
 * @param _angle
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::update_Angle_t(const float _angle[jNum])
{
	//拷贝关节空间值,并更新DH参数矩阵
	for (int i = 0; i < jNum; i++)
	{
		jointSpace_t.setElement(0, i, _angle[i]);
		jointSpace_ts.J[i] = _angle[i];
	}
	//正运动学解算
	workSpace_t = fk(jointSpace_t.getArray());
	workArray_t = T2A(workSpace_t);
	memcpy(&workArray_ts, workArray_t.getArray(), sizeof(workArray_ts));
	memcpy(&workSpace_ts, workSpace_t.getArray(), sizeof(workSpace_ts));
}

/**
 * @brief 通过末端六维位姿向量更新目标工作空间
 *
 * @tparam jNum
 * @param _endSpace
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::update_Space_t(const float _endSpace[6])
{
	workArray_t.setRowArray(_endSpace, 0);
	memcpy(&workArray_ts, workArray_t.getArray(), sizeof(workArray_ts));
	myMatrices T1(4);
	myMatrices T2(4);
	myMatrices T3(4);
	T1.rotateX_T(_endSpace[3]);
	T2.rotateY_T(_endSpace[4]);
	T3.rotateZ_T(_endSpace[5]);
	//配置末端矩阵的旋转部分
	myMatrices T = T1 * T3 * T2;
	//配置末端矩阵的位移部分
	T.setElement(0, 3, _endSpace[0]);
	T.setElement(1, 3, _endSpace[1]);
	T.setElement(2, 3, _endSpace[2]);

	//拷贝目标空间矩阵
	workSpace_t = T;
	memcpy(&workSpace_ts, workSpace_t.getArray(), sizeof(workSpace_ts));

	jointSpace_t = ik();
	memcpy(&jointSpace_ts, jointSpace_t.getArray(), sizeof(jointSpace_ts));
}

/**
 * @brief 通过齐次变换矩阵更新目标工作空间
 *
 * @tparam jNum
 * @param _T
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::update_Space_t(myMatrices _T)
{
	workArray_t = T2A(_T);
	memcpy(&workArray_ts, workArray_t.getArray(), sizeof(workArray_ts));
	// _T.print();
	// workArray_t.print();
	//拷贝目标空间矩阵
	workSpace_t = _T;
	memcpy(&workSpace_ts, workSpace_t.getArray(), sizeof(workSpace_ts));

	jointSpace_t = ik();
	memcpy(&jointSpace_ts,jointSpace_t.getArray(),sizeof(jointSpace_ts));
}

/**
 * @brief 正运动学
 *
 * @tparam jNum
 * @return myMatrices
 */
template <uint8_t jNum>
inline myMatrices Arm_Def_s<jNum>::fk(const float _angle[jNum])
{
	myMatrices T(4);
	T = preSpaceCali; //预姿态校准矩阵
	for (int i = 0; i < jNum; i++)
	{
		myMatrices T1(4); // alpha和a组成
		myMatrices T2(4); // d和theta组成
		float _alpha = DH_params.getElement(i, 0);
		float _a = DH_params.getElement(i, 1);
		float _d = DH_params.getElement(i, 2);
		float _theta = DH_params.getElement(i, 4) + _angle[jNum];
		float T1_array[4 * 4] = {1, 0, 0, _a,
														 0, cosf(_alpha), -sinf(_alpha), 0,
														 0, sinf(_alpha), cosf(_alpha), 0,
														 0, 0, 0, 1};
		float T2_array[4 * 4] = {cosf(_theta), -sinf(_theta), 0, 0,
														 sinf(_theta), cosf(_theta), 0, 0,
														 0, 0, 1, _d,
														 0, 0, 0, 1};
		T1.setArray(T1_array, 4 * 4);
		T2.setArray(T2_array, 4 * 4);
		myMatrices dT = T1 * T2;
		T = T * dT; //右乘
	}
	T = T * lastSpaceCali; // 加入末端姿态校准矩阵
	return T;
}

/**
 * @brief 非通用逆运动学
 *
 * @tparam jNum
 * @return myMatrices
 */
template <uint8_t jNum>
inline myMatrices Arm_Def_s<jNum>::ik()
{
	//求逆解之前先给向量空间添加约束
	for (int i = 0; i < 6; i++)
	{
		if (workArrayCs.getElement(i, 0) != 0 || workArrayCs.getElement(i, 1) != 0)
		{
			float temp = upper::constrain(workArray_t.getElement(0, i), workArrayCs.getElement(i, 0), workArrayCs.getElement(i, 1));
			workArray_t.setElement(0, i, temp);
			workArray_ts.A[i] = temp;
		}
	}
	myMatrices J(1, jNum);
	if (jNum == 5)
	{

		/*双足单腿*/
		float L1 = 0.066;
		float L2 = 0.145;
		float L3 = 0.180;
		float L4 = 0.200;
		float L5 = 0.060;
		J.setElement(0, 0, atan2f(-workArray_t.getElement(0, 1), workArray_t.getElement(0, 2)));
		J.setElement(0, 1, -workArray_t.getElement(0, 5));
		//首先减去L5得到关节5的位姿
		myMatrices reduceL5(4);
		reduceL5.transform_T(0,0,L5);
		reduceL5 = workSpace_t * reduceL5; // 此处直接得到了关节5的位姿
		float L34 = sqrtf(powf(reduceL5.getElement(0, 3), 2) + powf(reduceL5.getElement(1, 3), 2) + powf(reduceL5.getElement(2, 3), 2)) - L1 - L2;
		J.setElement(0, 3, PI - acosf(upper::constrain(((L3 * L3 + L4 * L4 - L34 * L34) / 2 / L3 / L4), 0.99)));
		float theta34 = acosf(upper::constrain(((L3 * L3 + L34 * L34 - L4 * L4) / 2 / L3 / L34), 0.99));
		float theta35 = atan2(-workArray_t.getElement(0, 0), -workArray_t.getElement(0, 2));
		J.setElement(0, 2, theta35 - theta34);
		J.setElement(0, 4, workArray_t.getElement(0, 4) - J.getElement(0, 3) - J.getElement(0, 2));
	}

	for (int i = 0; i < jNum; i++)
	{
		if (jointCs.getElement(i, 0) != 0 || jointCs.getElement(i, 1) != 0)
		{
			float temp = upper::constrain(J.getElement(0, i), jointCs.getElement(i, 0), jointCs.getElement(i, 1));
			J.setElement(0, i, temp);
		}
		else
		{
			J.setElement(0, i, J.getElement(0, i));
		}
	}

	return J;
}

/**
 * @brief 齐次变换矩阵转换到六维位姿向量(后三维角度数据是根据旋转角顺序得出的)
 *
 * @tparam jNum
 * @param _T
 * @return myMatrices
 */
template <uint8_t jNum>
inline myMatrices Arm_Def_s<jNum>::T2A(myMatrices _T)
{
	myMatrices A(1, 6);
	A.setElement(0, 0, _T.getElement(0, 3));
	A.setElement(0, 1, _T.getElement(1, 3));
	A.setElement(0, 2, _T.getElement(2, 3));
	switch (order_c)
	{
	case rOrder::YZX:
		A.setElement(0, 5, atan2f(_T.getElement(1, 0), sqrtf(powf(_T.getElement(0, 0), 2) + powf(_T.getElement(2, 0), 2))));
		A.setElement(0, 4, atan2f(-_T.getElement(2, 0) / cosf(A.getElement(0, 4)), _T.getElement(0, 0) / cosf(A.getElement(0, 4))));
		A.setElement(0, 3, atan2f(-_T.getElement(1, 2) / cosf(A.getElement(0, 4)), _T.getElement(1, 1) / cosf(A.getElement(0, 4))));
		break;
	case rOrder::ZXY:
		A.setElement(0, 3, atan2f(_T.getElement(2, 1), sqrtf(powf(_T.getElement(2, 0), 2) + powf(_T.getElement(2, 2), 2))));
		A.setElement(0, 5, atan2f(-_T.getElement(2, 0) / cosf(A.getElement(0, 4)), _T.getElement(2, 2) / cosf(A.getElement(0, 4))));
		A.setElement(0, 4, atan2f(-_T.getElement(0, 1) / cosf(A.getElement(0, 4)), _T.getElement(1, 1) / cosf(A.getElement(0, 4))));
		break;
	case rOrder::XZY:
		A.setElement(0, 5, atan2f(-_T.getElement(0, 1), sqrtf(powf(_T.getElement(1, 1), 2) + powf(_T.getElement(2, 1), 2))));
		A.setElement(0, 3, atan2f(_T.getElement(2, 1) / cosf(A.getElement(0, 4)), _T.getElement(1, 1) / cosf(A.getElement(0, 4))));
		A.setElement(0, 4, atan2f(_T.getElement(0, 2) / cosf(A.getElement(0, 4)), _T.getElement(0, 0) / cosf(A.getElement(0, 4))));
		break;
	default:
		break;
	}
	return A;
}

/**
 * @brief 设置末端姿态校准矩阵
 *
 * @tparam jNum
 * @param _spaceCali
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::set_spaceCali(const float _preSpaceCali[4 * 4], const float _lastSpaceCali[4 * 4])
{
	preSpaceCali.setArray(_preSpaceCali, 4 * 4);
	lastSpaceCali.setArray(_lastSpaceCali, 4 * 4);
}

/**
 * @brief 设置关节空间目标约束
 *
 * @tparam jNum
 * @param _acs
 */
template <uint8_t jNum>
inline void Arm_Def_s<jNum>::set_jointCs(const float _acs[2 * jNum])
{
	jointCs.setArray(_acs, 2 * jNum);
}

/**
 * @brief 设置空间向量目标约束
 *
 * @tparam jNum
 * @param _wcs
 */
template <uint8_t jNum>
void Arm_Def_s<jNum>::set_workArrayCs(const float _wcs[2 * 6])
{
	workArrayCs.setArray(_wcs, 2 * 6);
}

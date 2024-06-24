#include "Leg_intergration.h"
#include "internal.h"

Reset_State resetState;
LostCtrl_State lostCtrlState;
PreCtrl_State preCtrlState;
Ctrl_State ctrlState;
Debug_State debugState;

/**
 * @brief 重置状态机
 *
 */
void Reset_State::State_Handler()
{
  context->state_flag = sMachine::RESET;

  context->middleware->noCtrl();
  context->is_reset = true;
  context->Status_Switching(&lostCtrlState);
}

/**
 * @brief 失控状态机
 *
 */
void LostCtrl_State::State_Handler()
{
  context->state_flag = sMachine::LOSTCTRL;
  context->controller->setReset(true);
  if (L_LEG == 1)
  {
    context->controller->updatePos_t(context->middleware->recHostPack.endPoint);
  }
  else
  {
    context->controller->updatePos_t(context->middleware->recSlavePack.endPoint);
  }
  context->middleware->noCtrl();
}

/**
 * @brief 预控制状态机
 *
 */
void PreCtrl_State::State_Handler()
{
  context->state_flag = sMachine::PRECTRL;
  // 控制双腿到达初始位型
  context->controller->setReset(true);
  if (L_LEG == 1)
  {
    context->controller->updatePos_t(context->middleware->recHostPack.endPoint);
  }
  else
  {
    context->controller->updatePos_t(context->middleware->recSlavePack.endPoint);
  }
  float speed[5] = {10};
  context->middleware->ctrlActuate(context->controller->target_angle.getArray(), speed);
  if (context->controller->is_reaching())
  {
    context->Status_Switching(&ctrlState);
  }
}

/**
 * @brief 控制状态机
 *
 */
void Ctrl_State::State_Handler()
{
  context->state_flag = sMachine::CTRL;
  // 控制双腿到达目标位型
  if (L_LEG == 1)
  {
    context->controller->updatePos_t(context->middleware->recHostPack.endPoint);
  }
  else
  {
    context->controller->updatePos_t(context->middleware->recSlavePack.endPoint);
  }
  float speed[5];
  for (int i = 0; i < 5; i++)
  {
    // 计算速度并且转为RPM
    speed[i] = (context->controller->target_angle.getElement(0, i) - context->controller->current_angle.getElement(0, i)) * 60 / 2 / PI / context->middleware->recHostPack.dt;
  }
  context->middleware->ctrlActuate(context->controller->target_angle.getArray(), speed);
}

/**
 * @brief 调试状态机(随便拉屎)
 *
 */
void Debug_State::State_Handler()
{
  context->state_flag = sMachine::DEBUG;
	context->controller->updatePos_t(context->debug_endPoint);
//	context->controller->updateAngle_t(context->debug_joint_t);
  context->middleware->noCtrl();
}

/**
 * @brief 统筹类构造函数
 *
 * @param _controller
 * @param _middleware
 */
Humanoid_Leg_Classdef::Humanoid_Leg_Classdef(Arm_Controller_s<5> *_controller, robotMiddleware *_middleware) : controller(_controller), middleware(_middleware)
{
  current_state = &resetState;
  // 导入dh参数，初始化控制器
  const float preCali[4 * 4] = {0, -1, 0, 0,
                                1, 0, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1};
  const float lastCali[4 * 4] = {0, 0, -1, 0.06,
                                 -1, 0, 0, 0,
                                 0, 1, 0, 0,
                                 0, 0, 0, 1};
  const float angle[5] = {0, 0, 0, 0, 0};
  const float jointCs[2 * 5] = {PI * 0.4, 0,
                                PI * 0.1, PI * -0.3,
                                PI * 0.3, PI * -0.5,
                                PI * 0.6, 0,
                                PI * 0.3, PI * -0.1};
  const float workArrCs[2 * 6] = {0.3, -0.3,
                                  0.2, -0.05,
                                  -0.2, -0.65,
                                  PI * 0.4, 0,
                                  PI * 0.4, -PI * 0.4,
                                  PI * 0.3, -PI * 0.1};
  controller->arm_dof.import_DH(1, PI * 0.5, 0.06, 0, 0, 0);
  controller->arm_dof.import_DH(2, PI * 0.5, 0, 0.145 + 0.066, 0, PI * 0.5);
  controller->arm_dof.import_DH(3, PI * 0.5, 0, 0, 0, PI * 0.5);
  controller->arm_dof.import_DH(4, 0, 0.180, 0, 0, 0);
  controller->arm_dof.import_DH(5, 0, 0.200, 0, 0, 0);
  controller->arm_dof.set_spaceCali(preCali, lastCali);
  controller->arm_dof.set_jointCs(jointCs);
  controller->arm_dof.set_workArrayCs(workArrCs);
  controller->setOriginAngle(angle);
}

/**
 * @brief 状态更新函数
 *
 */
void Humanoid_Leg_Classdef::State_Data_Update()
{
  // 更新通信检测
  middleware->link_check();
  // 更新控制使能状态
  middleware->judge_ctrl_enable();
  // 机械臂控制，更新电机状态，正运动学计算当前姿态，逆运动学求解目标姿态
  float jointAngle[5];
  for (int i = 0; i < 5; i++)
  {
    jointAngle[i] = middleware->jointMotor->getMotorTotalAngle();
  }
  controller->updateAngle_c(jointAngle);
}

/**
 * @brief 状态判断与执行函数
 *
 */
void Humanoid_Leg_Classdef::State_Judge()
{
  if (!is_reset) // 第一次运行初始化
  {
    Status_Switching(&resetState);
  }
  else
  {
    if (is_debug) // debug状态只能进入debug状态机
    {
      Status_Switching(&debugState);
    }
    else
    {
      if (!middleware->ctrl_enable) // 如果使能了控制，那就开始
      {
        Status_Switching(&lostCtrlState);
      }
      else
      {
        Status_Switching(&preCtrlState);
      }
    }
  }
  current_state->State_Handler(); // 执行当前状态机
}

/**
 * @brief 返回当前腿部的参数
 *
 */
void Humanoid_Leg_Classdef::State_Return()
{
  middleware->returnStateData(controller->arm_dof.jointSpace_c.getArray(), controller->arm_dof.workArray_c.getArray());
}
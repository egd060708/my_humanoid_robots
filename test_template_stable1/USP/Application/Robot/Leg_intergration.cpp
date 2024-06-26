#include "Leg_intergration.h"
#include "internal.h"

Reset_State resetState;
LostCtrl_State lostCtrlState;
PreCtrl_State preCtrlState;
Ctrl_State ctrlState;
Debug_State debugState;

extern float ch[6];
float kp = 30;
float ki = 5;
float i_max = 5;
float o_max = 40;

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
  context->controller->resetPos_t();
  context->middleware->noCtrl();
  context->middleware->isReset_reaching = false;
  if (context->middleware->preCtrl_enable == true)
  {
    context->Status_Switching(&preCtrlState);
  }
}

/**
 * @brief 预控制状态机
 *
 */
void PreCtrl_State::State_Handler()
{
  context->state_flag = sMachine::PRECTRL;
  // 控制双腿到达初始位型
  context->controller->resetPos_t();
  context->middleware->ctrlActuate(context->controller->target_angle.getArray(), 4);
  context->middleware->isReset_reaching = context->controller->is_reaching();
  // if (context->middleware->isReset_reaching == true && context->middleware->ctrl_enable == true)
  // {
  //   context->Status_Switching(&ctrlState);
  // }
  if (context->middleware->ctrl_enable == true)
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
  if (L_LEG == 0)
  {
    if (context->middleware->is_host_connet)
    {
      context->controller->updatePos_t(context->middleware->cmd_endPoint);
      context->middleware->ctrlActuate(context->controller->target_angle.getArray(), 2);
    }
    else
    {
      context->middleware->ctrlActuate(context->controller->target_angle.getArray(), 4);
    }
  }
  else if (L_LEG == 1)
  {
    if (context->middleware->is_slave_connet)
    {
      context->controller->updatePos_t(context->middleware->cmd_endPoint);
      context->middleware->ctrlActuate(context->controller->target_angle.getArray(), 2);
    }
    else
    {
      context->middleware->ctrlActuate(context->controller->target_angle.getArray(), 4);
    }
  }
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

  // float speed[5] = {10};
  // myPID anglePID;
  // anglePID.SetPIDParam(kp,ki,0,i_max,o_max);
  // context->middleware->ctrlActuate(context->realAngle, speed);
  if (L_LEG == 1)
  {
    if (context->middleware->ctrl_enable == true)
    {
      // context->middleware->ctrlActuate(context->controller->target_angle.getArray(), context->test_t);

      // float angle = (context->test_angle / context->middleware->direction[0] + context->middleware->offsetAngle[0]);
      // anglePID.Target = angle;
      // anglePID.Current = context->middleware->realjointMotor[0].getData().singleAngle;
      // ch[0] = anglePID.Target;
      // ch[1] = anglePID.Current;
      // anglePID.Adjust();
      // context->middleware->realjointMotor[0].speedControl(anglePID.Out);
    }
    else
    {
      context->middleware->noCtrl();
    }
  }
  else
  {
    context->middleware->sendSlavePack.ctrl_enable = context->middleware->ctrl_enable;
    context->middleware->sendSlave();
    if (context->middleware->remote->getData().SD == SW_UP)
    {
      // context->middleware->ctrlActuate(context->controller->target_angle.getArray(), context->test_t);

      // float angle = (context->test_angle / context->middleware->direction[0] + context->middleware->offsetAngle[0]);
      // anglePID.Target = angle;
      // anglePID.Current = context->middleware->realjointMotor[0].getData().singleAngle;
      // ch[0] = anglePID.Target;
      // ch[1] = anglePID.Current;
      // anglePID.Adjust();
      // context->middleware->realjointMotor[0].speedControl(anglePID.Out);
    }
    else
    {
      context->middleware->noCtrl();
    }
  }

  if (context->is_offset == true && context->real_offset == false)
  {
    context->real_offset = true;
    context->middleware->realjointMotor[0].writeNowEncoderAsOffset();
  }
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
  const float angle[5] = {0, 0., -0.2, 0.4, -0.2};
#if L_LEG
  const float jointCs[2 * 5] = {PI * 0.3, 0,
                                PI * 0.3, PI * -0.1,
                                PI * 0.34, PI * -0.34,
                                PI * 0.45, 0,
                                PI * 0.3, PI * -0.3};
  const float workArrCs[2 * 6] = {0.3, -0.3,
                                  0.2, -0.05,
                                  -0.5, -0.65,
                                  0, 0,
                                  0, 0,
                                  0, 0};
#else
  const float jointCs[2 * 5] = {0, -PI * 0.3,
                                PI * 0.1, PI * -0.3,
                                PI * 0.34, PI * -0.34,
                                PI * 0.45, 0,
                                PI * 0.3, PI * -0.3};
  const float workArrCs[2 * 6] = {0.3, -0.3,
                                  0.05, -0.2,
                                  -0.5, -0.65,
                                  0, 0,
                                  0, 0,
                                  0, 0};
#endif

  // controller->arm_dof.import_DH(1, PI * 0.5, 0.06, 0, 0, 0);
  controller->arm_dof.import_DH(1, PI * 0.5, 0, 0, 0, 0);
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
  // 更新电机初始状态
  middleware->jointGetStartAngle();
  // 机械臂控制，更新电机状态，正运动学计算当前姿态，逆运动学求解目标姿态(记得转成弧度制)
  for (int i = 0; i < 5; i++)
  {
    // realAngle[i] = (middleware->realjointMotor[i].getData().totalAngleLocal + middleware->startAngle[i] - middleware->offsetAngle[i]) * middleware->direction[i];
    realAngle[i] = (middleware->realjointMotor[i].getData().singleAngle - middleware->offsetAngle[i]) * middleware->direction[i] / 360. * PI;
  }
  controller->updateAngle_c(realAngle);
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
      if (!middleware->preCtrl_enable) // 如果使能了控制，那就开始
      {
        Status_Switching(&lostCtrlState);
      }
      // else
      // {
      //   Status_Switching(&preCtrlState);
      // }
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
  middleware->returnStateData(controller->arm_dof.workArray_c.getArray());
}
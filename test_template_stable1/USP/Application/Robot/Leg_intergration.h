#pragma once
#include "robot_middleware.h"
#include "arm_controller.h"

class Humanoid_Leg_Classdef;

/*状态机模式*/
namespace sMachine
{
  enum E_StateMachine
  {
    RESET = 0,
    LOSTCTRL = 1,
    PRECTRL = 2,
    CTRL = 3,
    DEBUG = 4
  };
}

/*状态类型*/
class State_Base
{
protected:
  Humanoid_Leg_Classdef *context; // 上下文指针，指向要操作的类对象
public:
  // 上下文切换，让实例化对象获取被控对象的指针（索引）
  void Set_Context(Humanoid_Leg_Classdef *_context)
  {
    this->context = _context;
  }
  virtual void State_Handler() = 0; // 状态机方法，子类中实现实例化
};
/*初始化状态机*/
class Reset_State : public State_Base
{
  virtual void State_Handler();
};
/*失控状态机*/
class LostCtrl_State : public State_Base
{
  virtual void State_Handler();
};
/*预控制状态机*/
class PreCtrl_State : public State_Base
{
  virtual void State_Handler();
};
/*控制状态机*/
class Ctrl_State : public State_Base
{
  virtual void State_Handler();
};
/*调试状态机*/
class Debug_State : public State_Base
{
  virtual void State_Handler();
};

class Humanoid_Leg_Classdef
{
public:
  // 构造函数
  Humanoid_Leg_Classdef(Arm_Controller_s<5> *_controller, robotMiddleware *_middleware);
  // 切换状态函数
  void Status_Switching(State_Base *_state)
  {
    this->current_state = _state;           // 状态指针指向新的状态
    this->current_state->Set_Context(this); // 将当前数据挂载到状态机当中
  }
  // 状态机判断函数
  void State_Judge();
  // 数据更新函数
  void State_Data_Update();
  // 返回解算的数据
  void State_Return();

public:
  // 状态基类指针，用于更新状态
  State_Base *current_state;
  sMachine::E_StateMachine state_flag;
  // 使用友元类，使状态方法可以操作大类成员
  friend Reset_State;
  friend LostCtrl_State;
  friend PreCtrl_State;
  friend Ctrl_State;
  friend Debug_State;
  // 操作类型指针
  Arm_Controller_s<5> *controller; // 连杆控制器
  robotMiddleware *middleware;     //中间层

  float realAngle[5] = {0};
  // float lastTargetAngle[5] = {0};
  // float targetAngle[5] = {0};

  bool is_reset = false;
  bool is_debug = false;                                //定义一个debug标志，如果是debug模式，则只在debug状态机中运行
  float debug_endPoint[6] = {0, 0, -0.63, 0, 0., 0}; //调试用的末端指令
  float debug_joint_t[5] = {0, 0, 0, 0, 0};            //调试用的关节指令

  bool is_offset = false;
  bool real_offset = false;

  float test_angle = 0;
  float test_t = 5.;
  bool is_test_on = false;
};
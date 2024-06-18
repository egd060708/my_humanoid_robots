#pragma once
#include "arm_controller.h"

class Humanoid_Leg_Classdef;

/*状态机模式*/
namespace sMachine
{
  enum E_StateMachine
  {
    RESET = 0,
    LOSTCTRL = 1,
    CTRL = 2
  };
}

/*状态类型*/
class State_Base
{
protected:
  Humanoid_Leg_Classdef *context;// 上下文指针，指向要操作的类对象
public:
  // 上下文切换，让实例化对象获取被控对象的指针（索引）
  void Set_Context(Humanoid_Leg_Classdef *_context)
  {
    this->context = _context;
  }
  virtual void State_Handler() = 0; // 状态机方法，子类中实现实例化
};
/*初始化状态机*/
class Reset_State : public State_Base{
  virtual void State_Handler();
};
/*失控状态机*/
class LostCtrl_State : public State_Base{
  virtual void State_Handler();
};
/*控制状态机*/
class Ctrl_State : public State_Base{
  virtual void State_Handler();
};


class Humanoid_Leg_Classdef{
public:
  // 切换状态函数
  void Status_Switching(State_Base *_state)
  {
    this->current_state = _state;           // 状态指针指向新的状态
    this->current_state->Set_Context(this); // 将当前数据挂载到状态机当中
  }
private:
  // 状态基类指针，用于更新状态
  State_Base *current_state;
  sMachine::E_StateMachine state_flag;
  // 使用友元类，使状态方法可以操作大类成员
  friend Reset_State;
  friend LostCtrl_State;
  friend Ctrl_State;
  // 操作类型指针
  Arm_Controller_s<5> *controller;// 连杆控制器
};
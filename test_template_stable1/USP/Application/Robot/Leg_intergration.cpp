#include "Leg_intergration.h"

Reset_State resetState;
LostCtrl_State lostCtrlState;
Ctrl_State ctrlState;

/**
 * @brief 重置状态机
 * 
 */
void Reset_State::State_Handler(){
  context->state_flag = sMachine::RESET;
}

/**
 * @brief 失控状态机
 * 
 */
void LostCtrl_State::State_Handler(){
  context->state_flag = sMachine::LOSTCTRL;
}

/**
 * @brief 控制状态机
 * 
 */
void Ctrl_State::State_Handler(){
  context->state_flag = sMachine::CTRL;
}
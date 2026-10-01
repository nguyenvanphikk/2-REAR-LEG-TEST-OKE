/*============================= Stand Up ==============================*/
/**
 * MIT_3HP temporary, low-gain joint-position test near the encoder zero.
 */

#include "FSM_State_StandUp.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
_ini_foot_pos(4){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_StandUp<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandUp<T>::run() {
  // Suspended-robot joint test: use the same small knee target as the
  // previously tested front-leg command on all four legs.
  constexpr T kKneeTest = T(0.02);
  this->_data->_legController->setEnabled(true);
  for (int leg = 0; leg < 4; ++leg) {
    auto& command = this->_data->_legController->commands[leg];
    command.qDes.setZero();
    command.qDes[2] = kKneeTest;
    command.qdDes.setZero();
    command.kpJoint.setZero();
    command.kdJoint.setZero();
    command.kpJoint.diagonal().setConstant(T(5));
    command.kdJoint.diagonal().setConstant(T(0.2));
    command.kpCartesian.setZero();
    command.kdCartesian.setZero();
    command.tauFeedForward.setZero();
    command.forceFeedForward.setZero();
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition() {
  this->nextStateName = this->stateName;

  // Giu goc dich den khi nguoi dung chon PASSIVE/E-STOP.

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_STAND_UP:
      break;
    case K_BALANCE_STAND:
    case K_LOCOMOTION:
    case K_VISION:
      // Chế độ kiểm tra zero không cho chuyển tới điều khiển chuyển động.
      this->_data->controlParameters->control_mode = K_STAND_UP;
      break;


    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_StandUp<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_StandUp<T>::onExit() {
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;

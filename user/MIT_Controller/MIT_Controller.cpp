#include "MIT_Controller.hpp"

MIT_Controller::MIT_Controller():RobotController(){  }

int MIT_Controller::getRearTestStage() const {
  return _controlFSM && _controlFSM->currentState->stateName == FSM_StateName::STAND_UP
             ? _controlFSM->statesList.standUp->rearTestStage() : 0;
}

int MIT_Controller::getRearTestFault() const {
  return _controlFSM ? _controlFSM->statesList.standUp->rearTestFault() : 0;
}

int MIT_Controller::getControllerMode() const {
  if (!_controlFSM || !_controlFSM->currentState) return K_PASSIVE;
  switch (_controlFSM->currentState->stateName) {
    case FSM_StateName::PASSIVE: return K_PASSIVE;
    case FSM_StateName::STAND_UP: return K_STAND_UP;
    case FSM_StateName::BALANCE_STAND: return K_BALANCE_STAND;
    case FSM_StateName::LOCOMOTION: return K_LOCOMOTION;
    case FSM_StateName::RECOVERY_STAND: return K_RECOVERY_STAND;
    case FSM_StateName::VISION: return K_VISION;
    case FSM_StateName::BACKFLIP: return K_BACKFLIP;
    case FSM_StateName::FRONTJUMP: return K_FRONTJUMP;
    case FSM_StateName::JOINT_PD: return K_JOINT_PD;
    case FSM_StateName::IMPEDANCE_CONTROL: return K_IMPEDANCE_CONTROL;
    default: return K_INVALID;
  }
}

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters, 
                                      _visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {
  // Find the current gait schedule
  _gaitScheduler->step();

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}

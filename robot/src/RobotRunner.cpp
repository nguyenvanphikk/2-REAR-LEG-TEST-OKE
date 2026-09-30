/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>
#include <chrono>

#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name),
  _lcm(getLcmUrl(255)) {

    _robot_ctrl = robot_ctrl;
  }

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  if (robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  // Initialize the model and robot data
  _model = _quadruped.buildModel();
  // MIT_3HP bring-up: tam bo qua JPosInitializer. Robot o PASSIVE cho toi khi
  // nguoi dung chu dong yeu cau STAND_UP tu GUI. Khong nap trajectory YAML.
  _jpos_initializer = nullptr;

  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(false);

  memset(&rc_control, 0, sizeof(rc_control_settings));
  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,
        &rc_control,
        controlParameters,
        &_stateEstimate,
        controlParameters->controller_dt);

  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_visualizationData= visualizationData;
  _robot_ctrl->_robotType = robotType;
  _robot_ctrl->_driverCommand = driverCommand;
  _robot_ctrl->_controlParameters = controlParameters;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  //cheetahMainVisualization->p = _stateEstimate.position;
  visualizationData->clear();

  // Update the data from the robot
  setupStep();

  _safetyReason = 0;

  // Khi dieu khien bang GUI (use_rc = 0), mat heartbeat qua 300 ms se dua
  // robot ve PASSIVE. Tai PASSIVE, LegController gui flags = 0 xuong STM32.
  if (!controlParameters->use_rc && lastGuiCommandUs) {
    const uint64_t nowUs =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    const uint64_t lastUs = lastGuiCommandUs->load(std::memory_order_relaxed);
    _guiWatchdogOk = lastUs != 0 && nowUs - lastUs <= 300000;
    if (!_guiWatchdogOk) {
      _safetyReason = 1;
      driverCommand->zero();
      controlParameters->control_mode = 0;
    }
  } else {
    _guiWatchdogOk = true;
  }

  // Lay so lieu SPI de hien thi tren GUI; khong doi FSM theo SPI nhu code goc.
  if (spiHealth && spiHealthMutex) {
    {
      std::lock_guard<std::mutex> lock(*spiHealthMutex);
      _spiHealthSnapshot[0] = spiHealth[0];
      _spiHealthSnapshot[1] = spiHealth[1];
    }
    // Thong ke SPI chi de quan sat trong phep thu hai chan truoc; khong dung
    // tuoi response SPI de tu chuyen FSM ve PASSIVE.
  }

  static int count_ini(0);
  ++count_ini;
  if (count_ini < 10) {
    _legController->setEnabled(false);
  } else if (20 < count_ini && count_ini < 30) {
    _legController->setEnabled(false);
  } else if (40 < count_ini && count_ini < 50) {
    _legController->setEnabled(false);
  } else {
    _legController->setEnabled(true);

    if( (rc_control.mode == 0) && controlParameters->use_rc ) {
      if(count_ini%1000 ==0)   printf("ESTOP!\n");
      for (int leg = 0; leg < 4; leg++) {
        _legController->commands[leg].zero();
      }
      // Bao dam nhanh enable gui xuong STM32 bang 0 trong nhanh E-stop.
      _legController->setEnabled(false);
      _robot_ctrl->Estop();
    }else {
      // Chi chay pha JPos neu sau nay chu dong khoi phuc initializer.
      // Hien tai vao FSM ngay; PASSIVE ep flags=0 den khi GUI yeu cau STAND_UP.
      if (_jpos_initializer && !_jpos_initializer->IsInitialized(_legController)) {
        Mat3<float> kpMat;
        Mat3<float> kdMat;
        if (robotType == RobotType::MINI_CHEETAH) {
          kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
          kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
        } else if (robotType == RobotType::CHEETAH_3) {
          kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
          kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        } else {
          assert(false);
        }

        for (int leg = 0; leg < 4; leg++) {
          _legController->commands[leg].kpJoint = kpMat;
          _legController->commands[leg].kdJoint = kdMat;
        }
      } else {
        _robot_ctrl->runController();
        cheetahMainVisualization->p = _stateEstimate.position;

        // Update Visualization
        _robot_ctrl->updateVisualization();
        cheetahMainVisualization->p = _stateEstimate.position;
      }
    }

  }



  // Visualization (will make this into a separate function later)
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      cheetahMainVisualization->q[leg * 3 + joint] =
        _legController->datas[leg].q[joint];
    }
  }
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->p = _stateEstimate.position;
  cheetahMainVisualization->quat = _stateEstimate.orientation;

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  // Update the leg data
  if (robotType == RobotType::MINI_CHEETAH) {
    // Robot that dung chung SpiData voi SPI task. Giu khoa trong thoi gian
    // updateData() doc tron goi, tranh tron response cua hai chu ky khac nhau.
    if (spiDataMutex) {
      std::lock_guard<std::mutex> lock(*spiDataMutex);
      _legController->updateData(spiData);
    } else {
      _legController->updateData(spiData);
    }
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateData(tiBoardData);
  } else {
    assert(false);
  }

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // state estimator
  // check transition to cheater mode:
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }

  get_rc_control_settings(&rc_control);

  // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    const auto writeSpiCommand = [&]() {
      _legController->updateCommand(spiCommand);
      // Phep thu STAND_UP tam thoi chi bat hai chan truoc (FR=0, FL=1).
      // HR/HL van duoc doc feedback nhung khong vao mode co luc.
      if (_robot_ctrl->getControllerMode() == 1) {
        for (int leg = 2; leg < 4; ++leg) {
          spiCommand->flags[leg] = 0;
          spiCommand->kp_abad[leg] = spiCommand->kp_hip[leg] = spiCommand->kp_knee[leg] = 0;
          spiCommand->kd_abad[leg] = spiCommand->kd_hip[leg] = spiCommand->kd_knee[leg] = 0;
          spiCommand->tau_abad_ff[leg] = spiCommand->tau_hip_ff[leg] = spiCommand->tau_knee_ff[leg] = 0;
        }
      }
    };
    // Tao tron goi command trong mot lan khoa ngan. SPI task chi lay snapshot
    // sau khi updateCommand() da ghi xong ca 12 khop.
    if (spiDataMutex) {
      std::lock_guard<std::mutex> lock(*spiDataMutex);
      writeSpiCommand();
    } else {
      writeSpiCommand();
    }
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else {
    assert(false);
  }
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
  if ((_iterations % 10) == 0) {
    mit3hp_status_lcm.timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    mit3hp_status_lcm.fsm_state = _robot_ctrl->getControllerMode();
    mit3hp_status_lcm.requested_mode =
        static_cast<int32_t>(controlParameters->control_mode);
    // FSM STAND_UP hien la pha xem truoc moc 0, khong tuong duong motor ON.
    mit3hp_status_lcm.motors_enabled = false;
    for (int leg = 0; leg < 4; ++leg)
      mit3hp_status_lcm.motors_enabled |= (spiCommand->flags[leg] & 1) != 0;
    mit3hp_status_lcm.gui_watchdog_ok = _guiWatchdogOk;
    mit3hp_status_lcm.safety_reason = _safetyReason;
    for (int board = 0; board < 2; board++) {
      mit3hp_status_lcm.spi_tx_frames[board] =
          _spiHealthSnapshot[board].transmitted_frames;
      mit3hp_status_lcm.spi_tx_flags[board] =
          _spiHealthSnapshot[board].last_tx_flags;
      mit3hp_status_lcm.spi_success[board] =
          _spiHealthSnapshot[board].successful_transfers;
      mit3hp_status_lcm.spi_ioctl_errors[board] =
          _spiHealthSnapshot[board].ioctl_errors;
      mit3hp_status_lcm.spi_short_transfers[board] =
          _spiHealthSnapshot[board].incomplete_transfers;
      mit3hp_status_lcm.spi_checksum_errors[board] =
          _spiHealthSnapshot[board].checksum_errors;
      mit3hp_status_lcm.spi_all_zero_responses[board] =
          _spiHealthSnapshot[board].all_zero_responses;
      mit3hp_status_lcm.spi_checksum_mismatches[board] =
          _spiHealthSnapshot[board].checksum_mismatches;
      mit3hp_status_lcm.spi_implausible_feedback[board] =
          _spiHealthSnapshot[board].implausible_feedback;
      mit3hp_status_lcm.spi_last_success_us[board] =
          _spiHealthSnapshot[board].last_success_us;
    }
    _lcm.publish("mit3hp_status", &mit3hp_status_lcm);
  }
  _iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);
  if (cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
  } else {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  }
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}

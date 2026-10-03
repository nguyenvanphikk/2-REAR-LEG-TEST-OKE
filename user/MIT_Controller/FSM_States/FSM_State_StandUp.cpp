/*============================= Stand Up ==============================*/
/**
 * Suspended-robot rear-leg position and air-gait test. No MPC or WBC.
 */

#include "FSM_State_StandUp.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP") {
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_StandUp<T>::onEnter() {
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  _stage = 1;
  _fault = 0;
  _elapsed = _settled = _phase = 0;
  Vec3<T> prepare;
  prepare << T(0), T(0), T(0.02);
  beginMove(1, prepare);
}

template <typename T>
void FSM_State_StandUp<T>::beginMove(int stage, const Vec3<T>& target) {
  _stage = stage;
  _elapsed = _settled = 0;
  _moveDuration = stage == 1 ? T(10) : T(15);
  for (int i = 0; i < 2; ++i) {
    _start[i] = this->_data->_legController->datas[i + 2].q;
    _lastCommand[i] = _start[i];
    _target[i] = target;
    // Smoothstep peaks at 1.5 * travel / duration. Keep peak below 0.12 rad/s.
    _moveDuration = std::max(_moveDuration,
        (_target[i] - _start[i]).cwiseAbs().maxCoeff() * T(12.5));
  }
}

template <typename T>
void FSM_State_StandUp<T>::fail(int reason) {
  std::printf("[REAR TEST] stop fault=%d stage=%d elapsed=%.3f s move_duration=%.3f s\n",
      reason, _stage, double(_elapsed), double(_moveDuration));
  _fault = reason;
  _stage = 7;
  this->_data->_legController->setEnabled(false);
  this->_data->controlParameters->control_mode = K_PASSIVE;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_StandUp<T>::run() {
  const T dt = std::max(T(0.001), std::min(T(0.01),
      T(this->_data->controlParameters->controller_dt)));
  // Suspended test: 15 times faster than the previous 8-second cycle.
  const T gaitPeriod = T(8) / T(15);
  const T swingTime = gaitPeriod / T(2);
  const auto* pad = this->_data->_desiredStateCommand->gamepadCommand;
  Vec3<T> pose;
  pose << T(0), T(-0.8), T(1.6);
  if (_stage == 2 && pad->y) beginMove(3, pose);
  if (_stage == 4 && pad->b) {
    _stage = 5;
    _phase = 0;
    for (int i = 0; i < 2; ++i)
      computeLegJacobianAndPosition(*this->_data->_quadruped, pose,
          static_cast<Mat3<T>*>(nullptr), &_standFoot[i], i + 2);
  }

  this->_data->_legController->setEnabled(true);
  for (int leg = 0; leg < 4; ++leg) {
    auto& command = this->_data->_legController->commands[leg];
    command.qDes.setZero();
    command.qdDes.setZero();
    command.kpJoint.setZero();
    command.kdJoint.setZero();
    if (leg >= 2) {
      command.kpJoint.diagonal().setConstant(T(8));
      command.kdJoint.diagonal().setConstant(T(0.4));
    }
    command.kpCartesian.setZero();
    command.kdCartesian.setZero();
    command.tauFeedForward.setZero();
    command.forceFeedForward.setZero();
  }

  _elapsed += dt;
  for (int i = 0; i < 2; ++i) {
    const Vec3<T> measured = this->_data->_legController->datas[i + 2].q;
    const Vec3<T> velocity = this->_data->_legController->datas[i + 2].qd;
    // Joint angle limits are handled by the flashed STM32 firmware.
    // During this diagnostic test, only reject non-finite joint feedback.
    for (int joint = 0; joint < 3; ++joint) {
      int reason = 0;
      if (!std::isfinite(measured[joint]) || !std::isfinite(velocity[joint]))
        reason = 10;
      if (reason) {
        const char* joints[] = {"abad", "hip", "knee"};
        std::printf("[REAR TEST] %s %s fault=%d q=%.6f rad qd=%.6f rad/s command=%.6f rad\n",
            i == 0 ? "HR" : "HL", joints[joint], reason,
            double(measured[joint]), double(velocity[joint]),
            double(_lastCommand[i][joint]));
        fail(reason + i * 3 + joint);
        return;
      }
    }

    Vec3<T> desired = _target[i];
    if (_stage == 1 || _stage == 3) {
      const T u = std::min(T(1), _elapsed / _moveDuration);
      const T smooth = u * u * (T(3) - T(2) * u);
      desired = _start[i] + smooth * (_target[i] - _start[i]);
    } else if (_stage >= 5 && _stage <= 6) {
      // Original trot phase: HR=0.5, HL=0.0. Bezier swing is shared with MPC,
      // but this suspended test does not use ground contact or MPC forces.
      const T stick = T(pad->leftStickAnalog[1]);
      const T effort = std::min(T(1), std::abs(stick) / T(0.2));
      const bool moving = std::abs(stick) > T(0.02);
      _stage = moving ? 6 : 5;
      desired = pose;
      if (moving) {
        const T legPhase = std::fmod(_phase + (i == 0 ? T(0.5) : T(0)), T(1));
        const T halfStride = T(0.005) * effort * (stick > 0 ? T(1) : T(-1));
        Vec3<T> p0 = _standFoot[i], pf = _standFoot[i], foot;
        FootSwingTrajectory<T> swing;
        if (legPhase < T(0.5)) {
          p0[0] += halfStride;
          pf[0] -= halfStride;
          swing.setHeight(T(0));
          swing.setInitialPosition(p0);
          swing.setFinalPosition(pf);
          swing.computeSwingTrajectoryBezier(legPhase * T(2), swingTime);
        } else {
          p0[0] -= halfStride;
          pf[0] += halfStride;
          swing.setHeight(T(0.06));
          swing.setInitialPosition(p0);
          swing.setFinalPosition(pf);
          swing.computeSwingTrajectoryBezier((legPhase - T(0.5)) * T(2), swingTime);
        }
        foot = swing.getPosition();
        Vec3<T> q = _lastCommand[i];
        for (int iter = 0; iter < 32; ++iter) {
          Mat3<T> J;
          Vec3<T> p;
          computeLegJacobianAndPosition(*this->_data->_quadruped, q, &J, &p, i + 2);
          const Mat3<T> regularized = J * J.transpose() + T(0.0001) * Mat3<T>::Identity();
          q += J.transpose() * regularized.ldlt().solve(foot - p);
        }
        Vec3<T> reachedFoot;
        computeLegJacobianAndPosition(*this->_data->_quadruped, q,
            static_cast<Mat3<T>*>(nullptr), &reachedFoot, i + 2);
        if (!q.allFinite() || !reachedFoot.allFinite() ||
            (reachedFoot - foot).norm() > T(0.003)) {
          fail(2);
          return;
        }
        desired = q;
      }
    }

    // Follow the smooth Bezier directly during trot; retain the slow rate
    // limit for prepare, move and returning to pose after joystick release.
    const T maxStep = T(0.15) * dt;
    for (int joint = 0; joint < 3; ++joint)
      if (_stage == 6)
        _lastCommand[i][joint] = desired[joint];
      else
        _lastCommand[i][joint] += std::max(-maxStep,
            std::min(maxStep, desired[joint] - _lastCommand[i][joint]));
    this->_data->_legController->commands[i + 2].qDes = _lastCommand[i];
  }

  if (_stage == 1 || _stage == 3) {
    if (_elapsed >= _moveDuration) {
      bool atTarget = true;
      constexpr T kSettledTolerance = T(6.5 * 3.14159265358979323846 / 180.0);
      for (int i = 0; i < 2; ++i)
        atTarget &= (this->_data->_legController->datas[i + 2].q - _target[i])
                        .cwiseAbs().maxCoeff() < kSettledTolerance;
      _settled = atTarget ? _settled + dt : T(0);
      if (_settled >= T(0.5)) _stage = _stage == 1 ? 2 : 4;
      // Keep holding the target while waiting for both legs; no move timeout.
    }
  }
  if (_stage == 6) _phase = std::fmod(_phase + dt / gaitPeriod, T(1));
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
  _stage = 0;
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;

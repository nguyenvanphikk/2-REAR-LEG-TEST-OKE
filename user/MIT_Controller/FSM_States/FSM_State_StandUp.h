#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"
#include "Controllers/FootSwingTrajectory.h"

/**
 *
 */
template <typename T>
class FSM_State_StandUp : public FSM_State<T> {
 public:
  FSM_State_StandUp(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();
  int rearTestStage() const { return _stage; }
  int rearTestFault() const { return _fault; }

 private:
  // 1=prepare, 2=prepared, 3=move to pose, 4=pose held,
  // 5=air trot idle, 6=air trot moving, 7=fault.
  int _stage = 0;
  int _fault = 0;
  T _elapsed = 0;
  T _moveDuration = 0;
  T _settled = 0;
  T _phase = 0;
  Vec3<T> _start[2];
  Vec3<T> _target[2];
  Vec3<T> _lastCommand[2];
  Vec3<T> _standFoot[2];
  void beginMove(int stage, const Vec3<T>& target);
  void fail(int reason);
};

#endif  // FSM_STATE_STANDUP_H

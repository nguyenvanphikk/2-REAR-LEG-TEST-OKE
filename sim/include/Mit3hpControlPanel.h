#ifndef MIT3HP_CONTROL_PANEL_H
#define MIT3HP_CONTROL_PANEL_H

#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <QTableWidget>
#include <QTimer>
#include <QPointF>
#include <QWidget>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <lcm-cpp.hpp>

#include "cheetah_visualization_lcmt.hpp"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "mit3hp_status_lcmt.hpp"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "spi_data_t.hpp"
#include "vectornav_lcmt.hpp"

class VirtualJoystick : public QWidget {
 public:
  explicit VirtualJoystick(QWidget* parent = nullptr);
  QPointF value() const { return _value; }
  void center();

 protected:
  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent*) override;

 private:
  void updateFromMouse(const QPointF& position);
  QPointF _value{0., 0.};
};

class Mit3hpControlPanel : public QMainWindow {
  Q_OBJECT

 public:
  explicit Mit3hpControlPanel(QWidget* parent = nullptr);
  ~Mit3hpControlPanel() override;

 private slots:
  void publishCommand();
  void refreshUi();
  void requestPassive();
  void requestStandUp();
  void requestRearPose();
  void requestAirTrot();
  void requestBalance();
  void requestLocomotion();
  void resetMotion();
  void gaitChanged(int index);

 private:
  static constexpr int kPassive = 0;
  static constexpr int kStandUp = 1;
  static constexpr int kBalanceStand = 3;
  static constexpr int kLocomotion = 4;

  void buildUi();
  void sendRobotDouble(const char* name, double value);
  void sendRobotInteger(const char* name, int64_t value);
  void sendUserDouble(const char* name, double value);
  void setRequestedMode(int mode, const QString& name);
  void receiveLoop();
  static int64_t nowMs();

  void handleSpi(const lcm::ReceiveBuffer*, const std::string&,
                 const spi_data_t* msg);
  void handleImu(const lcm::ReceiveBuffer*, const std::string&,
                 const vectornav_lcmt* msg);
  void handleVisualization(const lcm::ReceiveBuffer*, const std::string&,
                           const cheetah_visualization_lcmt* msg);
  void handleParameterResponse(const lcm::ReceiveBuffer*, const std::string&,
                               const control_parameter_respones_lcmt* msg);
  void handleRobotStatus(const lcm::ReceiveBuffer*, const std::string&,
                         const mit3hp_status_lcmt* msg);
  void handleLegCommand(const lcm::ReceiveBuffer*, const std::string&,
                        const leg_control_command_lcmt* msg);
  void handleLegData(const lcm::ReceiveBuffer*, const std::string&,
                     const leg_control_data_lcmt* msg);

  lcm::LCM _txLcm;
  lcm::LCM _rxLcm;
  std::thread _rxThread;
  std::atomic<bool> _running{true};
  uint64_t _requestNumber = 0;
  int _requestedMode = kPassive;
  int _rearRequestedStage = 0;
  int _rearActualStage = 0;
  int _rearFault = 0;
  QString _requestedModeName = "PASSIVE";
  QString _lastAutoPassiveReason;
  int64_t _lastAutoPassiveMs = 0;

  std::mutex _dataMutex;
  spi_data_t _spi{};
  vectornav_lcmt _imu{};
  int64_t _lastSpiMs = 0;
  int64_t _lastImuMs = 0;
  int64_t _lastControllerMs = 0;
  int64_t _lastAckMs = 0;
  int64_t _lastStatusMs = 0;
  int _actualMode = kPassive;
  bool _motorsEnabled = false;
  bool _watchdogOk = false;
  int _safetyReason = 0;
  int64_t _spiTxFrames[2]{0, 0};
  int32_t _spiTxFlags[2]{0, 0};
  int64_t _spiSuccess[2]{0, 0};
  int64_t _spiIoctlErrors[2]{0, 0};
  int64_t _spiShortTransfers[2]{0, 0};
  int64_t _spiChecksumErrors[2]{0, 0};
  int64_t _spiAllZeroResponses[2]{0, 0};
  int64_t _spiChecksumMismatches[2]{0, 0};
  int64_t _spiImplausibleFeedback[2]{0, 0};
  int64_t _spiLastSuccessUs[2]{0, 0};
  int64_t _previousSpiTxFrames[2]{0, 0};
  int64_t _previousSpiSuccess[2]{0, 0};
  double _txRateHz[2]{0., 0.};
  double _validResponseRateHz[2]{0., 0.};
  int64_t _previousRateSampleMs = 0;
  int64_t _errorWindowStartMs[2]{0, 0};
  int64_t _errorWindowSuccess[2]{0, 0};
  int64_t _errorWindowIoctl[2]{0, 0};
  int64_t _errorWindowShort[2]{0, 0};
  int64_t _errorWindowRejected[2]{0, 0};
  double _errorPercent[2]{0., 0.};
  bool _errorPercentValid[2]{false, false};
  leg_control_command_lcmt _legCommand{};
  leg_control_data_lcmt _legData{};
  int64_t _lastLegCommandMs = 0;
  int64_t _lastLegDataMs = 0;

  QLabel* _controllerStatus = nullptr;
  QLabel* _spiStatus = nullptr;
  QLabel* _imuStatus = nullptr;
  QLabel* _modeStatus = nullptr;
  QLabel* _commandStatus = nullptr;
  QLabel* _imuValues = nullptr;
  QLabel* _batteryStatus = nullptr;
  QLabel* _boardStatus[2]{nullptr, nullptr};
  QTableWidget* _motorTable = nullptr;
  QPushButton* _standButton = nullptr;
  QPushButton* _rearPoseButton = nullptr;
  QPushButton* _airTrotButton = nullptr;
  QLabel* _rearStageStatus = nullptr;
  QPushButton* _balanceButton = nullptr;
  QPushButton* _walkButton = nullptr;
  VirtualJoystick* _moveStick = nullptr;
  VirtualJoystick* _turnStick = nullptr;
  QSlider* _commandLimit = nullptr;
  QComboBox* _gaitSelector = nullptr;
  QTimer _publishTimer;
  QTimer _uiTimer;
};

#endif

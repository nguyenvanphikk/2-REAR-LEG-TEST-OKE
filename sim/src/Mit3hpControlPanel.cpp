#include "Mit3hpControlPanel.h"

#include <QApplication>
#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QPixmap>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QDebug>
#include <cstring>
#include <cmath>

#include "ControlParameters/ControlParameterInterface.h"
#include "Utilities/utilities.h"

namespace {
QLabel* statusLabel(const QString& text) {
  auto* label = new QLabel(text);
  label->setMinimumWidth(180);
  label->setWordWrap(true);
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("padding: 8px; border-radius: 5px; background:#555; color:white;");
  return label;
}

void setHealth(QLabel* label, const QString& name, int64_t ageMs,
               int64_t warnMs, int64_t failMs) {
  QString color = "#238636";
  QString state = QString("OK (%1 ms)").arg(ageMs);
  if (ageMs < 0 || ageMs > failMs) {
    color = "#b42318";
    state = "DISCONNECTED";
  } else if (ageMs > warnMs) {
    color = "#b7791f";
    state = QString("SLOW (%1 ms)").arg(ageMs);
  }
  label->setText(name + "\n" + state);
  label->setStyleSheet(QString("padding:8px;border-radius:5px;background:%1;color:white;font-weight:bold;").arg(color));
}

}

VirtualJoystick::VirtualJoystick(QWidget* parent) : QWidget(parent) {
  setMinimumSize(230, 230);
  setMaximumSize(320, 320);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void VirtualJoystick::center() {
  _value = QPointF(0., 0.);
  update();
}

void VirtualJoystick::updateFromMouse(const QPointF& position) {
  const QPointF c(width() / 2., height() / 2.);
  QPointF delta = position - c;
  const double radius = qMax(1., qMin(width(), height()) / 2. - 22.);
  const double length = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
  if (length > radius) delta *= radius / length;
  _value.setX(delta.x() / radius);
  _value.setY(-delta.y() / radius);  // Keo len tren la gia tri duong.
  update();
}

void VirtualJoystick::mousePressEvent(QMouseEvent* event) {
  updateFromMouse(event->localPos());
}

void VirtualJoystick::mouseMoveEvent(QMouseEvent* event) {
  if (event->buttons() & Qt::LeftButton) updateFromMouse(event->localPos());
}

void VirtualJoystick::mouseReleaseEvent(QMouseEvent*) { center(); }

void VirtualJoystick::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  const QPointF c(width() / 2., height() / 2.);
  const double radius = qMin(width(), height()) / 2. - 12.;
  p.setPen(QPen(QColor("#6b7280"), 2));
  p.setBrush(QColor("#1f2937"));
  p.drawEllipse(c, radius, radius);
  p.setPen(QPen(QColor("#4b5563"), 1));
  p.drawLine(QPointF(c.x() - radius, c.y()), QPointF(c.x() + radius, c.y()));
  p.drawLine(QPointF(c.x(), c.y() - radius), QPointF(c.x(), c.y() + radius));
  const QPointF knob(c.x() + _value.x() * (radius - 16.),
                      c.y() - _value.y() * (radius - 16.));
  p.setPen(QPen(QColor("#d1d5db"), 2));
  p.setBrush(QColor("#2563eb"));
  p.drawEllipse(knob, 20., 20.);
}

Mit3hpControlPanel::Mit3hpControlPanel(QWidget* parent)
    : QMainWindow(parent), _txLcm(getLcmUrl(255)), _rxLcm(getLcmUrl(255)) {
  setWindowTitle("MIT_3HP - REAL ROBOT CONTROL");
  resize(1200, 800);
  buildUi();

  if (!_txLcm.good() || !_rxLcm.good()) {
    QMessageBox::critical(this, "LCM", "Could not initialize LCM.");
  }
  _rxLcm.subscribe("spi_data", &Mit3hpControlPanel::handleSpi, this);
  _rxLcm.subscribe("hw_vectornav", &Mit3hpControlPanel::handleImu, this);
  _rxLcm.subscribe("main_cheetah_visualization",
                   &Mit3hpControlPanel::handleVisualization, this);
  _rxLcm.subscribe("interface_response",
                   &Mit3hpControlPanel::handleParameterResponse, this);
  _rxLcm.subscribe("mit3hp_status",
                   &Mit3hpControlPanel::handleRobotStatus, this);
  _rxLcm.subscribe("leg_control_command",
                   &Mit3hpControlPanel::handleLegCommand, this);
  _rxLcm.subscribe("leg_control_data",
                   &Mit3hpControlPanel::handleLegData, this);
  _rxThread = std::thread(&Mit3hpControlPanel::receiveLoop, this);

  connect(&_publishTimer, &QTimer::timeout, this,
          &Mit3hpControlPanel::publishCommand);
  connect(&_uiTimer, &QTimer::timeout, this, &Mit3hpControlPanel::refreshUi);
  _publishTimer.start(50);   // 20 Hz heartbeat; watchdog robot la 300 ms.
  _uiTimer.start(100);

  // GUI thay tay cam SBUS. Lenh nay cung xu ly truong hop YAML cu van use_rc=1.
  sendRobotInteger("use_rc", 0);
  requestPassive();
}

Mit3hpControlPanel::~Mit3hpControlPanel() {
  requestPassive();
  publishCommand();
  _running.store(false);
  if (_rxThread.joinable()) _rxThread.join();
}

int64_t Mit3hpControlPanel::nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Mit3hpControlPanel::buildUi() {
  auto* central = new QWidget;
  auto* root = new QVBoxLayout(central);
  auto* brand = new QWidget;
  brand->setStyleSheet("background:white; border:1px solid #e2e8f0; border-radius:8px;");
  auto* brandRow = new QHBoxLayout(brand);
  brandRow->setContentsMargins(18, 8, 18, 8);
  auto* logo = new QLabel;
  logo->setPixmap(QPixmap(":/mit3hp/logo.png").scaled(
      190, 76, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  logo->setFixedSize(190, 76);
  logo->setAlignment(Qt::AlignCenter);
  logo->setStyleSheet("border:0; background:white;");
  brandRow->addWidget(logo);
  auto* title = new QLabel("MIT_3HP  |  REAL ROBOT CONTROL\nMotor testing and telemetry");
  title->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  title->setStyleSheet("font-size:20px;font-weight:bold;color:#172033;border:0;background:white;");
  brandRow->addWidget(title, 1);
  root->addWidget(brand);

  auto* health = new QHBoxLayout;
  _controllerStatus = statusLabel("CONTROLLER\nDISCONNECTED");
  _spiStatus = statusLabel("SPI\nNO VALID RESPONSE");
  _imuStatus = statusLabel("VECTORNAV\nDISCONNECTED");
  _modeStatus = statusLabel("ROBOT STATE\nPASSIVE");
  health->addWidget(_controllerStatus);
  health->addWidget(_spiStatus);
  health->addWidget(_imuStatus);
  health->addWidget(_modeStatus);
  root->addLayout(health);

  auto* tabs = new QTabWidget;
  root->addWidget(tabs, 1);

  auto* control = new QWidget;
  auto* controlLayout = new QVBoxLayout(control);
  auto* emergency = new QPushButton("E-STOP / PASSIVE / DISABLE MOTOR");
  emergency->setMinimumHeight(72);
  emergency->setStyleSheet("font-size:20px;font-weight:bold;background:#b42318;color:white;");
  connect(emergency, &QPushButton::clicked, this,
          &Mit3hpControlPanel::requestPassive);
  controlLayout->addWidget(emergency);

  auto* modes = new QHBoxLayout;
  _standButton = new QPushButton("1. PREPARE HR/HL [0, 0, 0.02]");
  _balanceButton = new QPushButton("2. BALANCE");
  _walkButton = new QPushButton("3. LOCOMOTION");
  for (auto* b : {_standButton, _balanceButton, _walkButton}) {
    b->setMinimumHeight(50);
    modes->addWidget(b);
  }
  connect(_standButton, &QPushButton::clicked, this,
          &Mit3hpControlPanel::requestStandUp);
  connect(_balanceButton, &QPushButton::clicked, this,
          &Mit3hpControlPanel::requestBalance);
  connect(_walkButton, &QPushButton::clicked, this,
          &Mit3hpControlPanel::requestLocomotion);
  controlLayout->addLayout(modes);
  auto* rearStages = new QHBoxLayout;
  _rearPoseButton = new QPushButton("2. MOVE HR/HL TO [0, -0.8, 1.6]");
  _airTrotButton = new QPushButton("3. AIR TROT (SUSPENDED ONLY)");
  rearStages->addWidget(_rearPoseButton);
  rearStages->addWidget(_airTrotButton);
  controlLayout->addLayout(rearStages);
  connect(_rearPoseButton, &QPushButton::clicked, this,
          &Mit3hpControlPanel::requestRearPose);
  connect(_airTrotButton, &QPushButton::clicked, this,
          &Mit3hpControlPanel::requestAirTrot);
  _rearStageStatus = new QLabel("Rear test: PASSIVE");
  _rearStageStatus->setStyleSheet("font-weight:bold;color:#172033;");
  controlLayout->addWidget(_rearStageStatus);
  auto* startupNote = new QLabel(
      "SUSPENDED REAR-LEG TEST: Prepare slowly, confirm, move to pose slowly, "
      "confirm, then use the movement stick for a small air trot. "
      "FR/FL motors remain off. VectorNav is not used by this test. "
      "SPI and GUI heartbeat warnings do not automatically stop this diagnostic test. "
      "Keep a physical motor power cut within reach. "
      "Balance and locomotion are locked.");
  startupNote->setWordWrap(true);
  startupNote->setStyleSheet("color:#b42318;font-weight:bold;");
  controlLayout->addWidget(startupNote);

  auto* gaitRow = new QHBoxLayout;
  gaitRow->addWidget(new QLabel("MPC gait (locked):"));
  _gaitSelector = new QComboBox;
  _gaitSelector->addItem("Trotting (recommended)", 0);
  _gaitSelector->addItem("Standing", 4);
  _gaitSelector->addItem("Trot Running (not tested on robot)", 5);
  _gaitSelector->addItem("Walking (not tested on robot)", 6);
  _gaitSelector->addItem("Walking 2 (not tested on robot)", 7);
  _gaitSelector->addItem("Pacing (not tested on robot)", 8);
  _gaitSelector->setEnabled(false);
  connect(_gaitSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &Mit3hpControlPanel::gaitChanged);
  gaitRow->addWidget(_gaitSelector, 1);
  controlLayout->addLayout(gaitRow);

  auto* motionBox = new QGroupBox("Virtual joysticks (movement up/down controls suspended air trot)");
  auto* motionLayout = new QVBoxLayout(motionBox);
  auto* sticks = new QHBoxLayout;
  auto* moveColumn = new QVBoxLayout;
  auto* turnColumn = new QVBoxLayout;
  auto* moveTitle = new QLabel("MOVEMENT\nUp/down: forward/back  |  Left/right: lateral");
  auto* turnTitle = new QLabel("HEADING\nLeft/right: yaw  |  Up/down: body pitch");
  moveTitle->setAlignment(Qt::AlignCenter);
  turnTitle->setAlignment(Qt::AlignCenter);
  _moveStick = new VirtualJoystick;
  _turnStick = new VirtualJoystick;
  moveColumn->addWidget(moveTitle);
  moveColumn->addWidget(_moveStick, 0, Qt::AlignCenter);
  turnColumn->addWidget(turnTitle);
  turnColumn->addWidget(_turnStick, 0, Qt::AlignCenter);
  sticks->addLayout(moveColumn);
  sticks->addLayout(turnColumn);
  motionLayout->addLayout(sticks);
  _commandLimit = new QSlider(Qt::Horizontal);
  _commandLimit->setRange(5, 100);
  _commandLimit->setValue(20);
  auto* limitRow = new QHBoxLayout;
  limitRow->addWidget(new QLabel("Command limit (%)"));
  limitRow->addWidget(_commandLimit);
  motionLayout->addLayout(limitRow);
  auto* zero = new QPushButton("STOP MOTION - ZERO COMMAND");
  connect(zero, &QPushButton::clicked, this,
          &Mit3hpControlPanel::resetMotion);
  motionLayout->addWidget(zero);
  controlLayout->addWidget(motionBox);
  _commandStatus = new QLabel("vx=0  vy=0  yaw=0  pitch=0");
  _commandStatus->setAlignment(Qt::AlignCenter);
  controlLayout->addWidget(_commandStatus);
  tabs->addTab(control, "CONTROL");

  auto* motors = new QWidget;
  auto* motorsLayout = new QVBoxLayout(motors);
  auto* note = new QLabel(
      "Monitor only. Joint angles use Jetson sign/scale and encoder zero; physical calibration must be verified. "
      "Red cells show stale feedback; they do not confirm the current physical angle.");
  note->setWordWrap(true);
  note->setStyleSheet("color:#b42318;font-weight:bold;");
  motorsLayout->addWidget(note);
  _motorTable = new QTableWidget(12, 13);
  _motorTable->setHorizontalHeaderLabels(
      {"Joint", "Joint q [rad]", "Joint angle [deg / rad]", "qd [rad/s]", "q target [rad]",
       "qd target [rad/s]", "Estimated torque [Nm]", "Kp", "Kd",
       "Sign / scale", "Offset [rad]", "Gear ratio", "Feedback flags"});
  const char* legs[] = {"FR", "FL", "HR", "HL"};
  const char* joints[] = {"Abad", "Hip", "Knee"};
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++)
      _motorTable->setItem(leg * 3 + joint, 0,
          new QTableWidgetItem(QString("%1 %2").arg(legs[leg], joints[joint])));
  _motorTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  _motorTable->horizontalHeader()->setMinimumSectionSize(90);
  _motorTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
  _motorTable->setColumnWidth(2, 180);
  _motorTable->horizontalHeaderItem(1)->setToolTip(
      "Motor feedback converted by Jetson: q_joint = (q_STM32 - offset) * sign/scale. "
      "Abad/hip use +/-1; knee uses +/-1/1.5. No additional gearbox division is applied here.");
  _motorTable->horizontalHeaderItem(2)->setToolTip(
      "Same joint q as the adjacent column, shown in degrees and radians relative to encoder zero. "
      "This is a unit conversion, not a separate physical angle measurement.");
  _motorTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  motorsLayout->addWidget(_motorTable);
  tabs->addTab(motors, "MOTORS");

  auto* system = new QWidget;
  auto* systemLayout = new QVBoxLayout(system);
  _imuValues = new QLabel("VectorNav: no data yet");
  _imuValues->setStyleSheet("font-family:monospace;font-size:15px;");
  _batteryStatus = new QLabel("Motor supply voltage: not provided by the STM32 firmware");
  _boardStatus[0] = statusLabel("FRONT BOARD /dev/spidev1.0\nNo valid response");
  _boardStatus[1] = statusLabel("REAR BOARD /dev/spidev1.1\nNo valid response");
  for (auto* board : _boardStatus) {
    board->setMinimumHeight(138);
  }
  systemLayout->addWidget(_boardStatus[0]);
  systemLayout->addWidget(_boardStatus[1]);
  systemLayout->addWidget(_imuValues);
  systemLayout->addWidget(_batteryStatus);
  systemLayout->addStretch();
  tabs->addTab(system, "SYSTEM");

  auto* settings = new QWidget;
  auto* settingsLayout = new QVBoxLayout(settings);
  settingsLayout->addWidget(new QLabel(
      "Current setup:\n"
      "- GUI sends a 20 Hz heartbeat. During the rear diagnostic test, heartbeat loss does not stop motors.\n"
      "- Rear test: HR/HL only. No speed, tracking, move-timeout or SPI cutoff; non-finite feedback still stops motors.\n"
      "- IMU is optional only in this suspended test; run controller with MIT3HP_REAR_TEST_NO_IMU=1 when disconnected.\n"
      "- Balance, locomotion, recovery and jumping remain locked.\n"
      "- Green SPI means a recent valid response, not STM32 command acknowledgement."));
  settingsLayout->addStretch();
  tabs->addTab(settings, "SETTINGS");

  setCentralWidget(central);
  _balanceButton->setEnabled(false);
  _walkButton->setEnabled(false);
}

void Mit3hpControlPanel::sendRobotDouble(const char* name, double value) {
  control_parameter_request_lcmt msg{};
  msg.requestNumber = ++_requestNumber;
  msg.requestKind = static_cast<int8_t>(ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME);
  msg.parameterKind = static_cast<int8_t>(ControlParameterValueKind::DOUBLE);
  std::strncpy(reinterpret_cast<char*>(msg.name), name, sizeof(msg.name) - 1);
  ControlParameterValue v{};
  v.d = value;
  std::memcpy(msg.value, &v, sizeof(v));
  _txLcm.publish("interface_request", &msg);
}

void Mit3hpControlPanel::sendRobotInteger(const char* name, int64_t value) {
  control_parameter_request_lcmt msg{};
  msg.requestNumber = ++_requestNumber;
  msg.requestKind = static_cast<int8_t>(ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME);
  msg.parameterKind = static_cast<int8_t>(ControlParameterValueKind::S64);
  std::strncpy(reinterpret_cast<char*>(msg.name), name, sizeof(msg.name) - 1);
  ControlParameterValue v{};
  v.i = value;
  std::memcpy(msg.value, &v, sizeof(v));
  _txLcm.publish("interface_request", &msg);
}

void Mit3hpControlPanel::sendUserDouble(const char* name, double value) {
  control_parameter_request_lcmt msg{};
  msg.requestNumber = ++_requestNumber;
  msg.requestKind = static_cast<int8_t>(ControlParameterRequestKind::SET_USER_PARAM_BY_NAME);
  msg.parameterKind = static_cast<int8_t>(ControlParameterValueKind::DOUBLE);
  std::strncpy(reinterpret_cast<char*>(msg.name), name, sizeof(msg.name) - 1);
  ControlParameterValue v{};
  v.d = value;
  std::memcpy(msg.value, &v, sizeof(v));
  _txLcm.publish("interface_request", &msg);
}

void Mit3hpControlPanel::gaitChanged(int index) {
  if (index < 0) return;
  const int gait = _gaitSelector->itemData(index).toInt();
  sendUserDouble("cmpc_gait", gait);
}

void Mit3hpControlPanel::setRequestedMode(int mode, const QString& name) {
  resetMotion();
  if (mode != kPassive) {
    _lastAutoPassiveReason.clear();
    _lastAutoPassiveMs = 0;
  }
  _requestedMode = mode;
  _requestedModeName = name;
  sendRobotDouble("control_mode", mode);
  _modeStatus->setText("REQUESTED MODE\n" + name);
}

void Mit3hpControlPanel::requestPassive() {
  _rearRequestedStage = 0;
  resetMotion();
  setRequestedMode(kPassive, "PASSIVE");
  _balanceButton->setEnabled(false);
  _walkButton->setEnabled(false);
}

void Mit3hpControlPanel::requestStandUp() {
  if (QMessageBox::question(this, "Prepare suspended rear legs",
      "Is the robot suspended, with all six rear joints checked for direction and zero?\n"
      "HR/HL will move slowly from measured angles to [0, 0, 0.02] rad. "
      "FR/FL motors stay off. Keep the physical motor cut within reach.\nContinue?")
      != QMessageBox::Yes) return;
  _rearRequestedStage = 1;
  setRequestedMode(kStandUp, "STAND_UP");
  _balanceButton->setEnabled(false);
  _walkButton->setEnabled(false);
}

void Mit3hpControlPanel::requestRearPose() {
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    if (_rearActualStage != 2) return;
  }
  if (QMessageBox::question(this, "Move rear legs to pose",
      "HR/HL will move slowly (at least 15 seconds) to [0, -0.8, 1.6] rad. "
      "Confirm both legs are clear and suspended. Continue?") != QMessageBox::Yes) return;
  _rearRequestedStage = 2;
}

void Mit3hpControlPanel::requestAirTrot() {
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    if (_rearActualStage != 4) return;
  }
  if (QMessageBox::question(this, "Enable slow air trot",
      "HR/HL will alternate at an 8-second cycle only while the movement "
      "stick is held. Start with a very small stick deflection. Continue?")
      != QMessageBox::Yes) return;
  _rearRequestedStage = 3;
}

void Mit3hpControlPanel::requestBalance() {
  QMessageBox::information(this, "Locked",
      "Balance is locked during joint testing.");
}

void Mit3hpControlPanel::requestLocomotion() {
  QMessageBox::information(this, "Locked",
      "Locomotion is locked during joint testing.");
}

void Mit3hpControlPanel::resetMotion() {
  _moveStick->center();
  _turnStick->center();
}

void Mit3hpControlPanel::publishCommand() {
  gamepad_lcmt msg{};
  const float limit = _commandLimit ? _commandLimit->value() / 100.f : 0.f;
  if (_requestedMode == kStandUp) {
    msg.x = _rearRequestedStage == 1;
    msg.y = _rearRequestedStage == 2;
    msg.b = _rearRequestedStage == 3;
    if (_rearRequestedStage == 3)
      msg.leftStickAnalog[1] = _moveStick->value().y() * qMin(limit, 0.2f);
  }
  if (_requestedMode == kBalanceStand || _requestedMode == kLocomotion) {
    const QPointF move = _moveStick->value();
    const QPointF turn = _turnStick->value();
    msg.leftStickAnalog[0] = move.x() * limit;
    msg.leftStickAnalog[1] = move.y() * limit;
    msg.rightStickAnalog[0] = turn.x() * limit;
    msg.rightStickAnalog[1] = turn.y() * limit;
  }
  _txLcm.publish("interface", &msg);
  _commandStatus->setText(QString("left=(%1, %2)  right=(%3, %4)  limit=%5%")
      .arg(msg.leftStickAnalog[0], 0, 'f', 2)
      .arg(msg.leftStickAnalog[1], 0, 'f', 2)
      .arg(msg.rightStickAnalog[0], 0, 'f', 2)
      .arg(msg.rightStickAnalog[1], 0, 'f', 2)
      .arg(static_cast<int>(limit * 100)));
  const float vx = msg.leftStickAnalog[1] * 3.f;
  const float vy = -msg.leftStickAnalog[0] * 2.f;
  const float yawRate = -msg.rightStickAnalog[0] * 2.5f;
  const float pitch = msg.rightStickAnalog[1] * 0.4f;
  _commandStatus->setText(QString(
      "Target: vx=%1 m/s | vy=%2 m/s | yaw rate=%3 rad/s | pitch=%4 rad | limit=%5%")
      .arg(vx, 0, 'f', 2).arg(vy, 0, 'f', 2)
      .arg(yawRate, 0, 'f', 2).arg(pitch, 0, 'f', 2)
      .arg(static_cast<int>(limit * 100)));
}

void Mit3hpControlPanel::receiveLoop() {
  while (_running.load()) _rxLcm.handleTimeout(100);
}

void Mit3hpControlPanel::handleSpi(const lcm::ReceiveBuffer*, const std::string&,
                                  const spi_data_t* msg) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _spi = *msg;
  _lastSpiMs = nowMs();
}

void Mit3hpControlPanel::handleImu(const lcm::ReceiveBuffer*, const std::string&,
                                  const vectornav_lcmt* msg) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _imu = *msg;
  _lastImuMs = nowMs();
}

void Mit3hpControlPanel::handleVisualization(
    const lcm::ReceiveBuffer*, const std::string&,
    const cheetah_visualization_lcmt*) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _lastControllerMs = nowMs();
}

void Mit3hpControlPanel::handleParameterResponse(
    const lcm::ReceiveBuffer*, const std::string&,
    const control_parameter_respones_lcmt*) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _lastAckMs = nowMs();
}

void Mit3hpControlPanel::handleRobotStatus(
    const lcm::ReceiveBuffer*, const std::string&,
    const mit3hp_status_lcmt* msg) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _lastStatusMs = nowMs();
  _actualMode = msg->fsm_state;
  _motorsEnabled = msg->motors_enabled != 0;
  _watchdogOk = msg->gui_watchdog_ok != 0;
  _safetyReason = msg->safety_reason;
  _rearActualStage = msg->rear_test_stage;
  _rearFault = msg->rear_test_fault;
  for (int board = 0; board < 2; board++) {
    _spiTxFrames[board] = msg->spi_tx_frames[board];
    _spiTxFlags[board] = msg->spi_tx_flags[board];
    _spiSuccess[board] = msg->spi_success[board];
    _spiIoctlErrors[board] = msg->spi_ioctl_errors[board];
    _spiShortTransfers[board] = msg->spi_short_transfers[board];
    _spiChecksumErrors[board] = msg->spi_checksum_errors[board];
    _spiAllZeroResponses[board] = msg->spi_all_zero_responses[board];
    _spiChecksumMismatches[board] = msg->spi_checksum_mismatches[board];
    _spiImplausibleFeedback[board] = msg->spi_implausible_feedback[board];
    _spiLastSuccessUs[board] = msg->spi_last_success_us[board];
  }
}

void Mit3hpControlPanel::handleLegCommand(
    const lcm::ReceiveBuffer*, const std::string&,
    const leg_control_command_lcmt* msg) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _legCommand = *msg;
  _lastLegCommandMs = nowMs();
}

void Mit3hpControlPanel::handleLegData(
    const lcm::ReceiveBuffer*, const std::string&,
    const leg_control_data_lcmt* msg) {
  std::lock_guard<std::mutex> lock(_dataMutex);
  _legData = *msg;
  _lastLegDataMs = nowMs();
}

void Mit3hpControlPanel::refreshUi() {
  spi_data_t spi;
  vectornav_lcmt imu;
  int64_t imuMs, ctrlMs, statusMs;
  int actualMode, rearStage, rearFault;
  bool motorsEnabled, watchdogOk;
  int safetyReason;
  int64_t txFrames[2];
  int32_t txFlags[2];
  int64_t success[2], ioctlErrors[2], shortTransfers[2], checksumErrors[2];
  int64_t allZeroResponses[2], checksumMismatches[2], implausibleFeedback[2];
  int64_t lastSuccessUs[2];
  leg_control_command_lcmt legCommand;
  leg_control_data_lcmt legData;
  int64_t legCommandMs, legDataMs;
  {
    std::lock_guard<std::mutex> lock(_dataMutex);
    spi = _spi;
    imu = _imu;
    imuMs = _lastImuMs;
    ctrlMs = _lastControllerMs;
    statusMs = _lastStatusMs;
    actualMode = _actualMode;
    rearStage = _rearActualStage;
    rearFault = _rearFault;
    motorsEnabled = _motorsEnabled;
    watchdogOk = _watchdogOk;
    safetyReason = _safetyReason;
    for (int board = 0; board < 2; board++) {
      txFrames[board] = _spiTxFrames[board];
      txFlags[board] = _spiTxFlags[board];
      success[board] = _spiSuccess[board];
      ioctlErrors[board] = _spiIoctlErrors[board];
      shortTransfers[board] = _spiShortTransfers[board];
      checksumErrors[board] = _spiChecksumErrors[board];
      allZeroResponses[board] = _spiAllZeroResponses[board];
      checksumMismatches[board] = _spiChecksumMismatches[board];
      implausibleFeedback[board] = _spiImplausibleFeedback[board];
      lastSuccessUs[board] = _spiLastSuccessUs[board];
    }
    legCommand = _legCommand;
    legData = _legData;
    legCommandMs = _lastLegCommandMs;
    legDataMs = _lastLegDataMs;
  }
  const int64_t now = nowMs();
  setHealth(_controllerStatus, "CONTROLLER", ctrlMs ? now - ctrlMs : -1, 100, 500);
  const int64_t monotonicUs = now * 1000;
  const int64_t boardAgeMs[2] = {
      lastSuccessUs[0] ? (monotonicUs - lastSuccessUs[0]) / 1000 : -1,
      lastSuccessUs[1] ? (monotonicUs - lastSuccessUs[1]) / 1000 : -1};
  const int64_t worstSpiAge = boardAgeMs[0] < 0 || boardAgeMs[1] < 0
      ? -1 : qMax(boardAgeMs[0], boardAgeMs[1]);
  setHealth(_spiStatus, "SPI VALID RESPONSE 1.0 + 1.1", worstSpiAge, 100, 300);
  setHealth(_imuStatus, "VECTORNAV", imuMs ? now - imuMs : -1, 30, 300);
  if (!imuMs)
    _imuStatus->setText("VECTORNAV\nOPTIONAL FOR SUSPENDED REAR TEST");
  const char* modeNames[] = {"PASSIVE", "STAND_UP", "?", "BALANCE", "LOCOMOTION"};
  const QString actualName = actualMode >= 0 && actualMode <= 4
      ? modeNames[actualMode] : QString("MODE %1").arg(actualMode);
  _modeStatus->setText(QString("ACTUAL MODE: %1\nREQUESTED: %2 | MOTOR: %3")
      .arg(actualName, _requestedModeName, motorsEnabled ? "ON" : "OFF"));
  const char* rearNames[] = {"PASSIVE", "MOVING TO PREPARE", "PREPARED / HOLD",
      "MOVING TO POSE", "POSE / HOLD", "AIR TROT / IDLE",
      "AIR TROT / MOVING", "FAULT / PASSIVE"};
  QString rearFaultText = QString("Fault %1").arg(rearFault);
  for (int base : {10, 20, 30}) {
    if (rearFault >= base && rearFault < base + 6) {
      const int index = rearFault - base;
      const char* joints[] = {"ABAD", "HIP", "KNEE"};
      rearFaultText = QString("%1 %2: %3")
          .arg(index < 3 ? "HR" : "HL", joints[index % 3],
               base == 10 ? "INVALID FEEDBACK" :
               base == 20 ? "SPEED >1 rad/s FOR 100 ms" : "TRACKING ERROR >0.35 rad");
    }
  }
  _rearStageStatus->setText(QString("Rear test: %1%2")
      .arg(rearStage >= 0 && rearStage <= 7 ? rearNames[rearStage] : "UNKNOWN")
      .arg(rearFault ? " | " + rearFaultText : QString()));
  const bool autoPassiveRecent =
      _lastAutoPassiveMs && now - _lastAutoPassiveMs < 30000;
  if (autoPassiveRecent)
    _modeStatus->setText(_modeStatus->text() + "\nAUTO PASSIVE: " +
                         _lastAutoPassiveReason);
  if (rearFault && _requestedMode != kPassive) {
    _lastAutoPassiveReason = rearFaultText;
    _lastAutoPassiveMs = now;
    requestPassive();
  }

  const char* safetyNames[] = {"OK", "GUI LOST", "BOARD 1.0 ERROR",
                               "BOARD 1.1 ERROR", "BOTH BOARDS ERROR",
                               "REAR TEST SPI / FEEDBACK"};
  if (safetyReason >= 1 && safetyReason <= 5) {
    _modeStatus->setText(_modeStatus->text() + "\nSAFETY: " +
                         safetyNames[safetyReason]);
    _modeStatus->setStyleSheet(
        "padding:8px;border-radius:5px;background:#b42318;color:white;font-weight:bold;");
  } else if (autoPassiveRecent) {
    _modeStatus->setStyleSheet(
        "padding:8px;border-radius:5px;background:#b7791f;color:white;font-weight:bold;");
  } else {
    _modeStatus->setStyleSheet(
        "padding:8px;border-radius:5px;background:#238636;color:white;font-weight:bold;");
  }

  // Dùng cửa sổ 1 giây để số Hz ổn định hơn chu kỳ vẽ GUI 100 ms.
  const bool statusFreshForRate = statusMs && now - statusMs < 500;
  if (!statusFreshForRate) {
    _previousRateSampleMs = 0;
    for (int board = 0; board < 2; ++board) {
      _txRateHz[board] = 0.;
      _validResponseRateHz[board] = 0.;
    }
  } else if (_previousRateSampleMs == 0) {
    _previousRateSampleMs = now;
    for (int board = 0; board < 2; ++board) {
      _previousSpiTxFrames[board] = txFrames[board];
      _previousSpiSuccess[board] = success[board];
    }
  } else if (now - _previousRateSampleMs >= 1000) {
    const double seconds = (now - _previousRateSampleMs) / 1000.;
    for (int board = 0; board < 2; ++board) {
      const int64_t txDelta = txFrames[board] - _previousSpiTxFrames[board];
      const int64_t rxDelta = success[board] - _previousSpiSuccess[board];
      _txRateHz[board] = txDelta >= 0 ? txDelta / seconds : 0.;
      _validResponseRateHz[board] = rxDelta >= 0 ? rxDelta / seconds : 0.;
      _previousSpiTxFrames[board] = txFrames[board];
      _previousSpiSuccess[board] = success[board];
    }
    _previousRateSampleMs = now;
  }
  for (int board = 0; board < 2; board++) {
    // Cac bo dem la tong tu khi controller khoi dong. Tinh phan tram tren
    // cua so 5 giay de nguoi dung thay loi gan day, khong bi lech boi lich su.
    const bool statusFresh = statusMs && now - statusMs < 500;
    const bool countersReset = success[board] < _errorWindowSuccess[board] ||
        ioctlErrors[board] < _errorWindowIoctl[board] ||
        shortTransfers[board] < _errorWindowShort[board] ||
        checksumErrors[board] < _errorWindowRejected[board];
    if (!statusFresh) {
      _errorWindowStartMs[board] = 0;
      _errorPercentValid[board] = false;
    } else if (_errorWindowStartMs[board] == 0 || countersReset) {
      _errorWindowStartMs[board] = now;
      _errorWindowSuccess[board] = success[board];
      _errorWindowIoctl[board] = ioctlErrors[board];
      _errorWindowShort[board] = shortTransfers[board];
      _errorWindowRejected[board] = checksumErrors[board];
      _errorPercentValid[board] = false;
    } else if (now - _errorWindowStartMs[board] >= 5000) {
      const int64_t valid = success[board] - _errorWindowSuccess[board];
      const int64_t rejected =
          ioctlErrors[board] - _errorWindowIoctl[board] +
          shortTransfers[board] - _errorWindowShort[board] +
          checksumErrors[board] - _errorWindowRejected[board];
      _errorPercentValid[board] = valid + rejected > 0;
      if (_errorPercentValid[board])
        _errorPercent[board] = 100. * rejected / (valid + rejected);
      _errorWindowStartMs[board] = now;
      _errorWindowSuccess[board] = success[board];
      _errorWindowIoctl[board] = ioctlErrors[board];
      _errorWindowShort[board] = shortTransfers[board];
      _errorWindowRejected[board] = checksumErrors[board];
    }
    const QString errorPercentText = _errorPercentValid[board]
        ? QString::number(_errorPercent[board], 'f', 1) + "%" : "--";
    const bool ok = boardAgeMs[board] >= 0 && boardAgeMs[board] <= 100;
    const QString boardName = board == 0 ? "FRONT BOARD (FR/FL)" : "REAR BOARD (HR/HL)";
    _boardStatus[board]->setText(QString(
        "%1 | /dev/spidev1.%2 | %3\n"
        "132-byte transfers: %4 Hz (%5 total)  |  Valid responses: %6 Hz (%7 total)\n"
        "Last valid response: %8 ms ago  |  Rejected (last 5 s): %9\n"
        "Errors: ioctl %10 | short %11 | all-zero %12 | XOR %13 | implausible motor data %14\n"
        "TX motor-enable flags: ch0=%15, ch1=%16  |  STM32 command ACK: unavailable")
        .arg(boardName).arg(board).arg(ok ? "RESPONSE OK" : "NO VALID RESPONSE")
        .arg(_txRateHz[board], 0, 'f', 1).arg(txFrames[board])
        .arg(_validResponseRateHz[board], 0, 'f', 1).arg(success[board])
        .arg(boardAgeMs[board]).arg(errorPercentText)
        .arg(ioctlErrors[board]).arg(shortTransfers[board])
        .arg(allZeroResponses[board]).arg(checksumMismatches[board])
        .arg(implausibleFeedback[board])
        .arg(txFlags[board] & 0xff).arg((txFlags[board] >> 8) & 0xff));
    _boardStatus[board]->setStyleSheet(QString(
        "padding:8px;border-radius:5px;background:%1;color:white;font-weight:bold;")
        .arg(ok ? "#238636" : "#b42318"));
  }

  const bool hardwareReady = ctrlMs && statusMs &&
      now - ctrlMs < 500 &&
      now - statusMs < 500 && watchdogOk && safetyReason == 0;
  if (!hardwareReady && _requestedMode != kPassive && _requestedMode != kStandUp) {
    QString reason;
    const auto addReason = [&](const QString& part) {
      if (!reason.isEmpty()) reason += ", ";
      reason += part;
    };
    if (!ctrlMs || now - ctrlMs >= 500) addReason("CONTROLLER >500 ms");
    if (!statusMs || now - statusMs >= 500) addReason("STATUS >500 ms");
    if (!watchdogOk) addReason("WATCHDOG GUI");
    if (safetyReason != 0)
      addReason(QString("SAFETY=%1").arg(safetyReason));
    _lastAutoPassiveReason = reason.isEmpty() ? "UNKNOWN" : reason;
    _lastAutoPassiveMs = now;
    qWarning().noquote() << "[MIT3HP GUI] Auto PASSIVE:"
                         << _lastAutoPassiveReason;
    requestPassive();
  }
  _standButton->setEnabled(ctrlMs && statusMs && now - ctrlMs < 500 &&
      now - statusMs < 500 && actualMode == kPassive);
  _rearPoseButton->setEnabled(rearStage == 2);
  _airTrotButton->setEnabled(rearStage == 4);
  _balanceButton->setEnabled(false);
  _walkButton->setEnabled(false);
  _gaitSelector->setEnabled(false);

  const float signs[3][4] = {
      {-1.f, -1.f, 1.f, 1.f},
      {-1.f, 1.f, -1.f, 1.f},
      {-0.6666667f, 0.6666667f, -0.6666667f, 0.6666667f}};
  const float ratios[3] = {6.f, 6.f, 9.f};
  const bool legDataFresh = legDataMs && now - legDataMs < 300;
  const bool legCommandFresh = legCommandMs && now - legCommandMs < 300;
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      const int idx = leg * 3 + joint;
      const float values[] = {
          legData.q[idx], legData.qd[idx], legCommand.q_des[idx],
          legCommand.qd_des[idx], legData.tau_est[idx],
          legCommand.kp_joint[idx], legCommand.kd_joint[idx],
          signs[joint][leg], 0.f, ratios[joint],
          static_cast<float>(spi.flags[leg])};
      for (int column = 0; column < 11; column++) {
        const int tableColumn = column == 0 ? 1 : column + 2;
        auto* item = _motorTable->item(idx, tableColumn);
      if (!item) {
        item = new QTableWidgetItem;
          _motorTable->setItem(idx, tableColumn, item);
      }
        item->setText(QString::number(values[column], 'f', 3));
        const bool boardFresh = boardAgeMs[leg / 2] >= 0 &&
                                boardAgeMs[leg / 2] <= 100;
        QColor cellColor = (legDataFresh && legCommandFresh && boardFresh)
                               ? QColor("#dcfce7") : QColor("#fecaca");
        // Gioi han torque dang dung trong rt_spi.cpp: Abad/Hip 17 Nm,
        // Knee 26 Nm. Day la canh bao tau uoc tinh, khong phai torque do duoc.
        const float torqueLimit[3] = {17.f, 17.f, 26.f};
        if (column == 4 && std::fabs(values[column]) > torqueLimit[joint])
          cellColor = QColor("#fca5a5");
        item->setBackground(cellColor);
      }
      auto* angleItem = _motorTable->item(idx, 2);
      if (!angleItem) {
        angleItem = new QTableWidgetItem;
        _motorTable->setItem(idx, 2, angleItem);
      }
      const bool angleFresh = legDataFresh && boardAgeMs[leg / 2] >= 0 &&
          boardAgeMs[leg / 2] <= 100;
      angleItem->setText(QString("%1 deg / %2 rad%3")
          .arg(double(legData.q[idx]) * 57.29577951308232, 0, 'f', 2)
          .arg(legData.q[idx], 0, 'f', 3)
          .arg(angleFresh ? "" : " *"));
      angleItem->setToolTip(angleFresh ? "Joint angle from recent SPI feedback."
          : "STALE: last received angle, not a current physical measurement.");
      angleItem->setBackground(QColor(angleFresh ? "#dcfce7" : "#fecaca"));
    }
  }
  _imuValues->setText(QString(
      "Quaternion: [%1, %2, %3, %4]\nGyro: [%5, %6, %7] rad/s\nAccel: [%8, %9, %10] m/s^2")
      .arg(imu.q[0], 0, 'f', 3).arg(imu.q[1], 0, 'f', 3)
      .arg(imu.q[2], 0, 'f', 3).arg(imu.q[3], 0, 'f', 3)
      .arg(imu.w[0], 0, 'f', 3).arg(imu.w[1], 0, 'f', 3)
      .arg(imu.w[2], 0, 'f', 3).arg(imu.a[0], 0, 'f', 3)
      .arg(imu.a[1], 0, 'f', 3).arg(imu.a[2], 0, 'f', 3));
}

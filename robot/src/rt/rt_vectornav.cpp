/*!
 * @file rt_vectornav.cpp
 * @brief VectorNav IMU communication
 */

#ifdef linux

#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>
#include <stdexcept>
#include <thread>

#include <lcm/lcm-cpp.hpp>

#include "SimUtilities/IMUTypes.h"
#include "Utilities/utilities.h"
#include "rt/rt_vectornav.h"
#include "vectornav_lcmt.hpp"

#define K_MINI_CHEETAH_VECTOR_NAV_SERIAL "/dev/ttyUSB0"

//#define PRINT_VECTORNAV_DEBUG

int processErrorReceived(const std::string& errorMessage, VnError errorCode);
void vectornav_handler(void* userData, VnUartPacket* packet,
                       size_t running_index);
/*!
 * VectorNav Driver data
 */
typedef struct {
  VnSensor vs;
  BinaryOutputRegister bor;
} vn_sensor;

vn_sensor vn;

static lcm::LCM* vectornav_lcm;
vectornav_lcmt vectornav_lcm_data;
static VectorNavData* g_vn_data = nullptr;
static std::atomic<uint64_t> g_vn_last_valid_packet_ns{0};

static uint64_t monotonic_time_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

uint64_t vectornav_last_packet_age_us() {
  const uint64_t last = g_vn_last_valid_packet_ns.load(std::memory_order_acquire);
  if (last == 0) return UINT64_MAX;
  return (monotonic_time_ns() - last) / 1000;
}

bool vectornav_data_is_valid(uint64_t max_age_us) {
  return vectornav_last_packet_age_us() <= max_age_us;
}

/*!
 * Initialize Vectornav communication and set up sensor
 */
bool init_vectornav(VectorNavData* vn_data) {
  g_vn_data = vn_data;
  printf("[Simulation] Setup LCM...\n");
  vectornav_lcm = new lcm::LCM(getLcmUrl(255));
  if (!vectornav_lcm->good()) {
    printf("[ERROR] Failed to set up LCM\n");
    throw std::runtime_error("lcm bad");
  }

  VnError error;
  VpeBasicControlRegister vpeReg;
  ImuFilteringConfigurationRegister filtReg;
  const char* configuredPort = std::getenv("CHEETAH_VECTORNAV_PORT");
  const char* SENSOR_PORT =
      configuredPort && configuredPort[0] ? configuredPort
                                          : K_MINI_CHEETAH_VECTOR_NAV_SERIAL;
  const uint32_t SENSOR_STREAM_BAUDRATE = 921600;
  const uint32_t SENSOR_BAUDRATES[] = {921600, 115200};
  uint32_t connectedBaudrate = 0;
  char modelNumber[30];
  char strConversions[50];
  uint32_t newHz, oldHz;
  // uint32_t hz_desired = 200;

  printf("[rt_vectornav] init_vectornav() on %s\n", SENSOR_PORT);
  g_vn_last_valid_packet_ns.store(0, std::memory_order_release);

  // initialize vectornav library
  VnSensor_initialize(&(vn.vs));

  // The sensor may already be at the streaming baud rate after a previous
  // program run. Probe both supported rates so restarting the robot process
  // does not require power-cycling the VN-100.
  for (uint32_t baudrate : SENSOR_BAUDRATES) {
    printf("[rt_vectornav] Trying serial baud rate %u...\n", baudrate);
    error = VnSensor_connect(&(vn.vs), SENSOR_PORT, baudrate);
    if (error == E_NONE) {
      error = VnSensor_readModelNumber(&(vn.vs), modelNumber,
                                       sizeof(modelNumber));
      if (error == E_NONE) {
        connectedBaudrate = baudrate;
        break;
      }
      VnSensor_disconnect(&(vn.vs));
    }
  }
  if (connectedBaudrate == 0) {
    printf("[rt_vectornav] Could not communicate at 921600 or 115200 baud.\n");
    processErrorReceived("Error reading model number.", error);
    return false;
  }
  printf("Model Number: %s\n", modelNumber);

  // The quaternion + angular-rate + acceleration binary packet at 200 Hz
  // does not fit at 115200 baud. Change both the VN-100 and the host serial
  // port to 921600 before enabling Binary Output 1.
  if (connectedBaudrate != SENSOR_STREAM_BAUDRATE) {
    if ((error = VnSensor_changeBaudrate(&(vn.vs), SENSOR_STREAM_BAUDRATE)) !=
        E_NONE) {
      printf("[rt_vectornav] VnSensor_changeBaudrate failed.\n");
      processErrorReceived("Error changing sensor baud rate.", error);
      return false;
    }
    printf("[rt_vectornav] Serial baud rate changed from %u to %u.\n",
           connectedBaudrate, SENSOR_STREAM_BAUDRATE);
  } else {
    printf("[rt_vectornav] Serial baud rate is already %u.\n",
           SENSOR_STREAM_BAUDRATE);
  }

  // Disable the ASCII asynchronous stream; binary output is configured below.
  if ((error = VnSensor_readAsyncDataOutputFrequency(&(vn.vs), &oldHz)) !=
      E_NONE) {
    printf("[rt_vectornav] VnSensor_readAsyncDataOutputFrequency failed.\n");
    processErrorReceived("Error reading async data output frequency.", error);
    return false;
  }

  // non-zero frequency causes the IMU to output ascii packets at the set
  // frequency, as well as binary
  if (oldHz == 0) {
    printf("[rt_vectornav] Optional ASCII output is already disabled.\n");
  } else if ((error = VnSensor_writeAsyncDataOutputFrequency(&(vn.vs), 0,
                                                              true)) == E_NONE) {
    if ((error = VnSensor_readAsyncDataOutputFrequency(&(vn.vs), &newHz)) !=
        E_NONE) {
      printf("[rt_vectornav] VnSensor_readAsyncDataOutputFrequency failed.\n");
      processErrorReceived("Error reading async data output frequency.", error);
      return false;
    }
    printf("[rt_vectornav] Changed frequency from %d to %d Hz.\n", oldHz,
           newHz);
  } else {
    // Some VN-100S firmware revisions do not support changing the legacy
    // ASCII async frequency on this port. Binary Output 1 below is independent
    // of this optional setting, so continue and validate the actual stream.
    printf("[rt_vectornav] Warning: could not disable optional ASCII output; "
           "continuing with binary output.\n");
  }

  // change to relative heading mode to avoid compass weirdness
  if ((error = VnSensor_readVpeBasicControl(&(vn.vs), &vpeReg)) != E_NONE) {
    printf("[rt_vectornav] VnSensor_ReadVpeBasicControl failed.\n");
    processErrorReceived("Error reading VPE basic control.", error);
    return false;
  }
  strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
  printf("[rt_vectornav] Sensor was in mode: %s\n", strConversions);
  if (vpeReg.headingMode != VNHEADINGMODE_RELATIVE) {
    vpeReg.headingMode = VNHEADINGMODE_RELATIVE;
    if ((error = VnSensor_writeVpeBasicControl(&(vn.vs), vpeReg, true)) !=
        E_NONE) {
      printf("[rt_vectornav] VnSensor_writeVpeBasicControl failed.\n");
      processErrorReceived("Error writing VPE basic control.", error);
      return false;
    }
    if ((error = VnSensor_readVpeBasicControl(&(vn.vs), &vpeReg)) != E_NONE) {
      processErrorReceived("Error reading VPE basic control.", error);
      printf("[rt_vectornav] VnSensor_ReadVpeBasicControl failed.\n");
      return false;
    }
  }
  strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
  printf("[rt_vectornav] Sensor now id mode: %s\n", strConversions);

  if ((error = VnSensor_readImuFilteringConfiguration(&(vn.vs), &filtReg)) !=
      E_NONE) {
    printf("[rt_vectornav] VnSensor_readGyroCompensation failed.\n");
  }
  printf("[rt_vectornav] AccelWindow: %d\n", filtReg.accelWindowSize);
  //        filtReg.accelWindowSize = 4;    // We're sampling at 200 hz, but the
  //        imu samples at 800 hz. filtReg.accelFilterMode = 3; if((error =
  //        VnSensor_writeImuFilteringConfiguration(&(vn.vs), filtReg, true)) !=
  //        E_NONE)
  //        {
  //            printf("[rt_vectornav] VnSensor_writeGyroCompensation
  //            failed.\n");
  //        }

  // setup binary output message type
  BinaryOutputRegister_initialize(
      &(vn.bor), ASYNCMODE_PORT1,
      4,  // divisor:  output frequency = 800/divisor
      (CommonGroup)(COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE |
                    COMMONGROUP_ACCEL),
      TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
      INSGROUP_NONE, GPSGROUP_NONE);

  BinaryOutputRegister currentBor;
  bool binaryOutputMatches = false;
  if (VnSensor_readBinaryOutput1(&(vn.vs), &currentBor) == E_NONE) {
    binaryOutputMatches =
        currentBor.asyncMode == vn.bor.asyncMode &&
        currentBor.rateDivisor == vn.bor.rateDivisor &&
        currentBor.commonField == vn.bor.commonField &&
        currentBor.timeField == vn.bor.timeField &&
        currentBor.imuField == vn.bor.imuField &&
        currentBor.gpsField == vn.bor.gpsField &&
        currentBor.attitudeField == vn.bor.attitudeField &&
        currentBor.insField == vn.bor.insField &&
        currentBor.gps2Field == vn.bor.gps2Field;
  }
  if (binaryOutputMatches) {
    printf("[rt_vectornav] Binary Output 1 is already configured.\n");
  } else if ((error = VnSensor_writeBinaryOutput1(&(vn.vs), &(vn.bor), true)) !=
             E_NONE) {
      printf("[rt_vectornav] VnSensor_writeBinaryOutput1 failed.\n");
      processErrorReceived("Error writing binary output 1.", error);
      return false;
  }

  // setup handler
  VnSensor_registerAsyncPacketReceivedHandler(&(vn.vs), vectornav_handler,
                                              NULL);

  // Do not report successful hardware initialization until at least one
  // complete, numerically valid packet has arrived.
  constexpr uint64_t kStartupTimeoutUs = 2000000;
  const auto waitStart = monotonic_time_ns();
  while (!vectornav_data_is_valid(50000)) {
    if ((monotonic_time_ns() - waitStart) / 1000 > kStartupTimeoutUs) {
      printf("[rt_vectornav] Timed out waiting for a valid IMU packet.\n");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  printf("[rt_vectornav] IMU is set up and streaming valid data!\n");
  return true;
}

int got_first_vectornav_message = 0;

/*!
 * Got new packet handler for vectornav
 */
void vectornav_handler(void* userData, VnUartPacket* packet,
                       size_t running_index) {
  (void)userData;
  (void)running_index;
  vec4f quat;
  vec3f omega;
  vec3f a;

  if (VnUartPacket_type(packet) != PACKETTYPE_BINARY) {
    printf("[vectornav_handler] got a packet that wasn't binary.\n");
    return;
  }

  if (!VnUartPacket_isCompatible(
          packet,
          (CommonGroup)(COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE |
                        COMMONGROUP_ACCEL),
          TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
          INSGROUP_NONE, GPSGROUP_NONE)) {
    printf("[vectornav_handler] got a packet with the wrong type of data.\n");
    return;
  }

  quat = VnUartPacket_extractVec4f(packet);
  omega = VnUartPacket_extractVec3f(packet);
  a = VnUartPacket_extractVec3f(packet);

  float quatNormSquared = 0.f;
  float accelNormSquared = 0.f;
  for (int i = 0; i < 4; i++) {
    if (!std::isfinite(quat.c[i])) return;
    quatNormSquared += quat.c[i] * quat.c[i];
  }
  for (int i = 0; i < 3; i++) {
    if (!std::isfinite(omega.c[i]) || !std::isfinite(a.c[i])) return;
    accelNormSquared += a.c[i] * a.c[i];
  }
  // Reject corrupt packets without rejecting normal robot dynamics.
  if (quatNormSquared < 0.81f || quatNormSquared > 1.21f ||
      accelNormSquared > 10000.f) {
    return;
  }

  for (int i = 0; i < 4; i++) {
    vectornav_lcm_data.q[i] = quat.c[i];
    g_vn_data->quat[i] = quat.c[i];
  }

  for (int i = 0; i < 3; i++) {
    vectornav_lcm_data.w[i] = omega.c[i];
    vectornav_lcm_data.a[i] = a.c[i];
    g_vn_data->gyro[i] = omega.c[i];
    g_vn_data->accelerometer[i] = a.c[i];
  }

  vectornav_lcm->publish("hw_vectornav", &vectornav_lcm_data);
  g_vn_last_valid_packet_ns.store(monotonic_time_ns(),
                                  std::memory_order_release);

#ifdef PRINT_VECTORNAV_DEBUG
  char strConversions[50];
  str_vec4f(strConversions, quat);
  printf("[QUAT] %s\n", strConversions);

  str_vec3f(strConversions, omega);
  printf("[OMEGA] %s\n", strConversions);

  str_vec3f(strConversions, a);
  printf("[ACC] %s\n", strConversions);
#endif
}

/*!
 * Error callback for vectornav
 */
int processErrorReceived(const std::string& errorMessage, VnError errorCode) {
  char errorCodeStr[100];
  strFromVnError(errorCodeStr, errorCode);
  printf("%s\nVECTORNAV ERROR: %s\n", errorMessage.c_str(), errorCodeStr);
  return -1;
}
#endif

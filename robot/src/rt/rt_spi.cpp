/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */
#ifdef linux

#include <byteswap.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <chrono>

#include <linux/spi/spidev.h>
#include "rt/rt_spi.h"
#include <lcm/lcm-cpp.hpp>

unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
// Toc do Jetson SPI de thu nghiem voi hai board STM32.
unsigned int spi_speed = 1000000;  // 1 MHz
uint8_t lsb = 0x01;

int spi_1_fd = -1;
int spi_2_fd = -1;

int spi_open();

static spine_cmd_t g_spine_cmd;
static spine_data_t g_spine_data;

spi_command_t spi_command_drv;
spi_data_t spi_data_drv;
spi_torque_t spi_torque;

pthread_mutex_t spi_mutex;
static spi_board_health_t spi_board_health[2]{};
// Du lieu goc/toc do chi duoc cap nhat khi goi 132 byte qua kiem tra XOR va
// mien gia tri. Goi loi giu nguyen mau hop le gan nhat cua board do.

static uint64_t monotonic_time_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Khong in mot dong cho moi goi loi trong SPI task: khi loi nhieu, terminal
// co the lam tre ca hai board. Bo dem loi tren GUI van ghi day du moi goi.
static bool should_log_spi_error(int board) {
  static uint64_t last_log_us[2] = {};
  const uint64_t now_us = monotonic_time_us();
  if (last_log_us[board] && now_us - last_log_us[board] < 1000000)
    return false;
  last_log_us[board] = now_us;
  return true;
}

const float max_torque[3] = {17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

// only used for actual robot
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
// Do lon 1/1.5 quy doi goc/toc do tu dau ra motor sang khop knee qua bo
// truyen dai 1.5:1. Dau +/- tam giu theo mapping cu va se hieu chuan thuc te.
const float knee_side_sign[4] = {-0.6666667f, 0.6666667f, -0.6666667f,
                                 0.6666667f};

// MIT_3HP dat moc q = 0 khi tung khop o tu the chan duoi thang. Sau khi set
// zero truc tiep cho du 12 motor, encoder raw tai tu the nay phai gan bang 0,
// vi vay Jetson khong cong/tru offset co khi cua Mini Cheetah nua.
const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float hip_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float knee_offset[4] = {0.f, 0.f, 0.f, 0.f};

/*!
 * Compute SPI message checksum
 * @param data : input
 * @param len : length (in 32-bit words)
 * @return
 */
uint32_t xor_checksum(uint32_t *data, size_t len) {
  uint32_t t = 0;
  for (size_t i = 0; i < len; i++) t = t ^ data[i];
  return t;
}

/*!
 * Emulate the spi board to estimate the torque.
 */
void fake_spine_control(spi_command_t *cmd, spi_data_t *data,
                        spi_torque_t *torque_out, int board_num) {
  torque_out->tau_abad[board_num] =
      cmd->kp_abad[board_num] *
          (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
      cmd->kd_abad[board_num] *
          (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
      cmd->tau_abad_ff[board_num];

  torque_out->tau_hip[board_num] =
      cmd->kp_hip[board_num] *
          (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
      cmd->kd_hip[board_num] *
          (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
      cmd->tau_hip_ff[board_num];

  torque_out->tau_knee[board_num] =
      cmd->kp_knee[board_num] *
          (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
      cmd->kd_knee[board_num] *
          (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
      cmd->tau_knee_ff[board_num];

  const float *torque_limits = disabled_torque;

  if (cmd->flags[board_num] & 0b1) {
    if (cmd->flags[board_num] & 0b10)
      torque_limits = wimp_torque;
    else
      torque_limits = max_torque;
  }

  if (torque_out->tau_abad[board_num] > torque_limits[0])
    torque_out->tau_abad[board_num] = torque_limits[0];
  if (torque_out->tau_abad[board_num] < -torque_limits[0])
    torque_out->tau_abad[board_num] = -torque_limits[0];

  if (torque_out->tau_hip[board_num] > torque_limits[1])
    torque_out->tau_hip[board_num] = torque_limits[1];
  if (torque_out->tau_hip[board_num] < -torque_limits[1])
    torque_out->tau_hip[board_num] = -torque_limits[1];

  if (torque_out->tau_knee[board_num] > torque_limits[2])
    torque_out->tau_knee[board_num] = torque_limits[2];
  if (torque_out->tau_knee[board_num] < -torque_limits[2])
    torque_out->tau_knee[board_num] = -torque_limits[2];
}

/*!
 * Initialize SPI
 */
void init_spi() {
  // check sizes:
  size_t command_size = sizeof(spi_command_t);
  size_t data_size = sizeof(spi_data_t);

  memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  memset(&spi_data_drv, 0, sizeof(spi_data_drv));
  memset(spi_board_health, 0, sizeof(spi_board_health));

  if (pthread_mutex_init(&spi_mutex, NULL) != 0)
    printf("[ERROR: RT SPI] Failed to create spi data mutex\n");

  if (command_size != K_EXPECTED_COMMAND_SIZE) {
    printf("[RT SPI] Error command size is %ld, expected %d\n", command_size,
           K_EXPECTED_COMMAND_SIZE);
  } else
    printf("[RT SPI] command size good\n");

  if (data_size != K_EXPECTED_DATA_SIZE) {
    printf("[RT SPI] Error data size is %ld, expected %d\n", data_size,
           K_EXPECTED_DATA_SIZE);
  } else
    printf("[RT SPI] data size good\n");

  printf("[RT SPI] Open\n");
  spi_open();
}

/*!
 * Open SPI device
 */
int spi_open() {
  int rv = 0;
  // MIT_3HP Jetson wiring (physical header pins):
  // SCK: 13, MISO: 22, MOSI: 37, GND: 39.
  // CS0: 18 -> A2; CS1: 16. Pinmux must be configured on the Jetson.
  spi_1_fd = open("/dev/spidev1.0", O_RDWR);
  if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 1.0");
  spi_2_fd = open("/dev/spidev1.1", O_RDWR);
  if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 1.1");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

  // Read back into a separate variable so a rejected device setting cannot
  // silently change the requested 16-bit width for both transfers.
  for (int board = 0; board < 2; ++board) {
    uint8_t actual_bits_per_word = 0;
    rv = ioctl(board == 0 ? spi_1_fd : spi_2_fd,
               SPI_IOC_RD_BITS_PER_WORD, &actual_bits_per_word);
    if (rv < 0) {
      perror("[ERROR] ioctl spi_ioc_rd_bits_per_word");
    } else if (actual_bits_per_word != spi_bits_per_word) {
      printf("[ERROR: RT SPI] /dev/spidev1.%d configured for %u bits/word, "
             "expected %u\n", board, actual_bits_per_word, spi_bits_per_word);
    }
  }

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");
  return rv;
}

int spi_driver_iterations = 0;

// Day thuc te: kenh 0 moi STM32 la chan trai, kenh 1 la chan phai.
// Thu tu model cua MIT van la FR, FL, HR, HL. Dao kenh o ca hai chieu SPI.
static int model_leg_for_spine_channel(int first_leg, int channel) {
  return first_leg + (1 - channel);
}

/*!
 * convert spi command to spine_cmd_t
 */
void spi_to_spine(spi_command_t *cmd, spine_cmd_t *spine_cmd, int leg_0) {
  for (int i = 0; i < 2; i++) {
    const int leg = model_leg_for_spine_channel(leg_0, i);
    spine_cmd->q_des_abad[i] =
        (cmd->q_des_abad[leg] * abad_side_sign[leg]) +
        abad_offset[leg];
    spine_cmd->q_des_hip[i] =
        (cmd->q_des_hip[leg] * hip_side_sign[leg]) +
        hip_offset[leg];
    spine_cmd->q_des_knee[i] =
        (cmd->q_des_knee[leg] / knee_side_sign[leg]) +
        knee_offset[leg];

    spine_cmd->qd_des_abad[i] =
        cmd->qd_des_abad[leg] * abad_side_sign[leg];
    spine_cmd->qd_des_hip[i] =
        cmd->qd_des_hip[leg] * hip_side_sign[leg];
    spine_cmd->qd_des_knee[i] =
        cmd->qd_des_knee[leg] / knee_side_sign[leg];

    spine_cmd->kp_abad[i] = cmd->kp_abad[leg];
    spine_cmd->kp_hip[i] = cmd->kp_hip[leg];
    spine_cmd->kp_knee[i] = cmd->kp_knee[leg];

    spine_cmd->kd_abad[i] = cmd->kd_abad[leg];
    spine_cmd->kd_hip[i] = cmd->kd_hip[leg];
    spine_cmd->kd_knee[i] = cmd->kd_knee[leg];

    spine_cmd->tau_abad_ff[i] =
        cmd->tau_abad_ff[leg] * abad_side_sign[leg];
    spine_cmd->tau_hip_ff[i] =
        cmd->tau_hip_ff[leg] * hip_side_sign[leg];
    spine_cmd->tau_knee_ff[i] =
        cmd->tau_knee_ff[leg] * knee_side_sign[leg];

    spine_cmd->flags[i] = cmd->flags[leg];
  }
  spine_cmd->checksum = xor_checksum((uint32_t *)spine_cmd, 32);
}

/*!
 * convert spine_data_t to spi data
 */
int spine_to_spi(spi_data_t *data, spine_data_t *spine_data, int leg_0) {
  // XOR cua goi toan 0 van hop le; khong duoc coi la board co nguon khi
  // chuong trinh da cho phep phep thu motor co luc.
  const uint32_t *response_words =
      reinterpret_cast<const uint32_t *>(spine_data);
  bool all_zero = true;
  for (size_t i = 0; i < sizeof(spine_data_t) / sizeof(uint32_t); ++i) {
    if (response_words[i] != 0) {
      all_zero = false;
      break;
    }
  }
  if (all_zero) {
    static uint32_t empty_count[2] = {};
    const int board = leg_0 / 2;
    if ((empty_count[board]++ % 500) == 0)
      printf("[ERROR: RT SPI] /dev/spidev1.%d all-zero response; "
             "STM32 presence not verified\n", board);
    return -1;
  }

  const uint32_t calculated = xor_checksum((uint32_t *)spine_data, 14);
  const uint32_t received = (uint32_t)spine_data->checksum;
  const bool checksum_ok = calculated == received;
  if (!checksum_ok) {
    if (should_log_spi_error(leg_0 / 2))
      printf("[ERROR: RT SPI] /dev/spidev1.%d response checksum mismatch: "
             "received=0x%08" PRIx32 " calculated=0x%08" PRIx32
             "; response discarded\n", leg_0 / 2, received, calculated);
    return -2;
  }

  // Du lieu CAN cua motor duoc giai ma trong mien p +/-12.5 rad,
  // v +/-65 rad/s. Chan gia tri phi huu han/ngoai mien du XOR trung.
  const float *feedback = reinterpret_cast<const float *>(spine_data);
  for (int i = 0; i < 12; ++i) {
    const float limit = i < 6 ? 12.5f : 65.f;
    if (!isfinite(feedback[i]) || fabsf(feedback[i]) > limit) {
      if (should_log_spi_error(leg_0 / 2))
        printf("[ERROR: RT SPI] /dev/spidev1.%d implausible motor feedback; "
               "response discarded\n", leg_0 / 2);
      return -3;
    }
  }

  for (int i = 0; i < 2; i++) {
    const int leg = model_leg_for_spine_channel(leg_0, i);
    data->q_abad[leg] = (spine_data->q_abad[i] - abad_offset[leg]) *
                         abad_side_sign[leg];
    data->q_hip[leg] = (spine_data->q_hip[i] - hip_offset[leg]) *
                        hip_side_sign[leg];
    data->q_knee[leg] = (spine_data->q_knee[i] - knee_offset[leg]) *
                         knee_side_sign[leg];

    data->qd_abad[leg] = spine_data->qd_abad[i] * abad_side_sign[leg];
    data->qd_hip[leg] = spine_data->qd_hip[i] * hip_side_sign[leg];
    data->qd_knee[leg] = spine_data->qd_knee[i] * knee_side_sign[leg];

    data->flags[leg] = spine_data->flags[i];
  }
  return 1;
}

/*!
 * send receive data and command from spine
 */
void spi_send_receive(spi_command_t *command, spi_data_t *data) {
  // update driver status flag
  spi_driver_iterations++;
  data->spi_driver_status = spi_driver_iterations << 16;

  // transmit and receive buffers
  uint16_t tx_buf[K_WORDS_PER_MESSAGE];
  uint16_t rx_buf[K_WORDS_PER_MESSAGE];

  for (int spi_board = 0; spi_board < 2; spi_board++) {
    // copy command into spine type:
    spi_to_spine(command, &g_spine_cmd, spi_board * 2);

    // pointers to command/data spine array
    uint16_t *cmd_d = (uint16_t *)&g_spine_cmd;
    uint16_t *data_d = (uint16_t *)&g_spine_data;

    // zero rx buffer
    memset(rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));

    // Linux sends each 8-bit word in memory order. Swap the bytes in each
    // native-endian uint16_t so the STM32 receives the same 16-bit values.
    for (int i = 0; i < K_WORDS_PER_MESSAGE; ++i)
      tx_buf[i] = (cmd_d[i] >> 8) | (cmd_d[i] << 8);

    // each word is two bytes long
    constexpr size_t transfer_bytes = K_WORDS_PER_MESSAGE * sizeof(uint16_t);
    static_assert(transfer_bytes == 132, "SPI transaction must be 132 bytes");

    // spi message struct
    struct spi_ioc_transfer spi_message[1];

    // zero message struct.
    memset(spi_message, 0, 1 * sizeof(struct spi_ioc_transfer));

    // set up message struct
    for (int i = 0; i < 1; i++) {
      spi_message[i].bits_per_word = spi_bits_per_word;
      spi_message[i].cs_change = 0;  // Release CS after this message.
      spi_message[i].delay_usecs = 0;
      spi_message[i].len = transfer_bytes;
      spi_message[i].rx_buf = (uint64_t)rx_buf;
      spi_message[i].tx_buf = (uint64_t)tx_buf;
    }

    // do spi communication
    int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd, SPI_IOC_MESSAGE(1),
                   &spi_message);
    if (rv < 0) {
      spi_board_health[spi_board].ioctl_errors++;
      const int saved_errno = errno;
      if (should_log_spi_error(spi_board))
        printf("[ERROR: RT SPI] /dev/spidev1.%d ioctl failed: %s (errno=%d); "
               "expected %zu bytes; response discarded\n",
               spi_board, strerror(saved_errno), saved_errno, transfer_bytes);
      continue;
    }
    if (rv != static_cast<int>(transfer_bytes)) {
      spi_board_health[spi_board].incomplete_transfers++;
      if (should_log_spi_error(spi_board))
        printf("[ERROR: RT SPI] /dev/spidev1.%d incomplete transfer: "
               "got %d bytes, expected %zu; response discarded\n",
               spi_board, rv, transfer_bytes);
      continue;
    }
    // ioctl hoan tat 132 byte: ghi lai co enable THUC SU da dua vao
    // transaction Jetson. Day chua chung minh STM32 da chap nhan checksum.
    spi_board_health[spi_board].transmitted_frames++;
    spi_board_health[spi_board].last_tx_flags =
        (static_cast<uint32_t>(g_spine_cmd.flags[0]) & 0xffu) |
        ((static_cast<uint32_t>(g_spine_cmd.flags[1]) & 0xffu) << 8);

    // Convert the 60 received bytes back into native-endian 16-bit values.
    for (int i = 0; i < 30; ++i)
      data_d[i] = (rx_buf[i] >> 8) | (rx_buf[i] << 8);

    // copy back to data
    const int response_status = spine_to_spi(data, &g_spine_data, spi_board * 2);
    if (response_status > 0) {
      spi_board_health[spi_board].successful_transfers++;
      spi_board_health[spi_board].last_success_us = monotonic_time_us();
      // Chan doan board 1.1: so sanh goc trong response STM32 voi goc da
      // quy doi cho HR/HL. Chi in 1 lan/giay de khong nghen SPI task.
      if (spi_board == 1) {
        static uint64_t last_rear_log_us = 0;
        const uint64_t now_us = spi_board_health[spi_board].last_success_us;
        if (now_us - last_rear_log_us >= 1000000) {
          last_rear_log_us = now_us;
          printf("[SPI 1.1 RX VALID] STM32 ch0 a/h/k=%+.3f/%+.3f/%+.3f "
                 "ch1=%+.3f/%+.3f/%+.3f | Jetson HL=%+.3f/%+.3f/%+.3f "
                 "HR=%+.3f/%+.3f/%+.3f\n",
                 g_spine_data.q_abad[0], g_spine_data.q_hip[0],
                 g_spine_data.q_knee[0], g_spine_data.q_abad[1],
                 g_spine_data.q_hip[1], g_spine_data.q_knee[1],
                 data->q_abad[3], data->q_hip[3], data->q_knee[3],
                 data->q_abad[2], data->q_hip[2], data->q_knee[2]);
        }
      }
    } else if (response_status < 0) {
      spi_board_health[spi_board].checksum_errors++;
      if (response_status == -1)
        spi_board_health[spi_board].all_zero_responses++;
      else if (response_status == -2)
        spi_board_health[spi_board].checksum_mismatches++;
      else if (response_status == -3)
        spi_board_health[spi_board].implausible_feedback++;
    }
  }
}

/*!
 * Run SPI
 */
void spi_driver_run() {
  // do spi board calculations
  for (int i = 0; i < 4; i++) {
    fake_spine_control(&spi_command_drv, &spi_data_drv, &spi_torque, i);
  }

  // in here, the driver is good
  pthread_mutex_lock(&spi_mutex);
  spi_send_receive(&spi_command_drv, &spi_data_drv);
  pthread_mutex_unlock(&spi_mutex);
}

/*!
 * Get the spi command
 */
spi_command_t *get_spi_command() {
  return &spi_command_drv;
}

/*!
 * Get the spi data
 */
spi_data_t *get_spi_data() { return &spi_data_drv; }

void get_spi_board_health(int board_index, spi_board_health_t* health) {
  if (!health || board_index < 0 || board_index > 1) return;
  pthread_mutex_lock(&spi_mutex);
  *health = spi_board_health[board_index];
  pthread_mutex_unlock(&spi_mutex);
}

#endif

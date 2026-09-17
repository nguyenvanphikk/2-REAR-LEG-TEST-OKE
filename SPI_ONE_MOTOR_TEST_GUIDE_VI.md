# Hướng dẫn thử Jetson → SPI → STM32 → CAN → một motor

## 1. Cấu hình hiện tại

Code hiện được giới hạn để thử duy nhất:

- Jetson: `/dev/spidev2.0` (CS0, chân 18 theo phần cứng đang sử dụng).
- STM32 SPI CS: `PA4`.
- STM32 CAN: CAN2 (`PB8/PB9`).
- Motor: CAN ID `1` (khớp abad).
- SPI: Mode 0, tốc độ Jetson 1 MHz.
- Gói SPI: 132 byte mỗi transaction.
- Watchdog lệnh: 250 ms; mất lệnh thì motor thoát Motor Mode.

Luồng dữ liệu:

```text
Jetson --SPI--> STM32 --CAN--> CAN transceiver --> Motor ID 1
Jetson <--SPI-- STM32 <--CAN-- CAN transceiver <-- Motor ID 1
```

UART chỉ dùng để xem log STM32, không dùng để điều khiển motor.

## 2. Nối dây

### 2.1. SPI Jetson ↔ STM32

```text
Jetson MOSI       → STM32 PA7
Jetson MISO       ← STM32 PA6
Jetson SCK        → STM32 PA5
Jetson CS0 pin 18 → STM32 PA4
Jetson GND        ↔ STM32 GND
```

Tất cả tín hiệu logic phải là 3.3 V và hai bo phải chung GND.

### 2.2. STM32 ↔ CAN transceiver

Code thử hiện chọn CAN2:

```text
STM32 PB9 (CAN TX) → CAN transceiver TXD
STM32 PB8 (CAN RX) ← CAN transceiver RXD
STM32 GND           ↔ CAN transceiver GND
```

Phía bus:

```text
Transceiver CANH ↔ Motor CANH
Transceiver CANL ↔ Motor CANL
```

Kiểm tra điện trở kết thúc bus CAN phù hợp. Không đảo CANH/CANL.

### 2.3. UART để xem log

Nếu dùng USB-UART 3.3 V:

```text
STM32 PA2 (TX) → USB-UART RX
STM32 PA3 (RX) ← USB-UART TX   (không bắt buộc nếu chỉ xem log)
STM32 GND       ↔ USB-UART GND
```

Không nối VCC của USB-UART nếu STM32 đã được cấp nguồn riêng.

Nếu dùng Nucleo-F446RE, có thể dùng luôn ST-Link Virtual COM qua cáp USB nếu bo đang nối PA2/PA3 tới ST-Link.

## 3. Build và nạp STM32

1. Mở file `SPIne.uvprojx` bằng Keil µVision.
2. Kiểm tra trong `main.cpp`:

```cpp
#define SINGLE_MOTOR_CAN_TEST 0
#define SPI_SINGLE_MOTOR_BRINGUP 1
#define SPI_BRINGUP_CAN_BUS 2
#define SPI_BRINGUP_MOTOR_ID 1
```

3. Chọn **Build** hoặc nhấn `F7`.
4. Phải đạt `0 Error`.
5. Nạp chương trình vào STM32 bằng **Download**.
6. Reset STM32.

Không dùng file `SPIne.bin` cũ nếu file đó được tạo trước khi thêm chế độ bring-up.

## 4. Mở UART STM32 bằng PuTTY trên Windows

### 4.1. Tìm cổng COM

1. Cắm USB ST-Link/Nucleo hoặc USB-UART vào máy Windows.
2. Mở **Device Manager**.
3. Mở mục **Ports (COM & LPT)**.
4. Ghi lại cổng, ví dụ `COM5`.

Có thể thấy tên như:

```text
STMicroelectronics STLink Virtual COM Port (COM5)
USB Serial Port (COM6)
```

### 4.2. Cấu hình PuTTY

1. Mở PuTTY.
2. Chọn **Session**.
3. Chọn **Connection type: Serial**.
4. Nhập:

```text
Serial line: COM5       (thay bằng cổng thực tế)
Speed:       921600
```

5. Vào **Connection → Serial**, đặt:

```text
Speed (baud): 921600
Data bits:    8
Stop bits:    1
Parity:       None
Flow control: None
```

6. Nhấn **Open**.
7. Nhấn Reset STM32.

PuTTY phải hiển thị chữ STM32 gửi ra, ví dụ:

```text
SPI Init ...
SPI Init done
[SPI] frames(+0), bad(+0), total=0/0, last_len=0, estop=1, enabled=0
```

`Plain text` chỉ là kết quả STM32 in ra; không phải lệnh cần gõ.

Nếu PuTTY không hiện gì:

- Kiểm tra đúng COM.
- Kiểm tra baud 921600.
- Kiểm tra Flow control = None.
- Nhấn Reset STM32.
- Nếu dùng USB-UART rời, kiểm tra PA2 TX đã nối sang RX của adapter và đã chung GND.

Nếu chữ bị rác, baud hoặc mức điện áp UART có thể sai.

## 5. Kiểm tra SPI trên Jetson

Mở Terminal 1 trên Jetson:

```bash
ls -l /dev/spidev2.0
```

Phải thấy thiết bị `/dev/spidev2.0`. Nếu báo `No such file or directory`, chưa được chạy controller; cần kiểm tra SPI pinmux/device tree trên Jetson.

Kiểm tra source:

```bash
cd /home/phi/robot_sources/Cheetah-Software
grep -n "spi_speed\|kActiveSpiBoards\|kBringupLocalLeg\|kBringupJoint" robot/src/rt/rt_spi.cpp
```

Kết quả phải tương ứng:

```text
spi_speed = 1000000
kActiveSpiBoards = 1
kBringupLocalLeg = 1
kBringupJoint = 0
```

## 6. Build code Jetson

```bash
cd /home/phi/robot_sources/Cheetah-Software
cmake --build build -j2 --target mcspi_ctrl
```

Kết quả cuối cần có:

```text
Built target robot
Built target mcspi_ctrl
```

## 7. Thử SPI khi motor chưa được enable

Giữ PuTTY mở trên Windows để xem log STM32.

Trên Jetson, chạy controller:

```bash
cd /home/phi/robot_sources/Cheetah-Software
sudo env LD_LIBRARY_PATH="$PWD/build:$LD_LIBRARY_PATH" \
  ./build/user/MiniCheetahSpi_Controller/mcspi_ctrl m r f
```

Chưa chạy chương trình phát `spi_debug_cmd`. Motor phải giữ `enabled=0`.

Trong PuTTY, kết quả tốt có dạng:

```text
[SPI] frames(+250), bad(+0), total=.../0, last_len=66, estop=1, enabled=0
```

Ý nghĩa:

- `frames` tăng: STM32 nhìn thấy các transaction SPI.
- `bad(+0)`: command checksum đúng.
- `last_len=66`: nhận đủ 66 từ 16-bit, tức 132 byte.
- `estop=1`: E-stop không bị kích hoạt.
- `enabled=0`: motor chưa được bật.

Phía Jetson không được xuất hiện:

```text
Couldn't open spidev 2.0
SPI_IOC_MESSAGE
short transfer
SPI ERROR BAD CHECKSUM
```

Chỉ khi PuTTY báo `frames` tăng, `bad=0`, `last_len=66` và Jetson không báo checksum thì mới coi SPI hai chiều đã thông.

## 8. Tạo chương trình gửi lệnh một motor

Mở Terminal 2 trên Jetson:

```bash
nano /tmp/send_one_motor.py
```

Dán nội dung:

```python
#!/usr/bin/env python3

import signal
import sys
import time

import lcm

sys.path.insert(
    0,
    "/home/phi/robot_sources/Cheetah-Software/lcm-types/python",
)

from leg_control_command_lcmt import leg_control_command_lcmt

LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
CHANNEL = "spi_debug_cmd"
COMMAND_INDEX = 3  # global leg 1, abad; STM32 CAN2, motor ID 1

running = True


def stop_handler(signum, frame):
    global running
    running = False


signal.signal(signal.SIGINT, stop_handler)
signal.signal(signal.SIGTERM, stop_handler)

lc = lcm.LCM(LCM_URL)

# Lần thử đầu chỉ dùng damping nhẹ, kp bằng 0.
target_position = 0.0
kp = 0.0
kd = 0.2

print("Sending to CAN2 motor ID 1. Press Ctrl+C to stop.")

while running:
    msg = leg_control_command_lcmt()
    msg.q_des[COMMAND_INDEX] = target_position
    msg.kp_joint[COMMAND_INDEX] = kp
    msg.kd_joint[COMMAND_INDEX] = kd
    lc.publish(CHANNEL, msg.encode())
    time.sleep(0.02)  # 50 Hz; watchdog là 250 ms

print("Stopped; waiting for watchdog to disable the motor...")
time.sleep(0.5)
```

Lưu trong nano:

```text
Ctrl+O
Enter
Ctrl+X
```

Kiểm tra Python LCM:

```bash
python3 -c "import lcm; print('LCM Python OK')"
```

Nếu thiếu module:

```bash
sudo apt update
sudo apt install python3-lcm
```

## 9. Thử motor lần đầu

Điều kiện an toàn:

- Motor/chân được nhấc khỏi mặt đất.
- E-stop ở trong tầm tay.
- Không đứng trong hướng quay của motor.
- SPI đã đạt `bad=0`, `last_len=66`.
- Script vẫn đang để `kp=0.0`, `kd=0.2`.

Giữ controller ở Terminal 1 và chạy ở Terminal 2:

```bash
python3 /tmp/send_one_motor.py
```

PuTTY phải hiện:

```text
[MOTOR] enter torque mode (estop=1)
```

và trạng thái:

```text
enabled=1
```

Lần thử `kp=0`, `kd=0.2` chủ yếu kiểm tra enable/damping, không yêu cầu motor chạy tới vị trí 0.

Nhấn `Ctrl+C` tại Terminal 2. Trong tối đa 250 ms PuTTY phải hiện:

```text
[MOTOR] exit torque mode
```

và:

```text
enabled=0
```

Nếu motor không disable, nhấn E-stop hoặc cắt nguồn công suất motor ngay.

## 10. Thử dịch chuyển nhỏ

Chỉ thực hiện sau khi bước damping và watchdog đạt yêu cầu.

Cần biết góc hiện tại của motor. Sửa script:

```python
target_position = GOC_HIEN_TAI
kp = 1.0
kd = 0.2
```

Chạy lại để kiểm tra motor giữ gần vị trí hiện tại. Sau đó mới thử:

```python
target_position = GOC_HIEN_TAI + 0.03
```

`0.03 rad` xấp xỉ `1.7°`.

Không đặt `q_des=0` với `kp>0` nếu chưa biết zero cơ khí, vì motor có thể quay về zero bất ngờ.

## 11. Tiêu chí đạt

### SPI Jetson ↔ STM32 đạt

```text
/dev/spidev2.0 tồn tại
Jetson không báo open/ioctl/short transfer
STM32 frames tăng liên tục
STM32 bad không tăng
STM32 last_len=66
Jetson không báo BAD CHECKSUM
```

### STM32 ↔ motor đạt

```text
PuTTY báo enter torque mode khi script chạy
enabled chuyển từ 0 sang 1
STM32 nhận feedback đúng CAN2/ID1
Motor phản ứng với damping hoặc bước vị trí nhỏ
Ctrl+C làm enabled trở về 0 trong tối đa 250 ms
```

## 12. Xử lý lỗi nhanh

| Hiện tượng | Kiểm tra |
|---|---|
| Không có `/dev/spidev2.0` | Jetson pinmux/device tree, SPI đã enable chưa |
| `frames=0` | CS0 pin 18→PA4, SCK, MOSI, GND |
| `last_len` khác 66 | CS bị nhả sớm, clock hoặc SPI mode |
| `bad` tăng | MOSI/byte order/nhiễu/tốc độ/GND |
| Jetson báo BAD CHECKSUM | MISO, PA6, byte order hoặc STM32 TX |
| `estop=0` | PB15 đang bị kéo thấp |
| `enabled` không lên 1 | Chưa có `spi_debug_cmd`, watchdog, checksum hoặc E-stop |
| Có enable nhưng motor không phản ứng | CAN2 PB8/PB9, transceiver, CANH/CANL, nguồn motor, ID 1 |
| Motor chạy mạnh bất ngờ | Nhấn E-stop; kiểm tra q_des, zero cơ khí và dấu tọa độ |


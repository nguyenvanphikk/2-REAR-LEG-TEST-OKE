# CẤU HÌNH CHẠY ROBOT THẬT MIT_3HP

## 1. Mục đích

File này tóm tắt cấu hình đang có trong source Jetson, các thay đổi đã thực
hiện cho MIT_3HP, tác động của từng thay đổi, các việc còn phải xác nhận và
các lệnh có thể sao chép trực tiếp để build/chạy robot.

> Trạng thái hiện tại: source và GUI đã build thành công, nhưng chưa được phép
> cho robot chịu tải/đứng tự do cho tới khi đã xác nhận mapping, chiều encoder,
> zero của đủ 12 motor, giới hạn khớp và watchdog STM32.

## 2. Cấu hình phần cứng đang dùng

### Jetson ↔ hai STM32 qua SPI

| Mục | Giá trị hiện tại |
|---|---|
| Board chân trước | `/dev/spidev1.0` |
| Board chân sau | `/dev/spidev1.1` |
| Mode | SPI Mode 0 |
| Tốc độ | 1 MHz |
| Bits/word | 8 trên Jetson; STM32 ghép thành 16 bit/word |
| CS | Active Low |
| `cs_change` | `0` |
| Kích thước transaction | 132 byte/board trong một `SPI_IOC_MESSAGE(1)`; CS giữ LOW suốt frame theo cấu hình |
| Thứ tự | Board 1.0 rồi board 1.1, tuần tự trong một SPI task |

Source hiện vẫn mở đúng hai thiết bị:

```cpp
open("/dev/spidev1.0", O_RDWR);
open("/dev/spidev1.1", O_RDWR);
```

Theo thông tin chủ robot cung cấp, cấu hình `spidev1.0/1.1` đã được dùng khi
thử điều khiển một motor trước đây. Source/Git không tự chứng minh được kết quả
thử phần cứng cũ; khi lắp đủ hệ thống vẫn phải kiểm tra lại từng board.

### Thứ tự chân trong source

```text
0 = FR (trước phải)
1 = FL (trước trái)
2 = HR (sau phải)
3 = HL (sau trái)
```

Mapping theo phep thu xoay motor cua chu robot (can kiem tra du 12 khop):

```text
/dev/spidev1.0: kenh STM32 0 → FL; kenh 1 → FR
/dev/spidev1.1: kenh STM32 0 → HL; kenh 1 → HR
Thu tu model/GUI van la FR, FL, HR, HL.
```

### Motor và tỷ số truyền

| Khớp | Hộp số motor | Truyền ngoài | Tổng rotor → khớp |
|---|---:|---:|---:|
| Abad | 6:1 | 1:1 | 6:1 |
| Hip | 6:1 | 1:1 | 6:1 |
| Knee | 6:1 | đai 1.5:1 | 9:1 |

Motor: Steadywin GIM8108-6, nguồn danh định 24 V, hỗ trợ MIT control.

## 3. Những phần đã thay đổi cho MIT_3HP

| Trạng thái | Thay đổi | Tác động ngắn gọn |
|---|---|---|
| ✅ | SPI chuyển sang `/dev/spidev1.0` và `1.1` | Đúng chân SPI Jetson đang sử dụng |
| ✅ | SPI 1 MHz, Mode 0, `cs_change=0` | Tốc độ thử nghiệm hiện tại; CS nhả sau mỗi transaction |
| ✅ | Hai board truyền tuần tự | Không có hai thread cùng truyền SPI đồng thời |
| ✅ | Kiểm tra ioctl đúng 132 byte và checksum | Gói lỗi không được đưa vào controller |
| ✅ | Mutex snapshot command/data | Tránh controller đọc/ghi trộn dữ liệu giữa hai chu kỳ |
| ✅ | Watchdog GUI 300 ms | Mất GUI → PASSIVE, xóa joystick, disable motor |
| ⚠️ | Không có watchdog phản hồi SPI trên Jetson | SPI lỗi chỉ hiện cảnh báo, gói lỗi giữ mẫu hợp lệ gần nhất. Người vận hành xác nhận firmware đã nạp không còn tự cắt motor sau 5 gói lỗi; cần kiểm tra đúng phiên bản firmware trên cả hai board |
| ✅ | PASSIVE thực sự gửi `flags=0` | STM32 được yêu cầu disable motor |
| ✅ | Bỏ tự chạy `JPosInitializer` | Bật chương trình không tự kéo chân qua pose Mini Cheetah cũ |
| ⚠️ | Nút STAND_UP tạm là thử khớp bốn chân khi treo robot | Cả FR/FL/HR/HL nhận enable và q_des=[0,0,0.02] rad, Kp=5/Kd=0.2; không có đoạn tăng góc trung gian. Không còn tự ngắt theo tuổi phản hồi SPI hoặc ngưỡng q/qd. Giữ đến khi PASSIVE/E-STOP, mất GUI/IMU hoặc bảo vệ STM32; không phải tư thế đứng; Balance/Locomotion bị khóa |
| ✅ | Offset Jetson của 12 khớp bằng 0 | Phù hợp phương án set-zero motor tại pose chân duỗi thẳng |
| ✅ | Tỷ số truyền model 6/6/9 | Phản ánh hộp số motor và đai knee 1.5:1 |
| ✅ | Knee scale có độ lớn `1/1.5` | Quy đổi feedback đầu ra motor sang góc khớp knee |
| ✅ | Hình học, vị trí khớp, FK/Jacobian theo URDF | Chân controller gần thiết kế CAD hơn Mini Cheetah gốc |
| ✅ | Mass/COM/inertia theo URDF | Model động lực học gần robot MIT_3HP hơn |
| ✅ | Tổng khối lượng Dense MPC `10.865863917 kg` | MPC dùng tổng khối lượng URDF thay cho 9 kg |
| ✅ | Body height locomotion giữ `0.29 m` | Chưa thay đổi khi chưa xác nhận pose CAD chịu tải |
| ✅ | `use_rc=0` và GUI riêng | GUI trên Jetson thay tay điều khiển SBUS |
| ✅ | Gait mặc định Trotting (`cmpc_gait=0`) | Không còn dùng giá trị 9 rơi ngầm về gait mặc định |
| ✅ | GUI có E-stop, FSM, joystick 2D, gait và telemetry | Điều khiển vận tốc và theo dõi robot ngay trên Jetson |

Các thay đổi I/O, GUI và watchdog không sửa công thức MPC, WBC hoặc state
estimator. Mapping gait 6/7 được nối đúng tới `Walking/Walking2` đã tồn tại.

## 4. Chiều, scale và offset hiện tại

Thứ tự mảng là `[FR, FL, HR, HL]`:

```cpp
abad_side_sign = {-1, -1, +1, +1};
hip_side_sign  = {-1, +1, -1, +1};
knee_side_sign = {-0.6666667, +0.6666667,
                  -0.6666667, +0.6666667};

abad_offset = {0, 0, 0, 0};
hip_offset  = {0, 0, 0, 0};
knee_offset = {0, 0, 0, 0};
```

`0.6666667 = 1/1.5` vừa chứa scale truyền đai vừa chứa dấu. Các dấu trên vẫn
là giá trị tạm theo mapping cũ; chưa được coi là đúng cho robot thật cho tới
khi quay thử từng khớp.

## 5. GUI điều khiển

GUI hiện có:

- E-stop/PASSIVE và trạng thái enable motor.
- Trình tự `PASSIVE → STAND_UP → BALANCE → LOCOMOTION`.
- Hai joystick 2D; có thể vừa tiến/lùi vừa đi ngang.
- Giới hạn lệnh mặc định 20%.
- Chọn Trotting, Standing, Trot Running, Walking, Walking2 hoặc Pacing.
- Trạng thái riêng hai SPI board, tần số response hợp lệ và tuổi dữ liệu.
- Bộ đếm ioctl, thiếu 132 byte, phản hồi bị loại; tỷ lệ bị loại của từng board
  tính trên 5 giây gần nhất. Phản hồi bị loại gồm sai XOR, toàn 0 và giá trị
  motor bất thường.
- IMU quaternion, gyro và acceleration.
- `q`, `qd`, `q_des`, `qd_des`, `tau_est`, Kp/Kd, sign/scale, offset, ratio và
  enable của 12 khớp.

Đơn vị lệnh GUI:

| Lệnh | Đơn vị | Giới hạn code gốc | 20% mặc định |
|---|---|---:|---:|
| Tiến/lùi `vx` | m/s | ±3.0 | ±0.6 |
| Ngang `vy` | m/s | ±2.0 | ±0.4 |
| Yaw rate | rad/s | ±2.5 | ±0.5 |
| Pitch | rad | ±0.4 | ±0.08 |

Joystick gửi vận tốc thân mong muốn. MPC/WBC tự tính lực chân và torque từng
khớp. GUI không cho nhập torque trực tiếp. `tau_est` chỉ là torque controller
ước tính, không phải torque/dòng điện đo từ motor.

## 6. Phần còn phải làm trước khi robot đứng/đi thật

### Bắt buộc xác nhận trên phần cứng

- [ ] Mapping `/dev/spidev1.0 → FR/FL`, `/dev/spidev1.1 → HR/HL` và thứ tự
  hai chân trong mỗi board.
- [ ] CAN ID/thứ tự Abad–Hip–Knee của đủ 12 motor.
- [ ] Chiều encoder của từng khớp; cập nhật 12 dấu sau khi đo.
- [ ] Set zero đủ 12 motor tại pose chân duỗi thẳng và xác nhận q gần 0.
- [ ] Xác nhận feedback knee nằm trước hay sau đai 1.5:1 và đai có đảo chiều.
- [ ] Kiểm tra VectorNav tại `/dev/ttyUSB0`, đúng +X trước, +Y trái, +Z lên.
- [ ] Cân robot hoàn chỉnh; kiểm tra URDF đã gồm pin, Jetson, driver và dây.

### Bắt buộc bổ sung/xác nhận về an toàn

- [ ] Watchdog trong mỗi STM32: mất command SPI hợp lệ phải đặt torque/Kp/Kd
  bằng 0 và disable 6 motor của board.
- [ ] E-stop vật lý ngắt enable hoặc nguồn công suất động cơ 24 V.
- [ ] Clamp góc command và kiểm tra feedback theo giới hạn khớp đã đo.
- [ ] Xác nhận giới hạn vận tốc an toàn từng khớp.
- [ ] Xác nhận torque/current liên tục và peak của GIM8108-6 trong robot.
- [ ] STM32 gửi lỗi CAN, quá dòng, quá nhiệt, điện áp và trạng thái driver nếu
  muốn GUI cảnh báo các lỗi này.

Giới hạn URDF chỉ dùng làm ứng viên kiểm tra, chưa coi là giới hạn phần cứng:

| Khớp | Góc URDF | Velocity URDF | Effort URDF |
|---|---:|---:|---:|
| Abad | -0.6…+0.6 rad | 20 rad/s | 17 Nm |
| Hip | -1.57…+1.57 rad | 20 rad/s | 17 Nm |
| Knee | -2.4…+0.2 rad | 12 rad/s | 25 Nm |

### Cần hiệu chỉnh sau khi đứng ổn định

- [ ] Gain Stand Up, Balance, MPC/WBC cho robot nặng hơn.
- [ ] Inertia/COM bằng CAD và phép đo robot lắp hoàn chỉnh.
- [ ] Noise/bias VectorNav bằng log thực tế.
- [ ] Ma sát, độ rơ, hiệu suất truyền đai và giới hạn lực tiếp xúc.
- [ ] Recovery Stand trước khi cho phép sử dụng; pose hiện vẫn từ Mini Cheetah.

## 7. Lệnh build

Chỉ cần build lại khi source C/C++ hoặc CMake thay đổi.

### Build controller robot thật

```bash
cd /home/hp3/robot_project/Cheetah-Software-MIT
cmake -S . -B jetson-build -DMINI_CHEETAH_BUILD=TRUE
cmake --build jetson-build --target mit_ctrl -j4
```

### Build GUI

```bash
cd /home/hp3/robot_project/Cheetah-Software-MIT
cmake -S . -B build-sim
cmake --build build-sim --target mit3hp_gui -j4
```

## 8. Lệnh chạy mỗi lần bật robot

### Terminal 1 — mở GUI

```bash
cd /home/hp3/robot_project/Cheetah-Software-MIT
./scripts/run_mit3hp_gui.sh
```

### Terminal 2 — chạy controller robot thật

```bash
cd /home/hp3/robot_project/Cheetah-Software-MIT/jetson-build
sudo LD_LIBRARY_PATH=. ./user/MIT_Controller/mit_ctrl m r f
```

Ý nghĩa:

```text
m = model Mini Cheetah đã sửa trực tiếp cho MIT_3HP
r = robot thật
f = nạp tham số từ YAML
```

Không cần chọn lại `m/r/f` trên GUI. Nếu source và cấu hình không đổi, những
lần bật máy sau chỉ cần chạy hai lệnh trên, không cần build lại.

## 9. Trình tự thử robot thật

1. Treo robot để bốn chân không chạm đất và chuẩn bị E-stop vật lý.
2. Bật Jetson/STM32 với nguồn công suất motor chưa enable.
3. Mở GUI; kiểm tra Controller, VectorNav, SPI và bộ đếm lỗi từng board. Màu xanh XOR chưa xác nhận STM32 có nguồn.
4. Chạy controller; robot phải ở PASSIVE, motor mềm, không tự chạy pose.
5. Test từng motor với giới hạn thấp; ghi mapping, dấu và zero đủ 12 khớp.
6. Kiểm tra watchdog GUI và E-stop vật lý; Jetson hiện không có watchdog mất SPI.
7. Chỉ thử nút khớp bốn chân 0/0/0.02 khi robot được treo, đủ 12 khớp đã kiểm tra zero/chiều và hai board SPI có phản hồi ổn định. Phần mềm không còn tự kiểm tra các điều kiện q/qd và tuổi SPI này trước/trong khi thử; người vận hành phải kiểm tra chúng. Cả bốn chân đều có lực; bấm PASSIVE/E-STOP để tắt lực.
8. Hoàn thiện đủ 12 phản hồi motor, mapping, chiều, zero và SPI trước khi khôi phục chuyển động.

## 10. Hành vi an toàn cần nhớ

- PASSIVE nghĩa là controller ngừng giữ tư thế và gửi `flags=0`; robot có thể
  hạ xuống hoặc ngã vì trọng lực.
- E-stop GUI là E-stop phần mềm, không thay thế nút ngắt nguồn/enable vật lý.
- Mất GUI quá 300 ms sẽ yêu cầu về PASSIVE. Jetson không còn tự chuyển PASSIVE
  theo tuổi phản hồi SPI hoặc q/qd trong phép thử bốn chân.
- Firmware đang nạp có thể khác file nguồn trên ổ đĩa. Người vận hành xác nhận
  bản đã nạp đã bỏ tự cắt lực sau 5 gói SPI lỗi; không dựa vào bảo vệ này.
  Khi đường truyền lỗi, dùng E-stop hoặc ngắt lực vật lý.
- Nếu Jetson mất nguồn hoặc treo, GUI không thể gửi PASSIVE; E-stop/ngắt lực
  vật lý là biện pháp cần có độc lập với Jetson.

## 11. Quản lý mã nguồn

- Tên nhánh Git đang dùng là `thu-2-chan-truoc`; phép thử hiện tại đã đổi thành phép thử khớp bốn chân 0/0/0.02.
- Mã STM32 SPINDE nằm ngoài repository này; kiểm tra phiên bản đã nạp cho
  từng board trước khi thử motor.
- Không đưa các thư mục `build-sim/`, `jetson-build/`,
  `jetson-build-aarch64/` hoặc tệp test sinh tự động lên Git.

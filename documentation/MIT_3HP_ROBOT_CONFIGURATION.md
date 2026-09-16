# Hồ sơ cấu hình robot MIT_3HP

## Mục đích

File này là nguồn thông tin cấu hình cho robot MIT_3HP. Khi yêu cầu sửa source code, hãy gửi file này cùng source/URDF mới nhất và yêu cầu trợ lý:

> Đọc toàn bộ `MIT_3HP_ROBOT_CONFIGURATION.md`, đối chiếu với source hiện tại, chỉ dùng các giá trị trong mục **Đã xác nhận** để sửa code. Không tự đoán các mục **Chưa xác nhận**. Sau khi sửa, cập nhật lại file này, liệt kê file/dòng đã đổi và kết quả kiểm tra.

Nếu thông tin mới khác file này, thông tin mới do chủ robot cung cấp được ưu tiên và phải được cập nhật vào file.

## Quy ước trạng thái

- **Đã xác nhận**: được phép dùng để sửa code.
- **Từ URDF/CAD, cần kiểm chứng**: có thể dùng để dựng model thử nghiệm; phải đo hoặc kiểm tra lại trước khi chạy robot thật.
- **Chưa xác nhận**: không tự chọn giá trị.

## Thông số đã xác nhận

### Jetson SPI

| Tham số | Giá trị |
|---|---|
| SPI mode | `SPI_MODE_0` |
| Bits/word | 8 |
| Tốc độ | 6 MHz (`6000000` Hz) |
| Board 0 / CS0 | `/dev/spidev1.0` |
| Board 1 / CS1 | `/dev/spidev1.1` |
| Kích thước mỗi transaction | 132 byte |
| `cs_change` | `0` — nhả CS sau mỗi message |
| CS polarity | Active LOW |
| Thứ tự truyền | Board 0 rồi board 1, tuần tự trong một SPI task |

Đấu chân Jetson (số chân vật lý trên header):

| Chân | Chức năng |
|---:|---|
| 13 | SCK |
| 22 | MISO |
| 37 | MOSI |
| 18 | CS0, nối A2 |
| 16 | CS1 |
| 39 | GND |

Yêu cầu xử lý response:

- Chỉ đưa response vào controller khi `ioctl()` trả đúng 132 byte.
- Kiểm tra checksum trước khi cập nhật dữ liệu controller.
- Response thiếu byte, lỗi `ioctl` hoặc sai checksum phải bị loại bỏ và ghi log.
- Giữ dữ liệu hợp lệ gần nhất của board khi response mới không hợp lệ.
- Không cho hai thread tự do truyền đồng thời trên hai fd.

### Motor và truyền động

Motor sử dụng: **Steadywin GIM8108-6**, hỗ trợ MIT control.

| Thông số | Giá trị |
|---|---:|
| Điện áp danh định | 24 V |
| Dải điện áp | 24–48 V |
| Công suất | 125 W |
| Torque danh định | 6 Nm |
| Torque stall | 15 Nm |
| Tốc độ danh định sau hộp số | 200 rpm |
| Tốc độ cực đại sau hộp số | 350 rpm |
| Dòng danh định | 5 A |
| Dòng stall | 20 A |
| Điện trở phase-to-phase | 0.28 Ω |
| Điện cảm phase-to-phase | 0.17 mH |
| Speed constant | 100 rpm/V |
| Torque constant theo catalog | 0.091 Nm/A |
| Rotor inertia theo catalog | 800 g·cm² = 0.00008 kg·m² |
| Số pole pairs | 21 |
| Tỉ số hộp số tích hợp motor | 6:1 |
| Loại hộp số | Planetary |
| Khối lượng có driver | 567 g |

### Tỉ số truyền các khớp

| Khớp | Hộp số motor | Truyền ngoài | Tổng rotor → khớp |
|---|---:|---:|---:|
| Abad | 6 | 1.0 | 6:1 |
| Hip | 6 | 1.0 | 6:1 |
| Knee | 6 | **1.5 truyền đai** | **9:1** |

Quy ước công thức knee, nếu không có đảo chiều bởi cách bắt dây đai:

```text
q_knee_joint  = q_motor_output / 1.5
qd_knee_joint = qd_motor_output / 1.5
tau_joint      = tau_motor_output × 1.5 × hiệu_suất
```

Nếu MIT driver đã báo vị trí/torque sau toàn bộ cơ cấu truyền đai thì không scale thêm lần nữa. Phải xác định điểm đo của feedback trước khi sửa `knee_side_sign`.

Trong model động lực học, nếu tham số gear ratio được định nghĩa từ rotor motor đến khớp thì dùng:

```text
abadGearRatio = 6
hipGearRatio  = 6
kneeGearRatio = 9
```

Không dùng lại knee gear ratio `9.33` của Mini Cheetah.

## Thông số từ URDF/CAD, cần kiểm chứng trên robot thật

URDF nguồn: `quadruped_robot.urdf` trong gói `URDF_THỰC TẾ MỚI NHẤT.zip` ngày 2026-09-15.

### Khối lượng

| Thành phần | Giá trị URDF |
|---|---:|
| Base | 4.625186964 kg |
| Mỗi chân | khoảng 1.559401 kg |
| IMU link | 0.003076 kg |
| Tổng URDF | 10.865863917 kg |

Phải cân robot hoàn chỉnh. Kiểm tra URDF đã bao gồm pin, Jetson, STM32, driver, dây điện, gá lắp và toàn bộ motor hay chưa.

### Base

```text
COM = (0.001123603, 0.001244030, -0.002002691) m
Ixx = 0.0059740844 kg·m²
Ixy = -0.000001752888 kg·m²
Ixz = -0.000001339932 kg·m²
Iyy = 0.0055722435 kg·m²
Iyz = 0.0001522085 kg·m²
Izz = 0.0069160685 kg·m²
```

### Vị trí khớp và chiều dài

| Quan hệ | Giá trị URDF |
|---|---|
| Base → abad trước | X = +0.1513 m, Y = ±0.0500 m |
| Base → abad sau | X = -0.1513 m, Y = ±0.0500 m |
| Abad → hip trước | X ≈ +0.0560 m, Y = ±0.0133 m |
| Abad → hip sau | X ≈ -0.0560 m, Y = ±0.0133 m |
| Hip → knee | Z ≈ -0.2100 m, Y = ±0.0720 m |
| Knee → foot | Z ≈ -0.185507 m |
| IMU so với base | XYZ = (-0.031854, 0, 0.033959) m; RPY = (0, 0, 0) |

Động học Mini Cheetah hiện chỉ dùng các chiều dài rút gọn. Model MIT_3HP phải biểu diễn đúng các offset X/Y/Z từ URDF, đặc biệt offset X giữa abad và hip.

Các chiều dài suy ra từ URDF:

```text
Độ dài vector hip → knee        = sqrt(0.072² + 0.210²) = 0.222000 m
Độ dài knee → foot              = 0.185507 m
Tổng độ dài hai đoạn dưới       = 0.407507 m
Khoảng cách abad → foot tại q=0 = 0.408458 m
```

Giá trị ứng viên ban đầu cho `_maxLegLength` là khoảng `0.40846 m`, nhưng phải kiểm tra lại bằng forward kinematics đầy đủ vì URDF có offset X/Y và các đoạn không nằm trên một đường thẳng. Không đưa giá trị này vào source thật cho đến khi model transform MIT_3HP được dựng và kiểm chứng.

URDF đã chứa vị trí tương đối của base, abad, hip, knee, foot và IMU; đồng thời chứa mass, COM và inertia của từng link. Source hiện tại **chưa dùng các vị trí đó**, mà vẫn dùng model Mini Cheetah rút gọn trong `buildMiniCheetah()`.

### Giới hạn khớp trong URDF

| Khớp | Góc dưới | Góc trên | Effort URDF | Velocity URDF |
|---|---:|---:|---:|---:|
| Abad | -0.6 rad | +0.6 rad | 17 Nm | 20 rad/s |
| Hip | -1.57 rad | +1.57 rad | 17 Nm | 20 rad/s |
| Knee | -2.4 rad | +0.2 rad | 25 Nm | 12 rad/s |

Các effort 17/17/25 Nm lớn hơn torque stall 15 Nm trong catalog motor. Không dùng các effort URDF làm giới hạn phần cứng trước khi xác minh thiết kế và khả năng chịu tải.

Source hiện chưa có clamp tổng quát cho góc mong muốn của 12 khớp trước khi đóng gói SPI. `SafetyChecker` hiện kiểm tra orientation, vị trí bàn chân mong muốn và lực feed-forward; các kiểm tra đó không thay thế joint-angle limit. Khi triển khai MIT_3HP phải clamp cả command và kiểm tra feedback theo giới hạn riêng của từng khớp.

## Trạng thái wimp mode và safe mode trong source hiện tại

- Có các hằng `wimp_torque = {6, 6, 6}` và `disabled_torque = {0, 0, 0}`.
- `fake_spine_control()` chọn wimp khi bit 1 của `flags` được đặt.
- Luồng `LegController::updateCommand()` hiện chỉ gửi `flags = 1` khi enable, nên không chủ động bật bit wimp.
- Torque tính bởi `fake_spine_control()` không được dùng để clamp gói command gửi motor thật.
- FSM có `PASSIVE`, `ESTOP` và một số kiểm tra safety, nhưng chưa tạo thành lớp bảo vệ đầy đủ cho motor GIM8108-6.

Vì vậy, robot hiện **chưa có wimp/safe mode phần cứng hoàn chỉnh**. Khi triển khai cần thêm ít nhất: joint-angle limit, velocity limit, torque/current limit, timeout command, lỗi SPI/checksum liên tiếp, mất CAN, over-temperature và hành vi disable/damping đã được thử trên STM32.

## Quy tắc kiểu số khi viết C++

- Trong biến/mảng kiểu `float`, nên viết hậu tố `f`, ví dụ `0.1513f`, `6.f`, `1.5f`.
- Trong code template `Quadruped<T>`, ưu tiên `T(0.1513)` hoặc `static_cast<T>(0.1513)` nếu muốn giữ đúng kiểu tổng quát cho cả `float` và `double`.
- Không thêm `f` vào số lấy từ URDF nếu đang lưu bằng `double` và cần giữ độ chính xác CAD.
- Các số nguyên như `6` hoặc `9` không bắt buộc có `f`; khi gán vào `float`, có thể viết `6.f` và `9.f` để thể hiện chủ ý.

## Thông số chưa xác nhận — bắt buộc đo

- Mapping chính xác `/dev/spidev1.0` và `/dev/spidev1.1` tới các chân `FR, FL, HR, HL`.
- Thứ tự hai chân và ba motor trong mỗi gói STM32.
- CAN ID của đủ 12 motor.
- Chiều dương thực tế của từng encoder và từng khớp.
- Zero offset của đủ 12 khớp ở tư thế chuẩn.
- Feedback knee từ driver nằm trước hay sau bộ truyền đai 1.5.
- Bộ truyền đai knee có đảo chiều quay hay không.
- Hiệu suất truyền đai knee.
- Torque/current limit liên tục và peak dùng thực tế.
- Điện áp bus thực tế.
- Hướng trục IMU thực so với body frame.
- Ma sát nhớt, ma sát khô và độ rơ từng khớp.
- Tổng khối lượng và COM robot sau khi lắp hoàn chỉnh.

## Quy ước thứ tự chân của source MIT

Source MIT dùng thứ tự:

```text
0 = FR (Front Right)
1 = FL (Front Left)
2 = HR/RR (Hind/Rear Right)
3 = HL/RL (Hind/Rear Left)
```

Mọi mảng `sign[4]`, `offset[4]`, command và response phải theo đúng thứ tự này. Không suy ra thứ tự board chỉ từ tên URDF.

## Các phần source dự kiến phải sửa

Chỉ sửa khi các thông số liên quan đã được xác nhận.

1. `common/include/Dynamics/`
   - Tạo `MIT3HP.h` với hàm `buildMIT3HP()`.
   - Nhập mass, COM, inertia, gear ratio và vị trí khớp của robot.
   - Không ghi đè model Mini Cheetah gốc.

2. `common/src/Dynamics/Quadruped.cpp` và động học chân
   - Mở rộng transform nếu cấu trúc hiện tại không biểu diễn được offset X/Y/Z của URDF.
   - Kiểm tra forward kinematics và Jacobian với các tư thế CAD đã biết.

3. `robot/src/RobotRunner.cpp`
   - Chọn `buildMIT3HP()` khi chạy robot MIT_3HP.

4. `robot/src/rt/rt_spi.cpp`
   - Cập nhật mapping chân, sign và zero offset đã đo.
   - Xử lý đúng truyền đai knee 1.5 tùy điểm đo feedback.
   - Cập nhật torque limit sau khi xác nhận.

5. `robot/src/HardwareBridge.cpp/.h`
   - Dùng một SPI task truyền board 0 rồi board 1.
   - Bảo vệ snapshot `_spiCommand` và `_spiData` giữa controller thread và SPI thread.
   - Không giữ mutex chia sẻ với controller trong lúc `ioctl()`.

6. MPC, balance controller và WBC
   - Loại bỏ các mass/inertia hardcode của Mini Cheetah/Cheetah 3.
   - Dùng tổng mass và base inertia của MIT_3HP.
   - Tính lại body height, nominal foot position, max ground reaction force và gain.

7. State estimator và IMU
   - Áp dụng rotation từ IMU frame sang body frame.
   - Cân nhắc bù gia tốc do IMU đặt lệch COM.
   - Tune lại noise/bias bằng log thực tế.

8. Safety
   - Giới hạn góc, tốc độ, torque và dòng trước khi gửi lệnh.
   - Timeout/mất SPI phải đưa command về disable hoặc damping an toàn.
   - Không dùng dữ liệu response sai checksum hoặc thiếu byte.

## Tiến độ thay đổi từng bước

- Bước 4 — `side_sign` và zero offset: **tạm hoãn**, chờ đo thực tế đủ 12 khớp.
- Bước 5 — đồng bộ controller/SPI: **đã thực hiện**.
  - Một SPI task tiếp tục truyền board 0 rồi board 1 tuần tự.
  - Mutex chỉ bảo vệ lúc sao chép snapshot command/response.
  - Không giữ mutex trong lúc `ioctl()`.
  - Khởi tạo vùng command/response chia sẻ về 0 trước khi các task chạy.

## Quy trình kiểm tra trước khi bật đủ 12 motor

1. Kê robot để chân không chạm đất.
2. Chỉ enable một motor với torque/current thấp.
3. Xác nhận CAN ID, joint, chiều dương và zero.
4. Lặp lại đủ 12 motor và ghi kết quả vào bảng bên dưới.
5. Kiểm tra forward kinematics: chuyển động q dương phải tạo chuyển động chân đúng convention của controller.
6. Kiểm tra board 0 và board 1 tuần tự, response đủ 132 byte và checksum đúng.
7. Enable từng chân, sau đó bốn chân ở torque thấp.
8. Chỉ đặt robot xuống đất sau khi joint limit và mất truyền thông đã được thử.

## Bảng hiệu chuẩn 12 khớp

Điền bảng này sau khi đo. `Sign` chỉ nhận `+1` hoặc `-1`. Offset dùng radian tại đầu ra khớp.

| Leg | Joint | Board | Slot/CAN ID | Sign | Zero offset (rad) | Feedback trước/sau truyền ngoài | Đã thử |
|---|---|---|---|---:|---:|---|---|
| FR | Abad | TBD | TBD | TBD | TBD | Không có | Chưa |
| FR | Hip | TBD | TBD | TBD | TBD | Không có | Chưa |
| FR | Knee | TBD | TBD | TBD | TBD | TBD | Chưa |
| FL | Abad | TBD | TBD | TBD | TBD | Không có | Chưa |
| FL | Hip | TBD | TBD | TBD | TBD | Không có | Chưa |
| FL | Knee | TBD | TBD | TBD | TBD | TBD | Chưa |
| RR/HR | Abad | TBD | TBD | TBD | TBD | Không có | Chưa |
| RR/HR | Hip | TBD | TBD | TBD | TBD | Không có | Chưa |
| RR/HR | Knee | TBD | TBD | TBD | TBD | TBD | Chưa |
| RL/HL | Abad | TBD | TBD | TBD | TBD | Không có | Chưa |
| RL/HL | Hip | TBD | TBD | TBD | TBD | Không có | Chưa |
| RL/HL | Knee | TBD | TBD | TBD | TBD | TBD | Chưa |

## Nhật ký thay đổi

| Ngày | Thay đổi | Trạng thái |
|---|---|---|
| 2026-09-15 | Tạo hồ sơ từ URDF, catalog GIM8108-6 và source hiện tại | Tài liệu |
| 2026-09-15 | Xác nhận truyền đai knee 1.5:1; tổng rotor → knee 9:1 | Đã xác nhận |
| 2026-09-15 | Sửa cách đọc catalog: `6:1` là tỉ số sáu trên một, không phải 6.1 | Đã xác nhận |
| 2026-09-16 | Bước 1: thay trực tiếp hình học trong `buildMiniCheetah()` và cập nhật forward kinematics/Jacobian | Đã thực hiện |
| 2026-09-16 | Bước 2: thay mass, COM, inertia từ URDF; gộp base+IMU và shank+foot bằng định lý trục song song | Đã thực hiện |
| 2026-09-16 | Bước 3: đổi tỉ số truyền abad/hip/knee thành 6/6/9 | Đã thực hiện |
| 2026-09-16 | Bước 4: chiều encoder, `side_sign` và zero offset | Tạm hoãn, chờ đo đủ 12 khớp |
| 2026-09-16 | Bước 5: thêm mutex snapshot giữa controller và SPI task; không khóa trong `ioctl()` | Đã thực hiện, build `robot` đạt |
| 2026-09-16 | Bước 6: đổi tổng khối lượng Dense Convex MPC từ 9 kg thành 10.865863917 kg theo URDF | Đã thực hiện |
| 2026-09-16 | Đổi độ lớn hệ số quy đổi knee trong SPI từ 0.6429 thành 1/1.5 = 0.6666667; dấu từng chân vẫn chờ đo | Đã thực hiện |

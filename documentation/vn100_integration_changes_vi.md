# Tích hợp VN-100S-CR trên Jetson: thay đổi và đánh giá ảnh hưởng

## 1. Phạm vi

Tài liệu này ghi lại các thay đổi đã thực hiện để Mini Cheetah nhận dữ liệu thật từ IMU VectorNav VN-100S-CR qua cáp USB–RS232 trên Jetson.

Các thay đổi trong tài liệu chỉ liên quan tới IMU và đường dữ liệu từ IMU vào state estimator. Phần SPI với hai STM32, firmware STM32, motor driver và watchdog tắt motor chưa được hoàn thiện trong giai đoạn này.

Các file đã sửa cho tích hợp IMU:

- `robot/src/HardwareBridge.cpp`
- `robot/src/rt/rt_vectornav.cpp`
- `robot/include/rt/rt_vectornav.h`

## 2. Luồng dữ liệu sau khi tích hợp

```text
VN-100S-CR
  -> USB–RS232/FTDI
  -> rt_vectornav.cpp
  -> vectornav_handler()
  -> MiniCheetahHardwareBridge::_vectorNavData
  -> RobotRunner
  -> StateEstimatorContainer
  -> VectorNavOrientationEstimator
  -> StateEstimate
  -> bộ điều khiển robot
```

`HardwareBridge` truyền đúng địa chỉ dữ liệu IMU cho `RobotRunner`:

```cpp
_robotRunner->vectorNavData = &_vectorNavData;
```

`RobotRunner` dùng pointer này để khởi tạo `StateEstimatorContainer` và chọn `VectorNavOrientationEstimator` khi chạy robot thật, không dùng cheater mode.

## 3. Các thay đổi đã thực hiện

### 3.1. Chọn VectorNav thay cho Microstrain

Đã tắt macro:

```cpp
//#define USE_MICROSTRAIN
```

Kết quả:

- Mini Cheetah gọi `init_vectornav()`.
- Dữ liệu VN-100 được đưa vào `_vectorNavData`.
- Không khởi tạo thread và logger Microstrain.
- Không còn mở Lord/Microstrain IMU ở cổng serial khác.

Ảnh hưởng:

- Bản build hiện tại dành cho VN-100, không dành cho Microstrain.
- Nếu thay lại phần cứng Microstrain thì phải bật lại macro và build lại.

### 3.2. Hỗ trợ VN-100 qua USB–RS232

Cổng mặc định đã đổi từ:

```text
/dev/ttyS0
```

sang:

```text
/dev/ttyUSB0
```

Đồng thời có thể chọn cổng mà không sửa source bằng biến môi trường:

```bash
CHEETAH_VECTORNAV_PORT=/dev/serial/by-id/<ten-thiet-bi-FTDI>
```

Thứ tự ưu tiên:

1. Dùng `CHEETAH_VECTORNAV_PORT` nếu được khai báo.
2. Nếu không, dùng `/dev/ttyUSB0`.

Ảnh hưởng:

- Có thể dùng đường dẫn `/dev/serial/by-id/...` ổn định khi lắp trên robot.
- Tránh mở nhầm thiết bị nếu số `ttyUSB0`, `ttyUSB1` thay đổi.
- Nếu biến môi trường chứa đường dẫn sai, khởi tạo IMU thất bại và chương trình dừng.

### 3.3. Tự dò baudrate

Driver tự thử:

```text
921600 baud
115200 baud
```

Nếu kết nối ở 115200, driver chuyển VN-100 và cổng host lên 921600 trước khi bật binary stream.

Lý do:

- 115200 không đủ an toàn cho quaternion, angular rate và acceleration ở 200 Hz.
- VN-100 có thể vẫn ở 921600 sau lần chạy trước.

Ảnh hưởng:

- Không cần sửa source khi trạng thái baudrate của cảm biến thay đổi giữa hai mức trên.
- Khi cảm biến ở 115200, lần thử 921600 đầu tiên có thể tạo một khoảng chờ timeout lúc khởi động.
- Cáp USB–RS232 phải hoạt động ổn định ở 921600.
- Nếu cảm biến được đặt ở baudrate khác 115200 và 921600, driver không tìm thấy cảm biến.

### 3.4. Binary Output 1 dùng PORT1

Đã đổi:

```cpp
ASYNCMODE_PORT2
```

thành:

```cpp
ASYNCMODE_PORT1
```

Binary packet gồm:

- Quaternion: 4 giá trị.
- Angular rate: 3 giá trị.
- Acceleration: 3 giá trị.

Rate divisor là 4, tương ứng khoảng:

```text
800 / 4 = 200 Hz
```

Ảnh hưởng:

- Phù hợp với cổng serial chính đang nối qua cáp USB–RS232.
- Nếu sau này đấu sang serial port vật lý thứ hai của VN-100 thì cấu hình port phải được xem xét lại.

### 3.5. Cấu hình cảm biến có thể chạy lặp lại

Driver đọc cấu hình hiện tại trước khi ghi:

- Nếu ASCII asynchronous output đã bằng 0 thì không ghi lại.
- Nếu heading mode đã là `Relative` thì không ghi lại.
- Nếu Binary Output 1 đã đúng thì không ghi lại.

Lý do:

- Firmware VN-100S-CR có thể trả `NOT_SUPPORTED` khi nhận một số lệnh ghi dư thừa.
- Khởi động lại chương trình không nên thất bại chỉ vì cảm biến đã được cấu hình đúng từ lần trước.

Ảnh hưởng:

- Giảm số lần ghi register của cảm biến.
- Cho phép chạy lại chương trình ổn định hơn.
- Nếu cấu hình hiện tại khác yêu cầu và cảm biến từ chối lệnh ghi, khởi tạo vẫn thất bại theo cơ chế fail-safe.

### 3.6. Kiểm tra nội dung packet IMU

Trước khi cập nhật `_vectorNavData`, callback kiểm tra:

- Packet phải là binary.
- Packet phải đúng nhóm quaternion + angular rate + acceleration.
- Quaternion, gyro và acceleration không chứa `NaN` hoặc `Inf`.
- Bình phương norm quaternion nằm trong khoảng 0.81 đến 1.21.
- Bình phương norm acceleration không lớn hơn 10000.

Packet không hợp lệ bị bỏ và không cập nhật timestamp hợp lệ.

Ảnh hưởng tới điều khiển:

- Giảm nguy cơ đưa dữ liệu hỏng rõ ràng vào state estimator.
- Giới hạn acceleration đang để rộng nhằm không loại nhầm chuyển động động lực học bình thường.
- Đây không thay thế kiểm tra calibration, bias, sai hướng lắp hoặc rung cơ khí.

### 3.7. Xác nhận packet thật trong lúc khởi động

Sau khi cấu hình binary output và đăng ký callback, driver chờ tối đa 2 giây để nhận một packet hợp lệ.

Chỉ khi có packet hợp lệ mới in:

```text
[rt_vectornav] IMU is set up and streaming valid data!
```

Nếu hết thời gian mà không có packet hợp lệ, `init_vectornav()` trả về thất bại.

`MiniCheetahHardwareBridge` hiện gọi `initError()` khi khởi tạo IMU thất bại, vì vậy tiến trình dừng và không tạo control loop.

Ảnh hưởng tới điều khiển:

- Robot không khởi động controller nếu VN-100 không tồn tại hoặc không gửi dữ liệu hợp lệ.
- Đây là thay đổi fail-safe có chủ ý.
- Robot sẽ không thể chạy ở chế độ hardware nếu muốn cố tình bỏ qua IMU.

### 3.8. Theo dõi độ mới của dữ liệu IMU

Đã bổ sung API:

```cpp
bool vectornav_data_is_valid(uint64_t max_age_us);
uint64_t vectornav_last_packet_age_us();
```

Timestamp chỉ cập nhật sau khi nhận và chấp nhận một packet hợp lệ.

Tình trạng hiện tại:

- API phát hiện IMU stale đã có.
- API chưa được nối vào SPI để cưỡng chế `flags=0` khi IMU mất dữ liệu lúc robot đang chạy.
- Việc nối watchdog IMU với lệnh disable motor phải được thực hiện cùng phần SPI.

### 3.9. Tắt debug terminal tốc độ cao

Hiện tại:

```cpp
//#define PRINT_VECTORNAV_DEBUG
```

Kết quả:

- Không in `QUAT`, `OMEGA`, `ACC` khoảng 200 lần mỗi giây.
- Giảm terminal I/O, CPU load và jitter cho real-time loop.
- Dữ liệu vẫn cập nhật `_vectorNavData` và publish trên LCM channel `hw_vectornav`.

## 4. Cách state estimator sử dụng dữ liệu

VN-100 cung cấp quaternion theo thứ tự:

```text
x, y, z, w
```

`VectorNavOrientationEstimator` chuyển sang định dạng nội bộ:

```text
w, x, y, z
```

Gyro được đưa vào `omegaBody`. Acceleration được đưa vào `aBody`. Rotation matrix từ quaternion được dùng để tính `omegaWorld` và `aWorld`.

Trong lần chạy đầu, estimator loại bỏ yaw ban đầu bằng `_ori_ini_inv`; roll và pitch ban đầu không bị ép về 0. Điều này ảnh hưởng trực tiếp tới hệ tọa độ heading của robot:

- Heading lúc khởi động được coi là mốc yaw ban đầu.
- Hướng lắp IMU trên thân robot vẫn phải đúng với quy ước trục body của source.
- Nếu IMU bị xoay so với thân robot, cần có phép biến đổi mounting rotation; code hiện chưa bổ sung phép hiệu chỉnh mounting riêng.

## 5. Những ảnh hưởng trực tiếp tới điều khiển robot

### Ảnh hưởng tích cực

- Controller nhận orientation, angular velocity và acceleration thật từ VN-100.
- Controller không khởi động nếu chưa có packet IMU hợp lệ.
- Packet hỏng rõ ràng bị loại.
- Debug tốc độ cao đã tắt để giảm ảnh hưởng real-time.
- State estimator nhận đúng thứ tự quaternion.

### Thay đổi hành vi cần lưu ý

- Khởi động phụ thuộc bắt buộc vào VN-100.
- Đường dẫn thiết bị sai sẽ làm toàn bộ chương trình hardware dừng.
- Cáp/driver không chạy được ở 921600 sẽ làm khởi tạo thất bại.
- Heading của state estimator được đặt tương đối theo yaw lúc khởi động.
- Cấu hình hiện dành cho VectorNav, không còn dùng Microstrain.

### Rủi ro còn lại

1. Watchdog IMU lúc runtime chưa cưỡng chế disable motor qua SPI.
2. Chưa xác nhận hướng lắp và dấu các trục IMU so với body frame của robot.
3. Chưa đánh giá rung cơ khí và độ cứng của gá IMU.
4. Chưa kiểm tra mất USB trong khi controller đang chạy.
5. Việc cập nhật struct IMU giữa callback serial và control loop vẫn theo kiến trúc gốc, chưa bổ sung snapshot/mutex riêng.
6. LCM Spy chưa hoạt động trên máy nếu Java và Java LCM types chưa được cài/generate; điều này không ngăn state estimator sử dụng IMU.
7. Phần SPI, ESTOP và motor-enable chưa đạt điều kiện an toàn để cấp nguồn động cơ.

## 6. Điều kiện xác nhận phần IMU

Lệnh chạy khuyến nghị:

```bash
cd /home/hp3/robot_project/Cheetah-Software-MIT
VN100_PORT="$(find /dev/serial/by-id -maxdepth 1 -type l -name 'usb-FTDI*' -print -quit)"

sudo env CHEETAH_VECTORNAV_PORT="$VN100_PORT" \
stdbuf -oL -eL \
./jetson-build-aarch64/user/MiniCheetahSpi_Controller/mcspi_ctrl m r f 2>&1 \
| grep --line-buffered -E "MiniCheetahHardware|rt_vectornav|Model Number|baud rate|IMU|VECTORNAV|FAILED|Warning|Binary"
```

Điều kiện đạt:

```text
Model Number: VN-100S-CR
[rt_vectornav] IMU is set up and streaming valid data!
```

Không được có:

```text
FAILED TO INITIALIZE HARDWARE
```

## 7. Kiểm tra cần làm sau khi lắp IMU lên thân robot

Thực hiện khi chưa cấp nguồn động cơ:

1. Đặt robot trên mặt phẳng và kiểm tra roll/pitch gần với tư thế thực tế.
2. Nâng từng phía thân robot và xác nhận dấu roll/pitch đúng.
3. Quay robot quanh trục đứng và xác nhận yaw thay đổi đúng chiều.
4. Giữ robot đứng yên và xác nhận gyro gần 0.
5. Xác nhận norm acceleration gần 9.81 m/s² khi đứng yên.
6. Chạy liên tục ít nhất 15 phút và theo dõi mất packet hoặc reset USB.
7. Cố tình rút USB trước khi khởi động và xác nhận chương trình từ chối khởi động.
8. Chưa thử rút USB khi motor có nguồn cho tới khi watchdog SPI đã hoàn thiện.

## 8. Trạng thái build

Sau các thay đổi IMU, hai target đã build thành công cho ARM64:

```text
[100%] Built target robot
[100%] Built target mcspi_ctrl
```

Build thành công chỉ xác nhận tính hợp lệ khi biên dịch. Xác nhận phần cứng cuối cùng vẫn dựa vào dòng `IMU is set up and streaming valid data!` và các thử nghiệm hướng lắp ở mục 7.

## 9. Kết luận

Phần VN-100 đã được nối đúng vào state estimator, có kiểm tra packet và fail-safe lúc khởi động. Debug terminal đã tắt. Phần IMU đủ điều kiện để lắp lên robot và kiểm tra không tải.

Hệ thống tổng thể chưa đủ điều kiện cấp nguồn động cơ cho tới khi hoàn thiện SPI, checksum/CRC, watchdog STM32, watchdog IMU-to-motor-disable, RC timeout và ESTOP đặt `flags=0`.

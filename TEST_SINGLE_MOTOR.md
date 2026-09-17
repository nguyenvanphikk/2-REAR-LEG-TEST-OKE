# Test một động cơ Mini Cheetah qua CAN

Firmware đang bật chế độ test bằng:

```cpp
#define SINGLE_MOTOR_CAN_TEST 1
```

Mặc định dùng cặp CAN PB8/PB9 (`b 2` trong giao diện test), motor ID 1 và
tốc độ CAN 1 Mbit/s. Tên `b 2` chỉ là tên logic cũ trong chương trình; trên
STM32F446 đây là ngoại vi CAN1.

## Đấu dây

- Theo nhãn module của bạn: STM32 `PB9` (`CAN_TX`) -> chân `RX` của module.
- Theo nhãn module của bạn: STM32 `PB8` (`CAN_RX`) <- chân `TX` của module.
- CAN transceiver `CANH/CANL` -> `CANH/CANL` của driver động cơ.
- Nối chung GND của STM32, transceiver và driver.
- Dùng termination 120 ohm phù hợp ở hai đầu bus.
- `PB15` là ESTOP active-low; để chạy thử phải ở mức high.

Không nối trực tiếp PB8/PB9 vào CANH/CANL. STM32 cần CAN transceiver.

Theo pin-map phần cứng STM32F446RE, PB8 vẫn là CAN_RX và PB9 vẫn là CAN_TX.
Trong code phải giữ `CAN(PB_8, PB_9, 1000000)` vì constructor dùng thứ tự
`(RX của STM32, TX của STM32)`. Module của bạn đặt nhãn theo phía module nên
dây được đấu chéo như hai dòng phía trên.

## Keil 5

Mở `SPIne.uvprojx`, chọn target `SPIne`, Build rồi nạp vào STM32F446RE.
Nếu dùng board NUCLEO-F446RE, cắm cổng micro-USB ST-LINK vào máy tính. Cổng này
vừa dùng để nạp code vừa tạo ST-LINK Virtual COM Port nối sẵn với USART2:
`PA2` (STM32 TX) và `PA3` (STM32 RX). Không cần USB-UART rời. Mở đúng COM port
trên máy tính với cấu hình `115200 8N1`, không flow control.

## Lệnh UART

Mỗi lệnh phải kết thúc bằng Enter.

```text
?          hiện trợ giúp
b 1        chọn cặp PB12 RX / PB13 TX
b 2        chọn PB8/PB9 (mặc định; PB8->TX module, PB9->RX module)
i 1        chọn motor CAN ID 1 (có thể chọn 1..3)
d 1.0      đặt KD = 1.0
v 0.2      đặt vận tốc 0.2 rad/s
m          vào motor mode và bắt đầu gửi lệnh
v -0.2     đổi chiều
v 0        dừng có điều khiển
x          dừng và thoát motor mode
z          đặt zero; chỉ cho phép khi đã x và v bằng 0
```

Bắt đầu với motor không tải, cố định chắc chắn, nguồn giới hạn dòng, `v 0.2`
và `d 0.5`. Chuẩn bị ngắt nguồn phần công suất nếu động cơ phản ứng sai.

Dòng `STAT` được in mỗi 200 ms. `rx` phải tăng nếu STM32 nhận được phản hồi
từ động cơ. `tx` là số frame gửi thành công; `fail` là số lần CAN mailbox gửi
thất bại. Nếu `tx` tăng nhưng `rx` luôn bằng 0, kiểm tra ID, bitrate, transceiver,
termination và dây CANH/CANL.

Để quay lại firmware điều khiển robot qua SPI, đổi cờ thành:

```cpp
#define SINGLE_MOTOR_CAN_TEST 0
```

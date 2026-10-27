Copy right by Huân Quang Trần
🛠️ Thông Số Thiết Bị & Linh Kiện Cần Thiết
Linh kiện	Số lượng	Ghi chú / Thông số
Bo mạch điều khiển ESP32	1	ESP32 NodeMCU (30-pin hoặc 38-pin)
Cảm biến vân tay Quang học	1	ZW101 / FPM10 / R307 (Chuẩn giao tiếp UART)
Module RFID RC522	1	Tần số 13.56 MHz (Chuẩn SPI)
Module Rơ-le (Relay Module)	2	Module Relay 5V/12V (Active-HIGH) dùng cho Nguồn ổ khóa & Nút đề
Cảm biến công tắc Chân chống	1	Công tắc hành trình / Cảm biến từ
Còi báo (Buzzer Active)	1	Còi 5V báo tín hiệu
Điện trở phân áp ADC	2	100k\Omega và 27k\Omega (Dùng đo áp bình 12V)
Nguồn hạ áp DC-DC (Buck)	1	LM2596 / MP1584 (Hạ áp từ 12V bình xuống 5V cấp cho ESP32)
🔌 Sơ Đồ Đấu Nối Chân (Wiring Diagram)
Dưới đây là sơ đồ kết nối chuẩn xác theo các định nghĩa chân trong mã nguồn ⁠ok_ble.ino⁠:
1. Cảm biến vân tay (UART2) & Nút Touch Wake
Chân cảm biến Vân tay	Chân kết nối ESP32	Ghi chú
TX	GPIO 32	RX2 của ESP32
RX	GPIO 33	TX2 của ESP32
WAKE / TOUCH	GPIO 13	Tín hiệu chạm kích hoạt ngắt (FP_WAKE_PIN)
VCC	5V	Nguồn cấp 5V
GND	GND	Mass chung
2. Module Thẻ từ RFID RC522 (SPI)
Chân RC522	Chân kết nối ESP32	Ghi chú
SDA / SS	GPIO 5	Chip Select (RFID_SS)
RST	GPIO 4	Reset (RFID_RST)
SCK	GPIO 18	SPI Clock (SPI_SCK)
MISO	GPIO 19	SPI Master In Slave Out (SPI_MISO)
MOSI	GPIO 23	SPI Master Out Slave In (SPI_MOSI)
3.3V	3.3V	Lưu ý: Chỉ cấp nguồn 3.3V cho RC522
GND	GND	Mass chung
3. Module Rơ-le, Công tắc Chân chống & Thiết bị ngoại vi
Thiết bị	Chân kết nối ESP32	Trạng thái / Ghi chú
Rơ-le Khóa Điện (RELAY_POWER)	GPIO 26	HIGH = Bật nguồn xe (RELAY_ON)
Rơ-le Đề (RELAY_STARTER)	GPIO 25	HIGH = Kích đề nổ máy (RELAY_ON)
Cảm biến Chân chống (SIDESTAND)	GPIO 27	LOW = Đang hạ chân chống
Mạch đo Áp Bình (BATTERY ADC)	GPIO 34	Chân ADC1 (Chỉ đọc Input)
Còi báo (BUZZER)	GPIO 12	Còi báo hiệu âm thanh
Đèn LED Onboard	GPIO 2	Đèn báo trạng thái bo mạch
📐 Mạch Đo Điện Áp Bình Ắc-quy (Battery Monitor Circuit)
Để bảo vệ chân ⁠GPIO 34⁠ (chịu tối đa 

), cần dùng mạch cầu phân áp điện trở:
💻 Hướng Dẫn Cài Đặt & Nạp Code
1. Yêu cầu môi trường

 Tải và cài đặt Arduino IDE (phiên bản 1.8.x hoặc 2.x).

 Cài đặt cấu hình bo mạch ESP32 trong Arduino IDE:

 URL Manager: ⁠https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json⁠
2. Thư viện cần thiết
Vào Tools 

 Manage Libraries... và cài đặt các thư viện sau:
1. Adafruit Fingerprint Sensor Library
2. MFRC522 (bởi GitHubCommunity)
3. Các thư viện mặc định của ESP32: ⁠Preferences.h⁠, ⁠BLEDevice.h⁠, ⁠BLEServer.h⁠, ⁠BLEUtils.h⁠, ⁠SPI.h⁠.
3. Cấu hình Arduino IDE trước khi nạp

 Board: ⁠ESP32 Dev Module⁠

 Partition Scheme: ⁠Huge APP (3MB No OTA/1MB SPIFFS)⁠ (Bắt buộc chọn cấu hình này để đủ dung lượng chứa bộ thư viện BLE).

 Upload Speed: ⁠921600⁠
📲 Giao Thức Kết Nối & Lệnh Điều Khiển qua BLE
Mạch phát tín hiệu Bluetooth với tên mặc định: ⁠XE_SMART_LOCK⁠.
Thông số UUID Services & Characteristics:

 SERVICE_UUID: ⁠4fa8691a-1056-5a69-2fbd-b15e00800000⁠

 CHARACTERISTIC_UUID_RX (Gửi lệnh): ⁠beb5483e-36e1-4688-b7f5-ea07361b26a8⁠

 CHARACTERISTIC_UUID_TX (Nhận phản hồi): ⁠1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e⁠
Danh sách lệnh dạng chuỗi (String Commands):
Chuỗi lệnh (Command)	Chức năng
UNLOCK	Mở khóa điện của xe
LOCK	Tắt nguồn xe / Khóa xe
START	Bật rơ-le đề nổ trong khoảng thời gian định sẵn
ENROLL	Bắt đầu quy trình thêm vân tay mới qua BLE
DELETE_ALL_FP	Xóa toàn bộ dữ liệu vân tay
ADD_CARD	Bật chế độ quét thêm thẻ RFID mới
DELETE_ALL_CARD	Xóa toàn bộ thẻ RFID trong bộ nhớ
STATUS	Truy vấn trạng thái điện áp bình, chân chống và trạng thái khóa
🖐️ Hướng Dẫn Thao Tác Vân Tay

 Chạm nhanh ngón tay (Nhấp 1 lần):

 Xe đang khóa 

 Mở khóa xe (Rơ-le nguồn bật, LED báo sáng).

 Xe đang mở 

 Tắt nguồn/Khóa xe.

 Giữ ngón tay liên tục (khi xe đã mở khóa):

 Giữ ngón tay trên cảm biến 

 Kích hoạt Rơ-le đề nổ máy.

 Nhả tay ra 

 Ngắt Rơ-le đề ngay lập tức.

 Cảnh báo an toàn: Nếu gạt chân chống nghiêng xuống (⁠SIDESTAND_PIN = LOW⁠), thao tác giữ đề bằng vân tay sẽ bị chặn để đảm bảo an toàn.
⚠️ Lưu Ý An Toàn Khi Lắp Đặt Trên Xe Máy
1. Cách ly nguồn điện (Nguồn Buck DC-DC):

 Không lấy nguồn trực tiếp từ bình ắc-quy cấp thẳng cho ESP32. Bắt buộc dùng mạch hạ áp DC-DC hạ từ 

 xuống 

 ổn định.
2. Cầu chì bảo vệ:

 Lắp thêm cầu chì 

 trên đường dây nguồn tổng trước khi đi qua Rơ-le điện nguồn của xe.
3. Chống nước & Chống rung:

 Đổ keo nến hoặc keo Silicone bảo vệ bo mạch ESP32 và các chân nối dây cảm biến vân tay ngoài xe.
4. Kiểm tra dây Mass (GND):

 Đảm bảo tất cả các thiết bị (ESP32, Module Relay, Cảm biến, Mạch hạ áp) được đấu chung một đường GND chắc chắn.
📄 Licenese
Dự án được phát triển dưới dạng mã nguồn mở DIY (MIT License). Bạn có thể tự do chỉnh sửa và phát triển thêm tính năng!
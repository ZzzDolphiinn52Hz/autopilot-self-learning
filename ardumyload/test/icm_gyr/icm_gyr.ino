#include <SPI.h>
#include <Wire.h>
#include <ICM42688.h>
#include <QMC5883LCompass.h>

// Định nghĩa chân Chip Select (CS) cho SPI của ICM42688P
const int csPin = 5;
ICM42688 IMU(SPI, csPin);

// Khởi tạo đối tượng la bàn GY-271
QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n--- KHỞI KHỞI ĐỘNG HỆ THỐNG CẢM BIẾN HỢP NHẤT ---");

  // 1. Khởi động ICM42688P qua giao tiếp SPI
  SPI.begin(); // Mặc định: SCK=18, MISO=19, MOSI=23
  int imuStatus = IMU.begin();
  if (imuStatus < 0) {
    Serial.println("❌ LỖI: Không kết nối được ICM42688P qua SPI!");
    while (1);
  }
  Serial.println("✅ ICM42688P (SPI) kết nối thành công.");

  // 2. Khởi động GY-271 qua giao tiếp I2C
  Wire.begin(21, 22); // Chân I2C mặc định trên ESP32: SDA=21, SCL=22
  compass.init();
  
  // ÁP DỤNG THÔNG SỐ HIỆU CHUẨN THỰC TẾ CỦA BẠN ĐỂ KHỬ NHIỄU SẮT CỨNG
  compass.setCalibration(-2302, -212, -3128, -193, 1181, 3931);
  
  Serial.println("✅ GY-271 (I2C) đã được nạp dữ liệu cấu hình Calibrate.");
  Serial.println("--------------------------------------------------------\n");
  delay(1000);
}

void loop() {
  // --- Đọc dữ liệu từ IMU ICM42688P ---
  IMU.getAGT();
  float accX = IMU.accX();
  float accY = IMU.accY();
  float accZ = IMU.accZ();
  float gyrZ = IMU.gyrZ(); // Trục Z của Gyro (rất quan trọng để kết hợp bù Yaw)

  // --- Đọc dữ liệu từ La bàn GY-271 ---
  compass.read();
  
  // Hàm getAzimuth() của thư viện này sẽ tự động tính toán góc hướng Bắc (Heading/Yaw)
  // từ các giá trị X, Y đã được trừ đi sai số cấu hình ở setup()
  int heading = compass.getAzimuth(); 
  
  // Thư viện trả về từ -180 đến 180 hoặc 0 đến 360 tùy phiên bản, 
  // ta chuẩn hóa về dạng góc la bàn chuẩn (0 độ = Hướng Bắc, tăng theo chiều kim đồng hồ)
  if (heading < 0) {
    heading += 360;
  }

  // --- IN DỮ LIỆU RA SERIAL MONITOR ---
  
  // Dữ liệu IMU (Gia tốc & Tốc độ góc trục Z)
  Serial.print("IMU -> AccZ: "); Serial.print(accZ, 3);
  Serial.print(" g | GyrZ: "); Serial.print(gyrZ, 2);
  Serial.print(" deg/s");

  // Dữ liệu góc Yaw/Heading thực tế từ la bàn sau khi sửa lỗi
  Serial.print("  ||  LA BÀN -> YAW (HEADING): ");
  Serial.print(heading);
  Serial.println("°");

  delay(50); // Đọc ở tần số 20Hz (mỗi 50ms) để dữ liệu cập nhật mượt mà
}
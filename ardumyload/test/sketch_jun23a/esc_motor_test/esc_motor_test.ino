#include <ESP32Servo.h>

Servo myESC;

// Định nghĩa các chân kết nối
const int potPin = 34;  // Chân đọc biến trở (ADC1_CH6)
const int escPin = 18;  // Chân xuất xung điều khiển ESC

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo các timer cho thư viện ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  myESC.setPeriodHertz(50);           // ESC tiêu chuẩn chạy ở tần số 50Hz
  myESC.attach(escPin, 1000, 2000);   // Độ rộng xung tiêu chuẩn: 1000us (Min) đến 2000us (Max)
  
  Serial.println("--- HỆ THỐNG ĐÃ SẴN SÀNG ---");
  Serial.println("Lưu ý: Vặn biến trở về CỰC TIỂU (0) để ESC kích hoạt (Arming).");
}

void loop() {
  // ESP32 có độ phân giải ADC là 12-bit (giá trị từ 0 đến 4095)
  int potValue = analogRead(potPin); 
  
  // Chuyển đổi giá trị biến trở (0-4095) sang độ rộng xung (1000-2000 micro giây)
  int throttle = map(potValue, 0, 4095, 1000, 2000);
  
  // Xuất xung điều khiển đến ESC
  myESC.writeMicroseconds(throttle); 
  
  // In giá trị lên Serial Monitor để theo dõi
  Serial.print("Gia tri ADC: ");
  Serial.print(potValue);
  Serial.print(" | Xung ra ESC (us): ");
  Serial.println(throttle);
  
  delay(20); // Chờ 20ms cho mỗi chu kỳ xung ổn định
}
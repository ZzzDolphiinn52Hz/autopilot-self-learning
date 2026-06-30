#include <HardwareSerial.h>

HardwareSerial GPS(1);

#define GPS_RX 16   // ESP32 RX, nối GPS TX
#define GPS_TX 17   // ESP32 TX, nối GPS RX nếu cần

uint32_t baudList[] = {
  9600, 19200, 38400, 57600, 115200, 230400
};

bool isLikelyNMEA(String line) {
  if (!line.startsWith("$")) return false;

  if (
    line.startsWith("$GP") ||
    line.startsWith("$GN") ||
    line.startsWith("$GL") ||
    line.startsWith("$GA") ||
    line.startsWith("$GB") ||
    line.startsWith("$BD")
  ) {
    return true;
  }

  return false;
}

void testBaud(uint32_t baud) {
  GPS.end();
  delay(300);

  GPS.begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);

  Serial.println();
  Serial.print("===== Thu baud: ");
  Serial.print(baud);
  Serial.println(" =====");

  unsigned long start = millis();
  String line = "";
  int validLines = 0;
  int totalBytes = 0;

  while (millis() - start < 5000) {
    while (GPS.available()) {
      char c = GPS.read();
      totalBytes++;

      if (c == '$') {
        line = "$";
      }
      else if (line.length() > 0) {
        if (c == '\n' || c == '\r') {
          if (line.length() > 6) {
            if (isLikelyNMEA(line)) {
              validLines++;
              Serial.print("NMEA: ");
              Serial.println(line);
            }
          }
          line = "";
        }
        else {
          if ((c >= 32 && c <= 126) && line.length() < 120) {
            line += c;
          } else {
            line = "";
          }
        }
      }
    }
  }

  Serial.print("Tong byte: ");
  Serial.println(totalBytes);

  Serial.print("So dong NMEA hop le: ");
  Serial.println(validLines);

  if (validLines > 0) {
    Serial.print("=> BAUD CO KHA NANG DUNG: ");
    Serial.println(baud);
  } else {
    Serial.println("=> Khong thay dong NMEA hop le");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("--- BAT DAU TIM NMEA THAT ---");

  for (int i = 0; i < sizeof(baudList) / sizeof(baudList[0]); i++) {
    testBaud(baudList[i]);
  }

  Serial.println("--- XONG ---");
}

void loop() {
}
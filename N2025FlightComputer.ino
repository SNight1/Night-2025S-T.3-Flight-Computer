#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define BMP280_ADDRESS 0x76  // สลับเป็น 0x77 หากไม่เจอ
#define MPU6050_ADDRESS 0x68
#define VOLTAGE_PIN 34       // ขา ADC วัดไฟ 5V ผ่าน Divider (10k/20k)
#define SDA_PIN 21
#define SCL_PIN 22

//NEO-6M GPS (UART2)
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

//  UBX Hex  GPS NEO-6M เป็น 5Hz
const byte UBLOX_INIT[] PROGMEM = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
};

enum FlightState {
  STATE_GROUND_IDLE,    // Calibrate แล้ว ขึ้น READY)
  STATE_POWERED_ASCENT, // เครื่องยนต์ทำงาน / ไต่ระดับขึ้น
  STATE_APOGEE,         // แตะจุดสูงสุด (เปิดร่มชูชีพ)
  STATE_DESCENT,        // กำลังร่อนลงพื้น
  STATE_LANDED          // ลงถึงพื้นจบภารกิจ
};

FlightState currentState = STATE_GROUND_IDLE;

uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t  t_fine;

float groundAltitude = 0.0;
float maxAltitude = 0.0;
float lastAltitude = 0.0;
unsigned long lastAltTime = 0;
float verticalVelocity = 0.0;

int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;
float accX, accY, accZ;
float pitch = 0.0, roll = 0.0, yaw = 0.0;

unsigned long launchStartTime = 0; // เวลาเริ่มต้นปล่อยตัว (T+0)
unsigned long previousMillis = 0;
const long interval = 50;           // ลูปเก็บข้อมูล 20 Hz

//BMP280 DIRECT REGISTER
uint16_t read16_LE(byte reg) {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP280_ADDRESS, (uint8_t)2);
  return (Wire.read() | (Wire.read() << 8));
}

int16_t readS16_LE(byte reg) { return (int16_t)read16_LE(reg); }

void readCalibrationData() {
  dig_T1 = read16_LE(0x88); dig_T2 = readS16_LE(0x8A); dig_T3 = readS16_LE(0x8C);
  dig_P1 = read16_LE(0x8E); dig_P2 = readS16_LE(0x90); dig_P3 = readS16_LE(0x92);
  dig_P4 = readS16_LE(0x94); dig_P5 = readS16_LE(0x96); dig_P6 = readS16_LE(0x98);
  dig_P7 = readS16_LE(0x9A); dig_P8 = readS16_LE(0x9C); dig_P9 = readS16_LE(0x9E);
}

float readTemperature() {
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xFA);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP280_ADDRESS, (uint8_t)3);

  int32_t adc_T = ((int32_t)Wire.read() << 12) | ((int32_t)Wire.read() << 4) | ((int32_t)Wire.read() >> 4);
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  return ((t_fine * 5 + 128) >> 8) / 100.0;
}

float readPressure() {
  readTemperature();
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP280_ADDRESS, (uint8_t)3);

  int32_t adc_P = ((int32_t)Wire.read() << 12) | ((int32_t)Wire.read() << 4) | ((int32_t)Wire.read() >> 4);
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 == 0) return 0;

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

  return (float)p / 25600.0;
}

float calculateRawAltitude(float press) {
  return 44330.0 * (1.0 - pow(press / 1013.25, 0.1903));
}

//MPU6050 & VOLTAGE
void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readMPU6050Data() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) {
    initMPU6050(); // Auto-recovery หากบัสหลุด
    return;
  }

  if (Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14) == 14) {
    rawAccX = (Wire.read() << 8) | Wire.read();
    rawAccY = (Wire.read() << 8) | Wire.read();
    rawAccZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    rawGyroX = (Wire.read() << 8) | Wire.read();
    rawGyroY = (Wire.read() << 8) | Wire.read();
    rawGyroZ = (Wire.read() << 8) | Wire.read();

    if (rawAccX == 0 && rawAccY == 0 && rawAccZ == 0) return;

    accX = rawAccX / 16384.0;
    accY = rawAccY / 16384.0;
    accZ = rawAccZ / 16384.0;

    pitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / M_PI;
    roll  = atan2(-accX, accZ) * 180.0 / M_PI;
    yaw  += (rawGyroZ / 131.0) * (interval / 1000.0);
  }
}

float readSupplyVoltage() {
  uint32_t rawSum = 0;
  for (int i = 0; i < 10; i++) {
    rawSum += analogRead(VOLTAGE_PIN);
  }
  float pinVoltage = (rawSum / 10.0 / 4095.0) * 3.3;
  return pinVoltage * 1.5; // Divider Ratio (10k/20k)
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);

  delay(500);

  Serial.println(F("\n=================================================="));
  Serial.println(F("   FLIGHT COMPUTER ALREADY    "));
  Serial.println(F("=================================================="));

  // Init BMP280
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF4); Wire.write(0x27);
  Wire.endTransmission();
  readCalibrationData();

  // Init MPU6050 & GPS
  initMPU6050();
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(500);
  gpsSerial.write(UBLOX_INIT, sizeof(UBLOX_INIT));

  // Ground Zero Calibration
  Serial.println(F("[INFO] Calibrating Ground Zero Altitude... Please wait..."));
  float sumAlt = 0;
  for (int i = 0; i < 50; i++) {
    sumAlt += calculateRawAltitude(readPressure());
    delay(20);
  }
  groundAltitude = sumAlt / 50.0;
  lastAltTime = millis();

  Serial.print(F("Ground Altitude Set: ")); Serial.print(groundAltitude, 2); Serial.println(F(" m"));
  Serial.println(F("=================================================="));
  Serial.println(F(">>> ROCKET IS READY TO LAUNCH! <<<"));
  Serial.println(F(">>> Type 'S' or 's' in Serial Monitor to LAUNCH! <<<"));
  Serial.println(F("=================================================="));
}

// ==================== MAIN LOOP ====================
void loop() {
  // อ่านค่า GPS Background
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // --- MANUAL LAUNCH TRIGGER CHECK ---
  if (currentState == STATE_GROUND_IDLE) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 's' || cmd == 'S') {
        currentState = STATE_POWERED_ASCENT;
        launchStartTime = millis();
        Serial.println(F("FlightTime(ms)\tState\t\tAlt(m)\tVelo(m/s)\tPitch\tRoll\tAccZ(g)\tV5(V)\tSats\tLat\t\tLng"));
      }
    }
  }

  // --- NON-BLOCKING TELEMETRY LOOP (20Hz) ---
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    float dt = (currentMillis - lastAltTime) / 1000.0;
    previousMillis = currentMillis;

    // อ่านค่าเซนเซอร์ทั้งหมด
    float press = readPressure();
    float temp = readTemperature();
    float currentAlt = calculateRawAltitude(press) - groundAltitude;
    if (dt > 0) verticalVelocity = (currentAlt - lastAltitude) / dt;
    lastAltitude = currentAlt;
    lastAltTime = currentMillis;

    readMPU6050Data();
    float v5 = readSupplyVoltage();

    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lng = gps.location.isValid() ? gps.location.lng() : 0.0;

    // คำนวณเวลาการบินนับตั้งแต่กด Launch (Flight Time in ms)
    unsigned long flightTimeMs = (currentState == STATE_GROUND_IDLE) ? 0 : (currentMillis - launchStartTime);

    //FLIGHT STATE MACHINE LOGIC
    String stateStr = "IDLE";
    switch (currentState) {
      case STATE_GROUND_IDLE:
        stateStr = "READY";
        break;

      case STATE_POWERED_ASCENT:
        stateStr = "ASCENT";
        if (currentAlt > maxAltitude) maxAltitude = currentAlt;
        // หากความเร็วทิศทางขึ้นดรอปติดลบ หรือเริ่มตก ให้เปลี่ยนเป็น Apogee
        if (currentAlt > 5.0 && verticalVelocity < -1.0) {
          currentState = STATE_APOGEE;
        }
        break;

      case STATE_APOGEE:
        stateStr = "APOGEE";
        // จำลองสั่งเปิดร่มชูชีพ (Deployment Logic)
        currentState = STATE_DESCENT;
        break;

      case STATE_DESCENT:
        stateStr = "DESCENT";
        if (currentAlt <= 1.0 && flightTimeMs > 5000) {
          currentState = STATE_LANDED;
        }
        break;

      case STATE_LANDED:
        stateStr = "LANDED";
        break;
    }

    //TELEMETRY STREAM OUTPUT
    if (currentState != STATE_GROUND_IDLE) {
      Serial.print(flightTimeMs); Serial.print(F("\t\t"));
      Serial.print(stateStr); Serial.print(F("\t"));
      Serial.print(currentAlt, 2); Serial.print(F("\t"));
      Serial.print(verticalVelocity, 2); Serial.print(F("\t\t"));
      Serial.print(pitch, 1); Serial.print(F("\t"));
      Serial.print(roll, 1); Serial.print(F("\t"));
      Serial.print(accZ, 2); Serial.print(F("\t"));
      Serial.print(v5, 2); Serial.print(F("\t"));
      Serial.print(sats); Serial.print(F("\t"));
      Serial.print(lat, 5); Serial.print(F("\t"));
      Serial.println(lng, 5);
    }
  }
}
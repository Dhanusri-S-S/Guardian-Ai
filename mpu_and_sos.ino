#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "driver/i2s.h"

// =====================================================
// PIN DEFINITIONS
// =====================================================

// SOS Button
#define SOS_BUTTON 26

// Vibration Motor
#define VIBRATION_PIN 13

// MPU6050 - I2C Bus 0
#define MPU_SDA 18
#define MPU_SCL 19

// MAX30102 - I2C Bus 1
#define MAX_SDA 21
#define MAX_SCL 22

// GPS
#define GPS_RX 16
#define GPS_TX 17

// INMP441
#define I2S_PORT I2S_NUM_0
#define I2S_SCK 32
#define I2S_WS  25
#define I2S_SD  33

// =====================================================
// BLE
// =====================================================

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789001"
#define CHARACTERISTIC_UUID "12345678-1234-1234-1234-123456789002"

BLECharacteristic *pCharacteristic;

bool deviceConnected = false;

// =====================================================
// MPU6050
// =====================================================

#define MPU6050_ADDR 0x68

float accelX;
float accelY;
float accelZ;

float accelMagnitude;

// Change this later after collecting real movement data.
const float MOTION_THRESHOLD = 1.80;

bool abnormalMovement = false;

// =====================================================
// MAX30102
// =====================================================

MAX30105 heartSensor;

const byte RATE_SIZE = 4;

byte rates[RATE_SIZE];
byte rateSpot = 0;

long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

bool heartSensorReady = false;

// =====================================================
// GPS
// =====================================================

TinyGPSPlus gps;

HardwareSerial GPS(2);

double latitude = 0;
double longitude = 0;

int satellites = 0;

bool gpsFix = false;

// =====================================================
// INMP441
// =====================================================

bool microphoneReady = false;

int soundLevel = 0;

// =====================================================
// SOS
// =====================================================

bool lastButtonState = HIGH;

// =====================================================
// BLE CALLBACKS
// =====================================================

class MyServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer *pServer) {
    deviceConnected = true;

    Serial.println();
    Serial.println("================================");
    Serial.println("PHONE CONNECTED THROUGH BLE");
    Serial.println("================================");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;

    Serial.println();
    Serial.println("PHONE DISCONNECTED");
    Serial.println("Restarting BLE advertising...");

    BLEDevice::startAdvertising();
  }
};

// =====================================================
// MPU6050 FUNCTIONS
// =====================================================

void writeMPURegister(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool initializeMPU6050() {

  Wire.begin(MPU_SDA, MPU_SCL);

  Wire.beginTransmission(MPU6050_ADDR);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Wake MPU6050
  writeMPURegister(0x6B, 0x00);

  // Accelerometer ±2g
  writeMPURegister(0x1C, 0x00);

  return true;
}

void readMPU6050() {

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 6);

  if (Wire.available() >= 6) {

    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();

    accelX = rawX / 16384.0;
    accelY = rawY / 16384.0;
    accelZ = rawZ / 16384.0;

    accelMagnitude =
      sqrt(
        accelX * accelX +
        accelY * accelY +
        accelZ * accelZ
      );

    // Basic activity indicator.
    // This is NOT an AI assault detector.
    abnormalMovement =
      abs(accelMagnitude - 1.0) > MOTION_THRESHOLD;
  }
}

// =====================================================
// MAX30102 FUNCTIONS
// =====================================================

bool initializeMAX30102() {

  Wire1.begin(MAX_SDA, MAX_SCL);

  if (!heartSensor.begin(Wire1, I2C_SPEED_STANDARD)) {

    Serial.println("MAX30102 initialization FAILED!");

    return false;
  }

  heartSensor.setup();

  heartSensor.setPulseAmplitudeRed(0x0A);
  heartSensor.setPulseAmplitudeIR(0x0A);

  return true;
}

void readHeartRate() {

  if (!heartSensorReady) {
    return;
  }

  long irValue = heartSensor.getIR();

  if (checkForBeat(irValue)) {

    long delta = millis() - lastBeat;

    lastBeat = millis();

    beatsPerMinute =
      60.0 / (delta / 1000.0);

    if (
      beatsPerMinute > 20 &&
      beatsPerMinute < 255
    ) {

      rates[rateSpot++] =
        (byte)beatsPerMinute;

      rateSpot %= RATE_SIZE;

      beatAvg = 0;

      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }

      beatAvg /= RATE_SIZE;
    }
  }
}

// =====================================================
// GPS FUNCTIONS
// =====================================================

void readGPS() {

  while (GPS.available() > 0) {

    gps.encode(GPS.read());
  }

  if (gps.location.isUpdated()) {

    latitude = gps.location.lat();
    longitude = gps.location.lng();

    satellites = gps.satellites.value();

    gpsFix = true;
  }
}

// =====================================================
// MICROPHONE FUNCTIONS
// =====================================================

bool initializeMicrophone() {

  i2s_config_t i2s_config = {

    .mode =
      (i2s_mode_t)(
        I2S_MODE_MASTER |
        I2S_MODE_RX
      ),

    .sample_rate = 16000,

    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_32BIT,

    .channel_format =
      I2S_CHANNEL_FMT_ONLY_LEFT,

    .communication_format =
      I2S_COMM_FORMAT_I2S,

    .intr_alloc_flags =
      ESP_INTR_FLAG_LEVEL1,

    .dma_buf_count = 8,

    .dma_buf_len = 64,

    .use_apll = false,

    .tx_desc_auto_clear = false,

    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {

    .bck_io_num = I2S_SCK,

    .ws_io_num = I2S_WS,

    .data_out_num =
      I2S_PIN_NO_CHANGE,

    .data_in_num = I2S_SD
  };

  esp_err_t result =
    i2s_driver_install(
      I2S_PORT,
      &i2s_config,
      0,
      NULL
    );

  if (result != ESP_OK) {
    return false;
  }

  result =
    i2s_set_pin(
      I2S_PORT,
      &pin_config
    );

  if (result != ESP_OK) {
    return false;
  }

  return true;
}

void readSoundLevel() {

  if (!microphoneReady) {
    return;
  }

  int32_t samples[64];

  size_t bytesRead;

  i2s_read(
    I2S_PORT,
    (void *)samples,
    sizeof(samples),
    &bytesRead,
    20
  );

  int count =
    bytesRead / sizeof(int32_t);

  if (count == 0) {
    return;
  }

  long total = 0;

  for (int i = 0; i < count; i++) {

    int32_t sample =
      samples[i] >> 14;

    total += abs(sample);
  }

  soundLevel =
    total / count;
}

// =====================================================
// BLE SEND FUNCTION
// =====================================================

void sendBLEMessage(String message) {

  if (!deviceConnected) {
    return;
  }

  pCharacteristic->setValue(
    message.c_str()
  );

  pCharacteristic->notify();

  Serial.print("BLE → PHONE: ");
  Serial.println(message);
}

// =====================================================
// SOS FUNCTION
// =====================================================

void triggerSOS() {

  Serial.println();
  Serial.println("================================");
  Serial.println("🚨 SOS BUTTON PRESSED!");
  Serial.println("================================");

  // Local vibration
  digitalWrite(
    VIBRATION_PIN,
    HIGH
  );

  // Send SOS to phone
  sendBLEMessage("SOS");

  delay(1500);

  digitalWrite(
    VIBRATION_PIN,
    LOW
  );
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("       GUARDIAN AI SAFETY BAND");
  Serial.println("       COMPLETE FIRMWARE");
  Serial.println("======================================");

  // -----------------------------------
  // GPIO
  // -----------------------------------

  pinMode(
    SOS_BUTTON,
    INPUT_PULLUP
  );

  pinMode(
    VIBRATION_PIN,
    OUTPUT
  );

  digitalWrite(
    VIBRATION_PIN,
    LOW
  );

  // -----------------------------------
  // MPU6050
  // -----------------------------------

  Serial.println();
  Serial.println("Initializing MPU6050...");

  if (initializeMPU6050()) {

    Serial.println(
      "MPU6050: READY"
    );

  } else {

    Serial.println(
      "MPU6050: NOT DETECTED"
    );
  }

  // -----------------------------------
  // MAX30102
  // -----------------------------------

  Serial.println(
    "Initializing MAX30102..."
  );

  heartSensorReady =
    initializeMAX30102();

  if (heartSensorReady) {

    Serial.println(
      "MAX30102: READY"
    );

  } else {

    Serial.println(
      "MAX30102: NOT DETECTED"
    );
  }

  // -----------------------------------
  // GPS
  // -----------------------------------

  Serial.println(
    "Initializing GPS..."
  );

  GPS.begin(
    9600,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  Serial.println(
    "GPS: READY"
  );

  // -----------------------------------
  // Microphone
  // -----------------------------------

  Serial.println(
    "Initializing INMP441..."
  );

  microphoneReady =
    initializeMicrophone();

  if (microphoneReady) {

    Serial.println(
      "INMP441: READY"
    );

  } else {

    Serial.println(
      "INMP441: FAILED"
    );
  }

  // -----------------------------------
  // BLE
  // -----------------------------------

  Serial.println(
    "Initializing BLE..."
  );

  BLEDevice::init(
    "Guardian_AI"
  );

  BLEServer *pServer =
    BLEDevice::createServer();

  pServer->setCallbacks(
    new MyServerCallbacks()
  );

  BLEService *pService =
    pServer->createService(
      SERVICE_UUID
    );

  pCharacteristic =
    pService->createCharacteristic(

      CHARACTERISTIC_UUID,

      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );

  pCharacteristic->addDescriptor(
    new BLE2902()
  );

  pCharacteristic->setValue(
    "READY"
  );

  pService->start();

  BLEAdvertising *pAdvertising =
    BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(
    SERVICE_UUID
  );

  pAdvertising->setScanResponse(true);

  BLEDevice::startAdvertising();

  Serial.println(
    "BLE: READY"
  );

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "Guardian AI Band is READY!"
  );

  Serial.println(
    "BLE Device: Guardian_AI"
  );

  Serial.println(
    "Waiting for phone connection..."
  );

  Serial.println(
    "======================================"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  // -----------------------------------
  // Read sensors
  // -----------------------------------

  readMPU6050();

  readHeartRate();

  readGPS();

  readSoundLevel();

  // -----------------------------------
  // SOS detection
  // -----------------------------------

  bool currentButtonState =
    digitalRead(SOS_BUTTON);

  if (
    lastButtonState == HIGH &&
    currentButtonState == LOW
  ) {

    triggerSOS();

    delay(300);
  }

  lastButtonState =
    currentButtonState;

  // -----------------------------------
  // Send sensor information
  // -----------------------------------

  static unsigned long lastBLEUpdate = 0;

  if (
    millis() - lastBLEUpdate >= 3000
  ) {

    lastBLEUpdate = millis();

    if (deviceConnected) {

      String movement =
        abnormalMovement
        ? "ABNORMAL"
        : "NORMAL";

      String gpsStatus =
        gpsFix
        ? "FIX"
        : "NO_FIX";

      String message =
        "DATA|MOV:" +
        movement +
        "|HR:" +
        String(beatAvg) +
        "|GPS:" +
        gpsStatus;

      sendBLEMessage(message);
    }
  }

  // -----------------------------------
  // Serial monitoring
  // -----------------------------------

  static unsigned long lastSerial =
    0;

  if (
    millis() - lastSerial >= 3000
  ) {

    lastSerial = millis();

    Serial.println();
    Serial.println("----- GUARDIAN AI STATUS -----");

    Serial.print("Movement: ");

    if (abnormalMovement) {
      Serial.println("ABNORMAL");
    } else {
      Serial.println("NORMAL");
    }

    Serial.print("Acceleration: ");
    Serial.print(accelMagnitude, 2);
    Serial.println(" g");

    Serial.print("Average BPM: ");
    Serial.println(beatAvg);

    Serial.print("GPS: ");

    if (gpsFix) {

      Serial.println("FIX");

      Serial.print("Latitude: ");
      Serial.println(latitude, 6);

      Serial.print("Longitude: ");
      Serial.println(longitude, 6);

      Serial.print("Satellites: ");
      Serial.println(satellites);

    } else {

      Serial.println("NO FIX");
    }

    Serial.print("Sound Level: ");
    Serial.println(soundLevel);

    Serial.print("BLE: ");

    if (deviceConnected) {
      Serial.println("CONNECTED");
    } else {
      Serial.println("WAITING");
    }

    Serial.println("-------------------------------");
  }

  delay(20);
} 
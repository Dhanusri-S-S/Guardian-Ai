/*
 ============================================================
                    GUARDIAN AI
          Predictive Emergency Safety Ecosystem
 ============================================================

 ESP32 CP2102

 FEATURES
 ------------------------------------------------------------
 MPU6050       -> Movement detection
 MAX30102      -> Heart rate
 NEO-6M GPS    -> Location
 INMP441       -> Audio recording
 SOS Button    -> Emergency trigger
 Vibration     -> Emergency feedback
 BLE           -> Mobile app communication

 SOS ACTION
 ------------------------------------------------------------
 When SOS button is pressed:

 1. Vibration starts
 2. BLE sends "SOS"
 3. GPS status is captured
 4. INMP441 records 5 seconds
 5. Audio is sent through USB Serial
 6. Python receiver saves WAV file

 ============================================================
 PIN CONFIGURATION
 ============================================================

 MPU6050
 SDA -> GPIO18
 SCL -> GPIO19

 MAX30102
 SDA -> GPIO21
 SCL -> GPIO22

 GPS
 TX -> GPIO16
 RX -> GPIO17

 INMP441
 SCK -> GPIO32
 WS  -> GPIO25
 SD  -> GPIO33
 L/R -> GND

 SOS BUTTON
 GPIO26 -> Button -> GND

 VIBRATION
 GPIO13 -> 1K resistor -> transistor base
 transistor emitter -> GND
 transistor collector -> motor -
 motor + -> 3.3V

 ============================================================
 BLE
 ============================================================

 Device:
 Guardian_AI

 Service:
 12345678-1234-1234-1234-123456789001

 Characteristic:
 12345678-1234-1234-1234-123456789002

 ============================================================
*/

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#include "MAX30105.h"
#include "heartRate.h"

#include "driver/i2s.h"


// ============================================================
//                       PIN DEFINITIONS
// ============================================================

// MPU6050
#define MPU_SDA 18
#define MPU_SCL 19

// MAX30102
#define MAX_SDA 21
#define MAX_SCL 22

// GPS
#define GPS_RX 16
#define GPS_TX 17

// INMP441
#define I2S_SCK 32
#define I2S_WS  25
#define I2S_SD  33

// SOS
#define SOS_BUTTON 26

// Vibration
#define VIBRATION_PIN 13


// ============================================================
//                       BLE UUID
// ============================================================

#define SERVICE_UUID \
"12345678-1234-1234-1234-123456789001"

#define CHARACTERISTIC_UUID \
"12345678-1234-1234-1234-123456789002"


// ============================================================
//                       I2C OBJECTS
// ============================================================

TwoWire MPUWire = TwoWire(0);
TwoWire MAXWire = TwoWire(1);


// ============================================================
//                       SENSOR OBJECTS
// ============================================================

MAX30105 max30102;

TinyGPSPlus gps;

HardwareSerial GPS(2);


// ============================================================
//                       BLE OBJECTS
// ============================================================

BLEServer *bleServer = nullptr;

BLECharacteristic *bleCharacteristic = nullptr;

bool deviceConnected = false;


// ============================================================
//                       MPU6050
// ============================================================

#define MPU6050_ADDR 0x68

float accelerationMagnitude = 0.0;

String movementStatus = "NORMAL";

const float MOTION_THRESHOLD = 1.80;


// ============================================================
//                       HEART RATE
// ============================================================

const byte RATE_SIZE = 4;

byte rates[RATE_SIZE];

byte rateSpot = 0;

long lastBeat = 0;

float beatsPerMinute = 0;

int averageBPM = 0;


// ============================================================
//                       SOUND
// ============================================================

int soundLevel = 0;


// ============================================================
//                       TIMERS
// ============================================================

unsigned long lastSensorSend = 0;

unsigned long lastSerialPrint = 0;

const unsigned long SENSOR_INTERVAL = 3000;

const unsigned long SERIAL_INTERVAL = 2000;


// ============================================================
//                       SOS BUTTON
// ============================================================

bool lastButtonState = HIGH;

unsigned long lastButtonTime = 0;

const unsigned long BUTTON_DEBOUNCE = 300;


// ============================================================
//                       AUDIO
// ============================================================

#define AUDIO_SAMPLE_RATE 8000

#define AUDIO_DURATION_SECONDS 5

#define AUDIO_TOTAL_SAMPLES \
(AUDIO_SAMPLE_RATE * AUDIO_DURATION_SECONDS)

#define AUDIO_BUFFER_SAMPLES 128

bool audioRecording = false;


// ============================================================
//                       BLE CALLBACKS
// ============================================================

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server)
  {
    deviceConnected = true;

    Serial.println();
    Serial.println("BLE: CONNECTED");
    Serial.println("--------------------------------");
  }


  void onDisconnect(BLEServer *server)
  {
    deviceConnected = false;

    Serial.println();
    Serial.println("BLE: DISCONNECTED");
    Serial.println("--------------------------------");

    BLEDevice::startAdvertising();

    Serial.println("BLE: ADVERTISING AGAIN");
  }
};


// ============================================================
//                       MPU REGISTER
// ============================================================

void writeMPURegister(byte reg, byte value)
{
  MPUWire.beginTransmission(MPU6050_ADDR);

  MPUWire.write(reg);

  MPUWire.write(value);

  MPUWire.endTransmission();
}


// ============================================================
//                       READ MPU
// ============================================================

bool readMPUData(
  int16_t &ax,
  int16_t &ay,
  int16_t &az
)
{
  MPUWire.beginTransmission(MPU6050_ADDR);

  MPUWire.write(0x3B);

  if (MPUWire.endTransmission(false) != 0)
  {
    return false;
  }

  MPUWire.requestFrom(MPU6050_ADDR, 6);

  if (MPUWire.available() < 6)
  {
    return false;
  }

  ax =
    (MPUWire.read() << 8) |
    MPUWire.read();

  ay =
    (MPUWire.read() << 8) |
    MPUWire.read();

  az =
    (MPUWire.read() << 8) |
    MPUWire.read();

  return true;
}


// ============================================================
//                       SETUP MPU
// ============================================================

void setupMPU6050()
{
  Serial.println("Initializing MPU6050...");

  MPUWire.begin(
    MPU_SDA,
    MPU_SCL
  );

  delay(100);

  // Wake sensor
  writeMPURegister(
    0x6B,
    0x00
  );

  // Accelerometer ±2g
  writeMPURegister(
    0x1C,
    0x00
  );

  delay(100);

  Serial.println(
    "MPU6050 initialized."
  );
}


// ============================================================
//                       READ MOVEMENT
// ============================================================

void readMovement()
{
  int16_t axRaw;

  int16_t ayRaw;

  int16_t azRaw;


  if (
    !readMPUData(
      axRaw,
      ayRaw,
      azRaw
    )
  )
  {
    return;
  }


  float ax =
    axRaw / 16384.0;

  float ay =
    ayRaw / 16384.0;

  float az =
    azRaw / 16384.0;


  accelerationMagnitude =
    sqrt(
      (ax * ax) +
      (ay * ay) +
      (az * az)
    );


  if (
    accelerationMagnitude >
    MOTION_THRESHOLD
  )
  {
    movementStatus =
      "ABNORMAL";
  }
  else
  {
    movementStatus =
      "NORMAL";
  }
}


// ============================================================
//                       MAX30102 SETUP
// ============================================================

void setupMAX30102()
{
  Serial.println(
    "Initializing MAX30102..."
  );


  MAXWire.begin(
    MAX_SDA,
    MAX_SCL
  );


  delay(100);


  if (
    !max30102.begin(
      MAXWire,
      I2C_SPEED_STANDARD
    )
  )
  {
    Serial.println(
      "MAX30102 initialization FAILED!"
    );

    return;
  }


  max30102.setup();


  max30102.setPulseAmplitudeRed(
    0x0A
  );


  max30102.setPulseAmplitudeIR(
    0x0A
  );


  Serial.println(
    "MAX30102 initialized."
  );
}


// ============================================================
//                       READ HEART RATE
// ============================================================

void readHeartRate()
{
  long irValue =
    max30102.getIR();


  if (
    checkForBeat(irValue)
  )
  {
    long delta =
      millis() - lastBeat;


    lastBeat =
      millis();


    if (delta > 0)
    {
      beatsPerMinute =
        60.0 /
        (delta / 1000.0);
    }


    if (
      beatsPerMinute > 20 &&
      beatsPerMinute < 255
    )
    {
      rates[rateSpot++] =
        (byte)beatsPerMinute;


      rateSpot %= RATE_SIZE;


      averageBPM = 0;


      for (
        byte i = 0;
        i < RATE_SIZE;
        i++
      )
      {
        averageBPM +=
          rates[i];
      }


      averageBPM /=
        RATE_SIZE;
    }
  }
}


// ============================================================
//                       GPS SETUP
// ============================================================

void setupGPS()
{
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
    "GPS initialized."
  );


  Serial.println(
    "Waiting for satellite fix..."
  );
}


// ============================================================
//                       READ GPS
// ============================================================

void readGPS()
{
  while (
    GPS.available() > 0
  )
  {
    gps.encode(
      GPS.read()
    );
  }
}


// ============================================================
//                       GPS STATUS
// ============================================================

String getGPSStatus()
{
  if (
    gps.location.isValid()
  )
  {
    return "FIX";
  }


  return "NO FIX";
}


// ============================================================
//                       GPS PRINT
// ============================================================

void printGPS()
{
  if (
    gps.location.isValid()
  )
  {
    Serial.print(
      "Latitude: "
    );

    Serial.println(
      gps.location.lat(),
      6
    );


    Serial.print(
      "Longitude: "
    );

    Serial.println(
      gps.location.lng(),
      6
    );


    Serial.print(
      "Satellites: "
    );

    Serial.println(
      gps.satellites.value()
    );
  }
  else
  {
    Serial.println(
      "GPS: NO FIX"
    );
  }
}


// ============================================================
//                       INMP441 SETUP
// ============================================================

void setupMicrophone()
{
  Serial.println(
    "Initializing INMP441..."
  );


  i2s_config_t i2s_config =
  {
    .mode =
      (i2s_mode_t)
      (
        I2S_MODE_MASTER |
        I2S_MODE_RX
      ),

    .sample_rate =
      AUDIO_SAMPLE_RATE,

    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_32BIT,

    .channel_format =
      I2S_CHANNEL_FMT_ONLY_LEFT,

    .communication_format =
      I2S_COMM_FORMAT_I2S,

    .intr_alloc_flags =
      0,

    .dma_buf_count =
      4,

    .dma_buf_len =
      256,

    .use_apll =
      false,

    .tx_desc_auto_clear =
      false,

    .fixed_mclk =
      0
  };


  i2s_pin_config_t pin_config =
  {
    .bck_io_num =
      I2S_SCK,

    .ws_io_num =
      I2S_WS,

    .data_out_num =
      I2S_PIN_NO_CHANGE,

    .data_in_num =
      I2S_SD
  };


  i2s_driver_install(
    I2S_NUM_0,
    &i2s_config,
    0,
    NULL
  );


  i2s_set_pin(
    I2S_NUM_0,
    &pin_config
  );


  i2s_zero_dma_buffer(
    I2S_NUM_0
  );


  Serial.println(
    "INMP441 initialized."
  );
}


// ============================================================
//                       NORMAL SOUND READING
// ============================================================

void readSound()
{
  int32_t samples[128];

  size_t bytesRead = 0;


  esp_err_t result =
    i2s_read(
      I2S_NUM_0,
      samples,
      sizeof(samples),
      &bytesRead,
      100
    );


  if (
    result != ESP_OK ||
    bytesRead == 0
  )
  {
    return;
  }


  int count =
    bytesRead /
    sizeof(int32_t);


  long long total = 0;


  for (
    int i = 0;
    i < count;
    i++
  )
  {
    int32_t sample =
      samples[i] >> 14;


    total +=
      abs(sample);
  }


  if (count > 0)
  {
    soundLevel =
      total / count;
  }
}


// ============================================================
//                       VIBRATION
// ============================================================

void vibrationOn()
{
  digitalWrite(
    VIBRATION_PIN,
    HIGH
  );
}


void vibrationOff()
{
  digitalWrite(
    VIBRATION_PIN,
    LOW
  );
}


// ============================================================
//                 SEND BLE MESSAGE
// ============================================================

void sendBLEMessage(
  String message
)
{
  if (
    !deviceConnected
  )
  {
    return;
  }


  bleCharacteristic->setValue(
    message.c_str()
  );


  bleCharacteristic->notify();


  Serial.print(
    "BLE SENT: "
  );

  Serial.println(
    message
  );
}


// ============================================================
//                 START BLE SOS
// ============================================================

void sendSOS()
{
  sendBLEMessage(
    "SOS"
  );
}


// ============================================================
//             RECORD SOS AUDIO
// ============================================================

void recordSOSAudio()
{
  /*
     IMPORTANT:

     We send a simple text header first:

     AUDIO_START:40000

     Then exactly 40000 raw audio bytes.

     Python waits for this header and then
     reads exactly 40000 bytes.

     Python converts those bytes into a
     valid WAV file.
  */


  const int totalSamples =
    AUDIO_TOTAL_SAMPLES;


  const int bufferSamples =
    AUDIO_BUFFER_SAMPLES;


  uint8_t audioBuffer[
    AUDIO_BUFFER_SAMPLES
  ];


  int32_t i2sBuffer[
    AUDIO_BUFFER_SAMPLES
  ];


  Serial.println(
    "AUDIO_RECORDING_START"
  );


  // Tell Python how many bytes are coming
  Serial.print(
    "AUDIO_START:"
  );

  Serial.println(
    totalSamples
  );


  Serial.flush();


  audioRecording = true;


  int samplesRecorded = 0;


  unsigned long recordingStart =
    millis();


  // ----------------------------------------------------------
  // Vibration is active for first 2 seconds
  // while audio recording continues.
  // ----------------------------------------------------------

  vibrationOn();


  while (
    samplesRecorded <
    totalSamples
  )
  {
    size_t bytesRead = 0;


    esp_err_t result =
      i2s_read(
        I2S_NUM_0,

        i2sBuffer,

        sizeof(i2sBuffer),

        &bytesRead,

        1000
      );


    if (
      result != ESP_OK ||
      bytesRead == 0
    )
    {
      continue;
    }


    int samplesRead =
      bytesRead /
      sizeof(int32_t);


    for (
      int i = 0;
      i < samplesRead &&
      samplesRecorded <
      totalSamples;
      i++
    )
    {
      /*
         INMP441 gives 32-bit samples.

         Shift down and convert to
         unsigned 8-bit PCM.
      */

      int32_t sample =
        i2sBuffer[i] >> 14;


      int32_t value =
        sample / 256;


      value += 128;


      if (value < 0)
      {
        value = 0;
      }


      if (value > 255)
      {
        value = 255;
      }


      audioBuffer[
        samplesRecorded %
        bufferSamples
      ] =
        (uint8_t)value;


      samplesRecorded++;


      /*
         When buffer becomes full,
         send it immediately.
      */

      if (
        samplesRecorded %
        bufferSamples == 0
      )
      {
        Serial.write(
          audioBuffer,
          bufferSamples
        );


        Serial.flush();
      }


      /*
         Turn vibration off after 2 seconds.
      */

      if (
        millis() -
        recordingStart >= 2000
      )
      {
        vibrationOff();
      }
    }
  }


  // ----------------------------------------------------------
  // If the last buffer is incomplete
  // send remaining bytes.
  // ----------------------------------------------------------

  int remaining =
    totalSamples %
    bufferSamples;


  if (remaining > 0)
  {
    Serial.write(
      audioBuffer,
      remaining
    );


    Serial.flush();
  }


  vibrationOff();


  audioRecording = false;


  /*
     Tell Python that the audio is complete.

     IMPORTANT:
     This is sent only AFTER all raw audio bytes.
  */

  Serial.println();


  Serial.println(
    "AUDIO_END"
  );


  Serial.println(
    "AUDIO_RECORDING_COMPLETE"
  );


  Serial.flush();
}


// ============================================================
//                    SOS HANDLER
// ============================================================

void handleSOS()
{
  /*
     IMPORTANT:

     Do not print normal Serial messages
     between AUDIO_START and AUDIO_END.

     Python expects raw binary audio.
  */


  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "       SOS BUTTON PRESSED!"
  );

  Serial.println(
    "================================"
  );


  // ----------------------------------------------------------
  // 1. SEND SOS THROUGH BLE
  // ----------------------------------------------------------

  sendSOS();


  // ----------------------------------------------------------
  // 2. PRINT LOCATION BEFORE AUDIO
  // ----------------------------------------------------------

  Serial.println(
    "Emergency Location:"
  );


  if (
    gps.location.isValid()
  )
  {
    Serial.print(
      "Latitude: "
    );

    Serial.println(
      gps.location.lat(),
      6
    );


    Serial.print(
      "Longitude: "
    );

    Serial.println(
      gps.location.lng(),
      6
    );
  }
  else
  {
    Serial.println(
      "GPS: NO FIX"
    );
  }


  // ----------------------------------------------------------
  // 3. RECORD AUDIO
  // ----------------------------------------------------------

  Serial.println(
    "Starting 5-second SOS audio..."
  );


  recordSOSAudio();


  Serial.println(
    "SOS PROCESS COMPLETE"
  );


  Serial.println(
    "================================"
  );


  Serial.println();
}


// ============================================================
//                    SOS BUTTON
// ============================================================

void checkSOSButton()
{
  bool currentState =
    digitalRead(
      SOS_BUTTON
    );


  /*
     INPUT_PULLUP

     HIGH = released
     LOW  = pressed
  */


  if (
    currentState == LOW &&
    lastButtonState == HIGH
  )
  {
    if (
      millis() -
      lastButtonTime >
      BUTTON_DEBOUNCE
    )
    {
      lastButtonTime =
        millis();


      handleSOS();
    }
  }


  lastButtonState =
    currentState;
}


// ============================================================
//                    BLE SETUP
// ============================================================

void setupBLE()
{
  Serial.println(
    "Initializing BLE..."
  );


  BLEDevice::init(
    "Guardian_AI"
  );


  bleServer =
    BLEDevice::createServer();


  bleServer->setCallbacks(
    new MyServerCallbacks()
  );


  BLEService *service =
    bleServer->createService(
      SERVICE_UUID
    );


  bleCharacteristic =
    service->createCharacteristic(
      CHARACTERISTIC_UUID,

      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );


  bleCharacteristic->addDescriptor(
    new BLE2902()
  );


  bleCharacteristic->setValue(
    "Guardian AI | STATUS: SAFE"
  );


  service->start();


  BLEAdvertising *advertising =
    BLEDevice::getAdvertising();


  advertising->addServiceUUID(
    SERVICE_UUID
  );


  advertising->setScanResponse(
    true
  );


  advertising->setMinPreferred(
    0x06
  );


  advertising->setMinPreferred(
    0x12
  );


  BLEDevice::startAdvertising();


  Serial.println(
    "BLE initialized."
  );


  Serial.println(
    "Device Name: Guardian_AI"
  );


  Serial.println(
    "BLE: WAITING"
  );
}


// ============================================================
//              SEND SENSOR DATA
// ============================================================

void sendSensorData()
{
  /*
     Short format so BLE packet remains small.

     M = movement
     H = heart rate
     G = GPS
     S = sound
  */

  String message =
    "D|M:" +
    movementStatus +
    "|H:" +
    String(averageBPM) +
    "|G:" +
    getGPSStatus();


  sendBLEMessage(
    message
  );
}


// ============================================================
//              SERIAL STATUS
// ============================================================

void printSystemStatus()
{
  Serial.println();

  Serial.println(
    "--------------------------------"
  );

  Serial.println(
    "       GUARDIAN AI STATUS"
  );

  Serial.println(
    "--------------------------------"
  );


  Serial.print(
    "Movement: "
  );

  Serial.println(
    movementStatus
  );


  Serial.print(
    "Acceleration: "
  );

  Serial.print(
    accelerationMagnitude,
    2
  );

  Serial.println(
    " g"
  );


  Serial.print(
    "Average BPM: "
  );

  Serial.println(
    averageBPM
  );


  Serial.print(
    "GPS: "
  );

  Serial.println(
    getGPSStatus()
  );


  Serial.print(
    "Sound Level: "
  );

  Serial.println(
    soundLevel
  );


  Serial.print(
    "BLE: "
  );


  if (
    deviceConnected
  )
  {
    Serial.println(
      "CONNECTED"
    );
  }
  else
  {
    Serial.println(
      "WAITING"
    );
  }


  Serial.println(
    "--------------------------------"
  );
}


// ============================================================
//                       SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );


  delay(1000);


  Serial.println();
  Serial.println();


  Serial.println(
    "============================================"
  );


  Serial.println(
    "          GUARDIAN AI STARTING"
  );


  Serial.println(
    " Predictive Emergency Safety Ecosystem"
  );


  Serial.println(
    "============================================"
  );


  Serial.println();


  // ----------------------------------------------------------
  // SOS BUTTON
  // ----------------------------------------------------------

  pinMode(
    SOS_BUTTON,
    INPUT_PULLUP
  );


  // ----------------------------------------------------------
  // VIBRATION
  // ----------------------------------------------------------

  pinMode(
    VIBRATION_PIN,
    OUTPUT
  );


  vibrationOff();


  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  setupMPU6050();


  // ----------------------------------------------------------
  // MAX30102
  // ----------------------------------------------------------

  setupMAX30102();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  setupGPS();


  // ----------------------------------------------------------
  // MICROPHONE
  // ----------------------------------------------------------

  setupMicrophone();


  // ----------------------------------------------------------
  // BLE
  // ----------------------------------------------------------

  setupBLE();


  Serial.println();


  Serial.println(
    "============================================"
  );


  Serial.println(
    "       GUARDIAN AI READY"
  );


  Serial.println(
    "============================================"
  );


  Serial.println();
}


// ============================================================
//                       MAIN LOOP
// ============================================================

void loop()
{
  // GPS needs continuous reading
  readGPS();


  // Check SOS
  checkSOSButton();


  // Normal sensor readings
  readMovement();

  readHeartRate();

  readSound();


  // Serial status
  if (
    millis() -
    lastSerialPrint >=
    SERIAL_INTERVAL
  )
  {
    lastSerialPrint =
      millis();


    printSystemStatus();
  }


  // BLE sensor data
  if (
    millis() -
    lastSensorSend >=
    SENSOR_INTERVAL
  )
  {
    lastSensorSend =
      millis();


    sendSensorData();
  }


  delay(20);
}
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include "SparkFun_SCD4x_Arduino_Library.h"
// #include <bsec2.h>
// #include <b
#include "ScioSense_ENS160.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <driver/i2s.h>
#include <math.h>
#include <esp_pm.h>
#include <BLE2902.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define LED_PIN LED_BUILTIN

SCD4x scd40;
// Bsec2 bme_bsec;
ScioSense_ENS160 ens160(ENS160_I2CADDR_0);

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

#define SENSOR_ID_BME680 0x01
#define SENSOR_ID_ENS160 0x02
#define SENSOR_ID_SCD40  0x03
#define SENSOR_ID_SPL    0x04

#define I2S_WS      8
#define I2S_BCLK    10
#define I2S_DATA_IN 9
#define SAMPLE_RATE 22050
#define SAMPLE_BITS I2S_BITS_PER_SAMPLE_32BIT
#define BUFFER_LEN  256


#include <Adafruit_BME680.h>
Adafruit_BME680 bme; // Use I2C interface

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

static bool     isLowPower      = false;
static uint32_t idleStartMs     = 0;
static bool     ledState        = false;
static uint32_t lastBlinkMs     = 0;
static const uint32_t BLINK_INTERVAL = 500;
static BLEAdvertising *pAdvertising = nullptr;

TaskHandle_t TaskBME680;
TaskHandle_t TaskENS160;
TaskHandle_t TaskSCD40;
TaskHandle_t TaskSPL;

struct BMEData {
  float pressure;
  float altitude;
  uint32_t timestamp_ms;
};

struct ENSData {
  float tvoc;
  float eco2;
  float iaq;  
  uint32_t timestamp_ms;
};

struct SCDData {
  float temperature;
  float humidity;
  float co2;
  uint32_t timestamp_ms;
};

struct SPLData {
  float rms;
  float spl;
  uint32_t timestamp_ms;
};

void sendBLEDataWithTag(uint8_t sensorID, const void* data, size_t length) {
  if (deviceConnected) {
    uint8_t buffer[1 + length];
    buffer[0] = sensorID;
    memcpy(&buffer[1], data, length);
    pCharacteristic->setValue(buffer, sizeof(buffer));
    pCharacteristic->notify();
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) {
    deviceConnected = true;
    if (isLowPower) exitLowPower();
  }
  void onDisconnect(BLEServer* s) {
    deviceConnected = false;
    idleStartMs = millis();
    pAdvertising->start();
  }
};

void setup() {
  Serial.begin(115200);

  esp_pm_config_esp32s3_t pm_cfg = {
    .max_freq_mhz       = 240,
    .min_freq_mhz       =  80,
    .light_sleep_enable = true
  };
  esp_pm_configure(&pm_cfg);

  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  BLEDevice::init("ESP32S3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  if (!scd40.begin()) Serial.println(F("SCD40 not detected."));
  scd40.startPeriodicMeasurement();

  ens160.begin();
  if (ens160.available()) {
    Serial.print("Ok this ens is avaivlable");
    ens160.setMode(ENS160_OPMODE_STD);}  else {
      Serial.println("Something wrong ahppened....");
    };


if (!bme.begin()) {
  Serial.println("Could not find BME680 sensor!");
}
bme.setTemperatureOversampling(BME680_OS_NONE);
bme.setHumidityOversampling(BME680_OS_NONE);
bme.setPressureOversampling(BME680_OS_4X);
bme.setIIRFilterSize(BME680_FILTER_SIZE_3);


  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_IN
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);

xTaskCreatePinnedToCore(taskBME680, "TaskBME680", 4096, NULL, 1, &TaskBME680, 1);
  xTaskCreatePinnedToCore(taskENS160, "TaskENS160", 4096, NULL, 1, &TaskENS160, 1);
  xTaskCreatePinnedToCore(taskSCD40,  "TaskSCD40",  4096, NULL, 1, &TaskSCD40, 1);
  xTaskCreatePinnedToCore(taskSPL,    "TaskSPL",    4096, NULL, 1, &TaskSPL, 1);

  idleStartMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (!deviceConnected && !isLowPower && (now - idleStartMs) > 10000) enterLowPower();

  if (isLowPower) {
    digitalWrite(LED_PIN, HIGH);
  } else if (deviceConnected) {
    if (now - lastBlinkMs >= BLINK_INTERVAL) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  delay(10);
}

void taskBME680(void *pvParameters) {
  for (;;) {
    if (!bme.performReading()) {
      Serial.println("Failed to perform BME680 reading");
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    float pressure = bme.pressure / 100.0;
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print("BME680 | Pressure: ");
    Serial.print(pressure);
    Serial.print(" hPa | Altitude: ");
    Serial.println(altitude);

    BMEData data;
    data.pressure = pressure;
    data.altitude = altitude;
    data.timestamp_ms = millis();

    sendBLEDataWithTag(SENSOR_ID_BME680, &data, sizeof(data));
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void taskENS160(void *pvParameters) {
  for (;;) {
    if (ens160.available()) {
      ens160.measure(true);
      ENSData data;
      data.tvoc = ens160.getTVOC();
      data.eco2 = ens160.geteCO2();
      data.iaq = ens160.getAQI(); 
      data.timestamp_ms = millis();

      Serial.print("ENS160 | TVOC: ");
      Serial.print(data.tvoc);
      Serial.print(" ppb | eCO2: ");
      Serial.print(data.eco2);
      Serial.print(" ppm | IAQ: ");
      Serial.println(data.iaq);

      sendBLEDataWithTag(SENSOR_ID_ENS160, &data, sizeof(data));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskSCD40(void *pvParameters) {
  for (;;) {
    if (scd40.readMeasurement()) {
      SCDData data;
      data.co2 = scd40.getCO2();
      data.temperature = scd40.getTemperature() - 5.0; // Offset
      data.humidity = scd40.getHumidity();
      data.timestamp_ms = millis();

      Serial.print("SCD40 | Temp: ");
      Serial.print(data.temperature);
      Serial.print(" °C | Hum: ");
      Serial.print(data.humidity);
      Serial.print(" % | CO2: ");
      Serial.print(data.co2);
      Serial.println(" ppm");

      sendBLEDataWithTag(SENSOR_ID_SCD40, &data, sizeof(data));
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}


void taskSPL(void *pvParameters) {
  int32_t buffer[BUFFER_LEN];
  size_t bytesRead;
  static int32_t dc_offset = 0;

  for (;;) {
    const int totalSamples = SAMPLE_RATE;
    int samplesRead = 0;
    float sumSquares = 0;

    while (samplesRead < totalSamples) {
      i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);
      int numSamples = bytesRead / sizeof(int32_t);

      for (int i = 0; i < numSamples && samplesRead < totalSamples; i++) {
        int32_t raw = buffer[i] >> 11;
        dc_offset += ((raw - dc_offset) >> 8);
        raw -= dc_offset;
        float sample = raw / 32768.0f;
        sumSquares += sample * sample;
        samplesRead++;
      }
    }

    float rms = sqrt(sumSquares / totalSamples);

    float referenceSPL = 20.0;
    float referenceRMS = 0.012;
    float calibrationOffset = referenceSPL - 20.0 * log10(referenceRMS);
    float spl = 20.0 * log10(rms + 1e-6) + calibrationOffset;

    Serial.print("SPL | RMS: ");
    Serial.print(rms, 6);
    Serial.print(" | SPL: ");
    Serial.print(spl, 1);
    Serial.println(" dB");

    SPLData data;
    data.rms = rms;
    data.spl = spl;
    data.timestamp_ms = millis();

    sendBLEDataWithTag(SENSOR_ID_SPL, &data, sizeof(data));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void enterLowPower() {
  Serial.println(">>> Entering low-power mode");
  isLowPower = true;
  vTaskSuspend(TaskBME680);
  vTaskSuspend(TaskENS160);
  vTaskSuspend(TaskSCD40);
  vTaskSuspend(TaskSPL);
  scd40.stopPeriodicMeasurement();
  setCpuFrequencyMhz(80);
}

void exitLowPower() {
  Serial.println("Exiting low-power mode");
  isLowPower = false;
  setCpuFrequencyMhz(240);
  scd40.startPeriodicMeasurement();
  vTaskResume(TaskBME680);
  vTaskResume(TaskENS160);
  vTaskResume(TaskSCD40);
  vTaskResume(TaskSPL);
  lastBlinkMs = millis();
  ledState = false;
}


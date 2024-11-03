#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// User Parameters for Accelerometer
const uint8_t addr = 0x19; // I2C address of the LIS331 accelerometer
const int maxScale = 24;   // Acceleration range

// LIS331 Registers
const uint8_t CTRL_REG1 = 0x20;
const uint8_t CTRL_REG4 = 0x23;
const uint8_t OUT_X_L = 0x28;
const uint8_t OUT_X_H = 0x29;
const uint8_t OUT_Y_L = 0x2A;
const uint8_t OUT_Y_H = 0x2B;
const uint8_t OUT_Z_L = 0x2C;
const uint8_t OUT_Z_H = 0x2D;

const uint8_t POWERMODE_NORMAL = 0x27;
const uint8_t RANGE_6G = 0x00;
const uint8_t RANGE_12G = 0x10;
const uint8_t RANGE_24G = 0x30;

// BLE Parameters
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// UUIDs
#define SERVICE_UUID        "03c0e0bc-4f14-48ca-843a-9afd45f5db6d"
#define CHARACTERISTIC_UUID "3be4bdcd-67ee-4e11-903f-bf2dc1d9eeab"

// Initialize LIS331 Accelerometer
void initialize(uint8_t addr, int maxScale) {
  Wire.beginTransmission(addr);
  Wire.write(CTRL_REG1);
  Wire.write(POWERMODE_NORMAL); // Normal power mode and 50 Hz sample rate
  Wire.endTransmission();

  Wire.beginTransmission(addr);
  Wire.write(CTRL_REG4);
  switch (maxScale) {
    case 6:
      Wire.write(RANGE_6G);
      break;
    case 12:
      Wire.write(RANGE_12G);
      break;
    case 24:
      Wire.write(RANGE_24G);
      break;
    default:
      Serial.println("Error in the scale provided -- please enter 6, 12, or 24");
  }
  Wire.endTransmission();
}

// Read the data from accelerometer
void readAxes(uint8_t addr, int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(addr);
  Wire.write(OUT_X_L | 0x80); // Set auto-increment bit for multi-byte read
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)6);

  x = Wire.read() | (Wire.read() << 8);
  y = Wire.read() | (Wire.read() << 8);
  z = Wire.read() | (Wire.read() << 8);

  // Two's complement
  if (x > 32767) x -= 65536;
  if (y > 32767) y -= 65536;
  if (z > 32767) z -= 65536;
}

// Calculate g-force from acceleration data
void convertToG(int maxScale, int16_t xAccl, int16_t yAccl, int16_t zAccl, float &xG, float &yG, float &zG) {
  xG = (2.0 * maxScale * xAccl) / 65536.0;
  yG = (2.0 * maxScale * yAccl) / 65536.0;
  zG = (2.0 * maxScale * zAccl) / 65536.0;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize BLE
  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client to connect...");
}

void loop() {
  // Initialize LIS331 accelerometer
  initialize(addr, maxScale);

  // Get acceleration data for x, y, and z axes
  int16_t xAccl, yAccl, zAccl;
  readAxes(addr, xAccl, yAccl, zAccl);

  // Calculate G force based on x, y, z acceleration data
  float xG, yG, zG;
  convertToG(maxScale, xAccl, yAccl, zAccl, xG, yG, zG);

  // Notify BLE clients
  if (deviceConnected) {
      char buffer[50];
      snprintf(buffer, sizeof(buffer), "%.1f, %.1f, %.1f", xG, yG, zG);
      pCharacteristic->setValue(buffer);
      pCharacteristic->notify();
      delay(10); // Avoid BLE congestion
  }

  // Handle device disconnection/reconnection
  if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
      Serial.println("Restart advertising");
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  delay(200); // Short delay to prevent overclocking
}

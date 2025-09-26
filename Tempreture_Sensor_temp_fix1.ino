  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #include <BLEDevice.h>
  #include <BLEUtils.h>
  #include <BLEServer.h>
  #include <BLE2902.h>
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/queue.h>
  #include <time.h>
  #include <esp_sleep.h>
  #include <esp32-hal-bt.h>
  #include <Preferences.h>
  #include <LIS3DHTR.h>

  // RTC configuration
  #define PCF85063_ADDR 0x51  // RTC I2C address

  // Sensor configuration
  #include <Adafruit_MAX31865.h>

  // MAX31865 pins
  #define PIN_SPI_SS   18
  #define PIN_SPI_MOSI 7
  #define PIN_SPI_MISO 2
  #define PIN_SPI_SCK  6
  #define RNOMINAL  100.0
  Adafruit_MAX31865 max31865(PIN_SPI_SS, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK);
  double rref = 400.0; // Default RREF, now editable

  // I2C pins
  #define PIN_I2C_SDA  20
  #define PIN_I2C_SCL  19

  // Display configuration
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 32
  #define OLED_RESET -1
  #define SCREEN_ADDRESS 0x3C
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

  // BLE configuration
  String DEVICE_NAME;  // Will be set dynamically based on chip ID
  #define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
  #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
  #define RX_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

  // Correct Bluetooth logo 10x16 pixels
  const unsigned char ble_logo_bitmap [] PROGMEM = {
    0x10, 0x00,  //    #    
    0x18, 0x00,  //    ##   
    0x14, 0x00,  //    # #  
    0x12, 0x00,  //    #  # 
    0x11, 0x00,  //    #   #
    0x12, 0x00,  //    #  # 
    0x14, 0x00,  //    # #  
    0x18, 0x00,  //    ##   
    0x18, 0x00,  //    ##   
    0x14, 0x00,  //    # #  
    0x12, 0x00,  //    #  # 
    0x11, 0x00,  //    #   #
    0x12, 0x00,  //    #  # 
    0x14, 0x00,  //    # #  
    0x18, 0x00,  //    ##   
    0x10, 0x00   //    #    
  };

  // Button configuration
  #define BUTTON_PIN 9
  #define BUTTON2_PIN 1
  const unsigned long DEBOUNCE_DELAY = 300;

  // Battery configuration
  #define BATTERY_PIN 3
  #define BATTERY_HIGH 800
  #define BATTERY_MID 720
  #define BATTERY_BOX_WIDTH 10
  #define BATTERY_BOX_HEIGHT 18
  #define BATTERY_SEGMENT_GAP 1
  #define BATTERY_BORDER 2
  #define BATTERY_SEGMENTS 3

  // Display modes
  enum DisplayMode {
    DISPLAY_TEMP,
    DISPLAY_DEVICE_NAME
  };

#define ADC_MAX_VALUE 4095.0f
//#define ADC_REF_VOLTAGE 3.3f    // 若板子 ADC 参考电压或衰减不同，可调整或校准
#define ADC_REF_VOLTAGE 4.16f    // 临时调整方案
// 分压比：取点在第二个200k和100k之间 => 100 / (200+200+100) = 0.2
#define DIVIDER_RATIO (100.0f / (200.0f + 200.0f + 100.0f))  // = 0.2
#define BATTERY_MULTIPLIER (1.0f / DIVIDER_RATIO)            // = 5.0

// 电池电压范围（根据你电池类型调整）
#define BATTERY_MIN_V 3.0f
#define BATTERY_MAX_V 4.2f

float readBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);                 // 原始 ADC 值
    float vIo = (raw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE; // IO 点电压 (0..3.3V)
    float vBat = vIo * BATTERY_MULTIPLIER;             // 还原电池电压
    return vBat;
}

int batteryPercent(float vbat) {
    float percent = (vbat - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100.0f;
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    return (int)(percent + 0.5f);
}



  //息屏部分
  const unsigned long INACTIVITY_TIMEOUT = 10000; // 30秒无活动息屏
  const float MOVEMENT_THRESHOLD = 0.3;          // 运动检测阈值
  unsigned long lastMovementTime = 0;
  bool oledSleeping = false;
  bool movementDetected = false;

  // Global variables
  BLEServer *pServer = nullptr;
  BLECharacteristic *pCharacteristic = nullptr;
  BLECharacteristic *rxCharacteristic = nullptr;
  BLEAdvertising *pAdvertising = nullptr;
  bool deviceConnected = false;
  bool bleActive = false;
  volatile bool buttonPressed = false;
  volatile bool button2Pressed = false;
  unsigned long lastDebounceTime = 0;
  unsigned long lastDebounceTime2 = 0;
  DisplayMode currentDisplayMode = DISPLAY_TEMP;

  // Task handles
  TaskHandle_t tempTaskHandle = NULL;
  TaskHandle_t displayTaskHandle = NULL;
  TaskHandle_t bleTaskHandle = NULL;
  TaskHandle_t cmdTaskHandle = NULL;
  TaskHandle_t imuTaskHandle = NULL;

  // Queues for inter-task communication
  QueueHandle_t tempQueue;
  QueueHandle_t cmdQueue;
  QueueHandle_t imuQueue;

  // Temperature structure for queue
  struct TempData {
    float temperature;
    unsigned long timestamp;
  };

  // Command structure for queue
  struct CmdData {
    String type;
    String value;
  };

  // Shared data structure
  struct SharedData {
      float temperature;
      bool deviceConnected;
      bool bleActive;
      time_t currentTime;
  };

  // Shared data instance
  SharedData sharedData = {
      .temperature = 25.0,
      .deviceConnected = false,
      .bleActive = false,
      .currentTime = 0
  };

  // BLE blink configuration
  const unsigned long BLE_BLINK_SPEED = 6000;  // Blink speed in milliseconds (1000ms = 1 second)
  const unsigned long BLE_BLINK_ON_TIME = BLE_BLINK_SPEED / 2;  // Time the symbol is visible
  unsigned long lastBlinkTime = 0;  // Track last blink state change
  bool blinkState = false;  // Current blink state

  // Add these global variables at the top with other globals
  SemaphoreHandle_t bleMutex = NULL;

  // Add this with other global variables
  unsigned long lastButtonPressTime = 0;
  const unsigned long BUTTON_COOLDOWN = 1000; // 1 second cooldown between BLE toggles

  // Add these with other global variables
  bool bleTransmitting = false;  // Flag for BLE advertising state

  // Transmission frequency control
  unsigned long bleTransmissionInterval = 10000;  // Default 1 second interval
  unsigned long lastTransmissionTime = 0;        // Track last transmission time

  // Device name preferences
  Preferences preferences;
  String deviceDisplayName = "Bacon_test";  // Default display name

  // CRC-8 polynomial: x^8 + x^2 + x + 1 (0x07)
  const uint8_t CRC8_POLY = 0x07;

  // IMU configuration
  LIS3DHTR<TwoWire> LIS;

  // IMU data structure
  struct IMUData {
    float accelX;
    float accelY;
    float accelZ;
    unsigned long timestamp;
  };


bool checkMovement() {
 IMUData imuData;
    if (xQueueReceive(imuQueue, &imuData, 0) == pdTRUE) {
        // 计算加速度变化量
        static float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
        float deltaX = abs(imuData.accelX - lastAccelX);
        float deltaY = abs(imuData.accelY - lastAccelY);
        float deltaZ = abs(imuData.accelZ - lastAccelZ);
        
        lastAccelX = imuData.accelX;
        lastAccelY = imuData.accelY;
        lastAccelZ = imuData.accelZ;
        
        // 如果任一轴的加速度变化超过阈值，则认为有运动
        return (deltaX > MOVEMENT_THRESHOLD || 
                deltaY > MOVEMENT_THRESHOLD || 
                deltaZ > MOVEMENT_THRESHOLD);
    }
    return false;
  
}


  // RTC Functions
  void clearStopBit() {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x00);  // Control_1 register
    Wire.endTransmission();

    Wire.requestFrom(PCF85063_ADDR, 1);
    if (Wire.available()) {
      byte ctrl1 = Wire.read();
      if (ctrl1 & 0x80) {
        ctrl1 &= ~0x80;  // Clear STOP bit
        Wire.beginTransmission(PCF85063_ADDR);
        Wire.write(0x00);
        Wire.write(ctrl1);
        Wire.endTransmission();
        Serial.println("Cleared STOP bit in RTC control register");
      }
    }
  }

  void clearCIF() {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x01);  // Control_2 register
    Wire.endTransmission();

    Wire.requestFrom(PCF85063_ADDR, 1);
    if (Wire.available()) {
      byte ctrl2 = Wire.read();
      if (ctrl2 & 0x20) {
        ctrl2 &= ~0x20;  // Clear CIF bit
        Wire.beginTransmission(PCF85063_ADDR);
        Wire.write(0x01);
        Wire.write(ctrl2);
        Wire.endTransmission();
        Serial.println("Cleared CIF bit in RTC control register");
      }
    }
  }

  bool checkRTC() {
    Wire.beginTransmission(PCF85063_ADDR);
    int res = Wire.endTransmission();
    if (res != 0) {
      Serial.printf("RTC not responding! I2C error: %d\n", res);
      return false;
    }
    Serial.println("RTC communication OK");
    return true;
  }

  bool readDate(byte &day, byte &month, byte &year) {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x07);  // Start at day register
    if (Wire.endTransmission(false) != 0) return false;

    // Read 4 bytes to get Day (07h), Month (09h), and Year (0Ah), skipping Weekday (08h)
    if (Wire.requestFrom(PCF85063_ADDR, 4) != 4) return false;

    byte raw_day = Wire.read() & 0x3F;
    Wire.read(); // Discard the Weekday register value (0x08)
    byte raw_month = Wire.read() & 0x1F;
    byte raw_year = Wire.read();

    day = ((raw_day >> 4) * 10) + (raw_day & 0x0F);
    month = ((raw_month >> 4) * 10) + (raw_month & 0x0F);
    year = ((raw_year >> 4) * 10) + (raw_year & 0x0F);

    if (day < 1 || day > 31 || month < 1 || month > 12) return false;

    return true;
  }

  bool readTime(byte &hour, byte &minute, byte &second) {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x04);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(PCF85063_ADDR, 3) != 3) return false;

    byte raw_sec = Wire.read() & 0x7F;
    byte raw_min = Wire.read() & 0x7F;
    byte raw_hour = Wire.read() & 0x3F;

    second = ((raw_sec >> 4) * 10) + (raw_sec & 0x0F);
    minute = ((raw_min >> 4) * 10) + (raw_min & 0x0F);
    hour = ((raw_hour >> 4) * 10) + (raw_hour & 0x0F);

    if (second > 59 || minute > 59 || hour > 23) return false;

    return true;
  }

  void stopClock() {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x00);  // Control_1 register
    Wire.endTransmission();

    Wire.requestFrom(PCF85063_ADDR, 1);
    if (Wire.available()) {
      byte ctrl1 = Wire.read();
      if ((ctrl1 & 0x80) == 0) { // Only stop if it's running
        ctrl1 |= 0x80;  // Set STOP bit
        Wire.beginTransmission(PCF85063_ADDR);
        Wire.write(0x00);
        Wire.write(ctrl1);
        Wire.endTransmission();
        Serial.println("Clock stopped for setting.");
      }
    }
  }

  void startClock() {
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x00);  // Control_1 register
    Wire.endTransmission();

    Wire.requestFrom(PCF85063_ADDR, 1);
    if (Wire.available()) {
      byte ctrl1 = Wire.read();
      if (ctrl1 & 0x80) { // Only start if it's stopped
        ctrl1 &= ~0x80;  // Clear STOP bit
        Wire.beginTransmission(PCF85063_ADDR);
        Wire.write(0x00);
        Wire.write(ctrl1);
        Wire.endTransmission();
        Serial.println("Clock restarted.");
      }
    }
  }

  bool setDateTime(byte day, byte month, byte year, byte hour, byte minute, byte second) {
    stopClock();

    // Convert decimal to BCD
    byte bcd_sec = ((second / 10) << 4) | (second % 10);
    byte bcd_min = ((minute / 10) << 4) | (minute % 10);
    byte bcd_hour = ((hour / 10) << 4) | (hour % 10);
    byte bcd_day = ((day / 10) << 4) | (day % 10);
    byte bcd_weekday = 0; // Weekday is not used
    byte bcd_month = ((month / 10) << 4) | (month % 10);
    byte bcd_year = ((year / 10) << 4) | (year % 10);

    // Set all registers sequentially from Seconds (04h) to Years (0Ah)
    Wire.beginTransmission(PCF85063_ADDR);
    Wire.write(0x04);
    Wire.write(bcd_sec);
    Wire.write(bcd_min);
    Wire.write(bcd_hour);
    Wire.write(bcd_day);
    Wire.write(bcd_weekday);
    Wire.write(bcd_month);
    Wire.write(bcd_year);
    int res = Wire.endTransmission();

    startClock();

    if (res != 0) {
      Serial.printf("I2C error during setDateTime(): %d\n", res);
      return false;
    }

    Serial.printf("Date/Time set to: %04d/%02d/%02d %02d:%02d:%02d\n", 2000 + year, month, day, hour, minute, second);
    return true;
  }

  

  time_t getRTCTime() {
    byte hour, minute, second;
    byte day, month, year;
    
    if (!readTime(hour, minute, second) || !readDate(day, month, year)) {
      return 0; // Return 0 if RTC read fails
    }
    
    struct tm timeinfo;
    timeinfo.tm_year = 2000 + year - 1900; // Convert to tm_year format
    timeinfo.tm_mon = month - 1;           // Convert to tm_mon format (0-11)
    timeinfo.tm_mday = day;
    timeinfo.tm_hour = hour;
    timeinfo.tm_min = minute;
    timeinfo.tm_sec = second;
    timeinfo.tm_isdst = 0; // No DST
    
    return mktime(&timeinfo);
  }

  // Forward declarations
  float getTemperature();
  void sendCommandPacket(const String& type, const String& value);
  void drawTemperature(float temp);
  void drawDeviceName();
  String generateDeviceName();

  // Button ISRs
  void IRAM_ATTR buttonISR() {
    if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
      buttonPressed = true;
      lastDebounceTime = millis();
    }
  }

  void IRAM_ATTR button2ISR() {
    if (millis() - lastDebounceTime2 > DEBOUNCE_DELAY) {
      button2Pressed = true;
      lastDebounceTime2 = millis();
    }
  }

  // Task functions
  void tempTask(void * parameter) {
    TempData tempData;
    for (;;) {
      tempData.temperature = getTemperature();
      tempData.timestamp = millis();
      
      // Update shared data immediately
      sharedData.temperature = tempData.temperature;
      
      xQueueSend(tempQueue, &tempData, 0);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

void OLED_Sleep() {
   display.ssd1306_command(SSD1306_DISPLAYOFF); // 关闭显示
  delay(100);
  Serial.println("OLED Sleep Mode");
}

void OLED_Wake() {
   display.ssd1306_command(SSD1306_DISPLAYON); // 重新开启显示
  delay(100);
  Serial.println("OLED Wake Up");
}

  void displayTask(void * parameter) {
  TempData tempData;
    unsigned long lastTimeUpdate = 0;
    unsigned long lastRotationCheck = 0;
    float lastDisplayedTemp = 0.0;
    bool hasValidTemp = false;
    int lastRotation = -1;
    unsigned long lastActivityCheck = 0;
    
    for (;;) {
        unsigned long currentMillis = millis();
        
        // 定期检查活动状态（每100ms）
        if (currentMillis - lastActivityCheck >= 100) {
            lastActivityCheck = currentMillis;
            
            if (checkMovement()) {
                lastMovementTime = currentMillis;
                movementDetected = true;
                
                // 如果屏幕是休眠状态，唤醒屏幕
                if (oledSleeping) {
                    OLED_Wake();
                    oledSleeping = false;
                    Serial.println("Movement detected - OLED waking up");
                }
            }
            
            // 检查是否需要息屏
            if (!oledSleeping && currentMillis - lastMovementTime >= INACTIVITY_TIMEOUT) {
                OLED_Sleep();
                oledSleeping = true;
                Serial.println("No movement detected - OLED sleeping");
            }
        }
    
    
      // Check for new temperature data
      if (!oledSleeping) {

        if (xQueueReceive(tempQueue, &tempData, pdMS_TO_TICKS(50)) == pdTRUE) {  // Reduced timeout for faster response
          // Update shared data with new temperature
          sharedData.temperature = tempData.temperature;
          lastDisplayedTemp = tempData.temperature;
          hasValidTemp = true;
          
          // Update RTC time every second
          unsigned long currentMillis = millis();
          if (currentMillis - lastTimeUpdate >= 1000) {
            // Get time from hardware RTC
            time_t rtcTime = getRTCTime();
            if (rtcTime > 0) {
              sharedData.currentTime = rtcTime;
            }
            lastTimeUpdate = currentMillis;
          }
          drawTemperature(tempData.temperature);
        } else {
          // Check for rotation changes more frequently
          unsigned long currentMillis = millis();
          if (currentMillis - lastRotationCheck >= 25) {  // Check rotation every 25ms for very responsive rotation
            // Check if rotation has changed
            IMUData imuData;
            if (xQueueReceive(imuQueue, &imuData, 0) == pdTRUE) {
              int currentRotation = 0;
              if (imuData.accelX > 0.5) {
                currentRotation = 1;  // Portrait
              } else if (imuData.accelY > 0.5) {
                currentRotation = 2;  // Landscape upside down
              } else {
                currentRotation = 0;  // Landscape normal
              }
              
              // Redraw if rotation changed or if we have a valid temperature that's different
              if (currentRotation != lastRotation || 
                  (hasValidTemp && abs(sharedData.temperature - lastDisplayedTemp) > 0.1)) {
                drawTemperature(sharedData.temperature);
                lastDisplayedTemp = sharedData.temperature;
                lastRotation = currentRotation;
              }
            }
            lastRotationCheck = currentMillis;
          }
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);  // Much faster update rate for very responsive rotation
    }
  }

  void bleTask(void * parameter) {
    TempData tempData;
    unsigned long lastBatteryUpdate = 0;
    for (;;) {
      if (deviceConnected && bleTransmitting) {
        unsigned long currentMillis = millis();
        
        // Check if it's time to transmit temperature
        if (currentMillis - lastTransmissionTime >= bleTransmissionInterval) {
          if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            float temp = getTemperature();
            char tempStr[10];
            dtostrf(temp, 4, 1, tempStr);
            sendCommandPacket("temp", tempStr);
            lastTransmissionTime = currentMillis;
            xSemaphoreGive(bleMutex);
          }
        }
        
        // Send battery reading every 5 seconds (independent of temperature frequency)
        if (currentMillis - lastBatteryUpdate >= 5000) {
          if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int batteryReading = analogRead(BATTERY_PIN);
            char batteryStr[10];
            sprintf(batteryStr, "%d", batteryReading);
            sendCommandPacket("battery", batteryStr);
            lastBatteryUpdate = currentMillis;
            xSemaphoreGive(bleMutex);
          }
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Check more frequently for better responsiveness
    }
  }

  void cmdTask(void * parameter) {
    CmdData cmdData;
    for (;;) {
      if (xQueueReceive(cmdQueue, &cmdData, portMAX_DELAY) == pdTRUE) {
        if (cmdData.type == "get_temp") {
          TempData tempData;
          if (xQueueReceive(tempQueue, &tempData, 0) == pdTRUE) {
            char tempStr[10];
            dtostrf(tempData.temperature, 4, 1, tempStr);
            sendCommandPacket("temp", tempStr);
          }
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }

  void imuTask(void * parameter) {
    IMUData imuData;
    for (;;) {
      // Get IMU acceleration data
      imuData.accelX = LIS.getAccelerationX();
      imuData.accelY = LIS.getAccelerationY();
      imuData.accelZ = LIS.getAccelerationZ();
      imuData.timestamp = millis();
      xQueueSend(imuQueue, &imuData, 0);
        (10 / portTICK_PERIOD_MS);  // 100Hz sampling rate for much faster rotation response
    }
  }

  // BLE Callbacks
  class MyCallbacks : public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *characteristic) override {
        String cmd = String(characteristic->getValue().c_str());
        Serial.print("Received CMD: ");
              Serial.println(cmd);
              
        int cmdIndex = cmd.indexOf("|");
        int typeIndex = cmd.indexOf("|", cmdIndex + 1);
        int dataIndex = cmd.indexOf("|", typeIndex + 1);
        int endIndex = cmd.indexOf("|", dataIndex + 1);

        if (cmdIndex != -1 && typeIndex != -1 && dataIndex != -1 && endIndex != -1) {
          String keyword = cmd.substring(0, cmdIndex);
          String type = cmd.substring(cmdIndex + 1, typeIndex);
          String payload = cmd.substring(typeIndex + 1, dataIndex);
          String received_crc = cmd.substring(dataIndex + 1, endIndex);
          String endTag = cmd.substring(endIndex + 1);

          // Calculate CRC for received data
          String data_to_check = type + "|" + payload;
          uint8_t calculated_crc = calculateCRC8(data_to_check);
          uint8_t received_crc_value = strtol(received_crc.c_str(), NULL, 16);

          if (keyword == "cmd" && endTag == "end" && calculated_crc == received_crc_value) {
            if (type == "get_temp") {
              float temp = getTemperature();
              char tempStr[10];
              dtostrf(temp, 4, 1, tempStr);
              sendCommandPacket("temp", tempStr);
            }
            else if (type == "time_sync") {
              // Parse DDMMYYHHMMSS format
              if (payload.length() == 12) {  // DDMMYYHHMMSS = 12 characters
                int day = (payload.substring(0, 2)).toInt();
                int month = (payload.substring(2, 4)).toInt();
                int year = (payload.substring(4, 6)).toInt();
                int hours = (payload.substring(6, 8)).toInt();
                int minutes = (payload.substring(8, 10)).toInt();
                int seconds = (payload.substring(10, 12)).toInt();
                
                // Validate the parsed values
                if (day >= 1 && day <= 31 && month >= 1 && month <= 12 && 
                    year >= 0 && year <= 99 && hours >= 0 && hours <= 23 && 
                    minutes >= 0 && minutes <= 59 && seconds >= 0 && seconds <= 59) {
                  
                  // Convert 2-digit year to full year (assume 20xx)
                  int fullYear = 2000 + year;
                  
                  // Set date and time using RTC
                  if (setDateTime(day, month, year, hours, minutes, seconds)) {
                    // Update shared data with new RTC time
                    sharedData.currentTime = getRTCTime();
                    
                    // Print the updated time
                    Serial.printf("RTC date/time updated to: %04d/%02d/%02d %02d:%02d:%02d\n", 
                                fullYear, month, day, hours, minutes, seconds);
                  } else {
                    Serial.println("Failed to update RTC date/time");
                  }
                } else {
                  Serial.printf("Invalid date/time values: %02d/%02d/%02d %02d:%02d:%02d\n", 
                              day, month, year, hours, minutes, seconds);
                }
              } else {
                Serial.printf("Invalid time_sync payload length: %d (expected 12 for DDMMYYHHMMSS)\n", payload.length());
              }
            }
            else if (type == "set_freq") {
              // Parse frequency in milliseconds
              int frequency = payload.toInt();
              if (frequency >= 100 && frequency <= 60000) {  // 100ms to 60 seconds
                bleTransmissionInterval = frequency;
                lastTransmissionTime = 0;  // Reset timer to start immediately
                Serial.printf("Transmission frequency set to %d ms\n", frequency);
                
                // Send confirmation
                char freqStr[10];
                sprintf(freqStr, "%d", frequency);
                sendCommandPacket("freq_set", freqStr);
              } else {
                Serial.println("Invalid frequency value. Must be between 100-60000 ms");
                sendCommandPacket("error", "Invalid frequency range");
              }
            }
            else if (type == "get_freq") {
              // Send current frequency
              char freqStr[10];
              sprintf(freqStr, "%d", bleTransmissionInterval);
              sendCommandPacket("current_freq", freqStr);
            }
            else if (type == "set_name") {
              // Set device display name
              if (payload.length() > 0 && payload.length() <= 20) {  // Max 20 characters
                deviceDisplayName = payload;
                preferences.putString("dev_name", deviceDisplayName);
                Serial.printf("Device display name set to: %s\n", deviceDisplayName.c_str());
                
                // Send confirmation
                sendCommandPacket("name_set", deviceDisplayName);
              } else {
                Serial.println("Invalid name length. Must be 1-20 characters");
                sendCommandPacket("error", "Invalid name length");
              }
            }
            else if (type == "get_name") {
              // Send current device display name
              sendCommandPacket("current_name", deviceDisplayName);
            }
            else if (type == "set_rref") {
              double newRref = payload.toDouble();
              if (newRref >= 100.0 && newRref <= 1000.0) {
                rref = newRref;
                char rrefStr[16];
                dtostrf(rref, 7, 2, rrefStr);
                sendCommandPacket("rref_set", rrefStr);
                Serial.printf("RREF set to %.2f\n", rref);
              } else {
                sendCommandPacket("error", "Invalid RREF");
              }
            }
            else if (type == "get_rref") {
              char rrefStr[16];
              dtostrf(rref, 7, 2, rrefStr);
              sendCommandPacket("current_rref", rrefStr);
            }
            CmdData cmdData = {type, payload};
            xQueueSend(cmdQueue, &cmdData, 0);
              }
          }
      }
  };

  class MyServerCallbacks : public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) override {
      deviceConnected = true;
          sharedData.deviceConnected = true;
          Serial.println("BLE client connected.");
      }
      
      void onDisconnect(BLEServer* pServer) override {
      deviceConnected = false;
          sharedData.deviceConnected = false;
          Serial.println("BLE client disconnected.");
          
      if (bleActive) {
        pServer->getAdvertising()->start();
        Serial.println("BLE advertising restarted for reconnection");
          }
      }
  };

  // Helper Functions
  String generateDeviceName() {
    // Get the ESP32-C6 unique chip ID using the correct method
    uint64_t chipId = ESP.getEfuseMac(); // Get MAC address as unique identifier
    
    // Convert to full hex string (16 characters for 64-bit MAC)
    char deviceId[17];
    snprintf(deviceId, sizeof(deviceId), "%016llX", (unsigned long long)chipId);
    
    // Create device name in format: ScyclonLT_*ID*
    // BLE device names are limited to 31 characters
    // "ScyclonLT_" = 10 characters + 16 characters for full MAC = 26 characters total
    String deviceName = "ScyclonLT_" + String(deviceId);
    
    Serial.printf("Generated device name: %s\n", deviceName.c_str());
    Serial.printf("Chip ID (MAC): %016llX\n", (unsigned long long)chipId);
    Serial.printf("Device name length: %d characters\n", deviceName.length());
    
    return deviceName;
  }

  float getTemperature() {
    return max31865.temperature(RNOMINAL, rref);
  }

 void drawTemperature(float temp) {
    if (currentDisplayMode == DISPLAY_DEVICE_NAME) {
        display.setRotation(0); // Always landscape for device name/info
        drawDeviceName();
        return;
    }
    display.clearDisplay();
    IMUData imuData;
    bool hasIMUData = (xQueueReceive(imuQueue, &imuData, 0) == pdTRUE);
    bool isPortrait = false;
    if (hasIMUData && imuData.accelX > 0.5) {
        display.setRotation(1); // Portrait
        isPortrait = true;
    } else if (hasIMUData && imuData.accelY > 0.5) {
        display.setRotation(2); // Landscape upside down
    } else {
        display.setRotation(0); // Landscape normal
    }

    if (isPortrait) {
        // Portrait: only show temperature
        display.setTextSize(2);
        char tempStr[8];
        snprintf(tempStr, sizeof(tempStr), "%.1fC", temp);
        int startX = 12;
        int lineSpacing = 16;
        int len = strlen(tempStr);
        int totalHeight = len * lineSpacing;
        int y0 = (128 - totalHeight) / 2;
        for (int i = 0; tempStr[i] != '\0'; i++) {
            display.setCursor(startX, y0 + i * lineSpacing);
            display.write(tempStr[i]);
        }
    } else {
        // -------- Landscape 模式 --------
        // 温度
        display.setTextSize(2);
        display.setCursor(15, 8);
        display.print(temp, 1);
        display.drawCircle(75, 10, 2, WHITE);
        display.setCursor(82, 8);
        display.print("C");

        // BLE 图标
        int bleX = 102;
        int bleY = 2;
        if (bleActive && bleTransmitting) {
            if (!deviceConnected) {
                unsigned long currentTime = millis();
                if (currentTime - lastBlinkTime >= (blinkState ? BLE_BLINK_ON_TIME : (BLE_BLINK_SPEED - BLE_BLINK_ON_TIME))) {
                    blinkState = !blinkState;
                    lastBlinkTime = currentTime;
                }
                if (blinkState) {
                    display.drawBitmap(bleX - 4, bleY, ble_logo_bitmap, 10, 16, WHITE);
                }
            } else {
                display.drawBitmap(bleX - 4, bleY, ble_logo_bitmap, 10, 16, WHITE);
            }
        } else if (bleActive && !bleTransmitting) {
            display.drawBitmap(bleX - 4, bleY, ble_logo_bitmap, 10, 16, WHITE);
            display.drawLine(bleX - 4, bleY + 8, bleX + 6, bleY + 8, WHITE);
        }

        // -------- 电池 --------
        int batteryX = bleX + 16;
        int batteryY = bleY;

        // ADC 原始值 & 电池电量
        int raw = analogRead(BATTERY_PIN);
        float vbat = readBatteryVoltage();
        int percent = batteryPercent(vbat);

        // 电池外框
        display.drawRect(batteryX, batteryY, BATTERY_BOX_WIDTH, BATTERY_BOX_HEIGHT, WHITE);
        display.fillRect(batteryX + BATTERY_BORDER, batteryY - 2,
                         BATTERY_BOX_WIDTH - (2 * BATTERY_BORDER), 2, WHITE);

        // 电池格子
        int segments = (percent * BATTERY_SEGMENTS) / 100;
        if (percent > 0 && segments == 0) segments = 1;
        int usableHeight = BATTERY_BOX_HEIGHT - (2 * BATTERY_BORDER);
        int totalGap = (BATTERY_SEGMENTS - 1) * BATTERY_SEGMENT_GAP;
        int segmentHeight = (usableHeight - totalGap) / BATTERY_SEGMENTS;
        int remainder = (usableHeight - totalGap) % BATTERY_SEGMENTS;
        int segY = batteryY + BATTERY_BOX_HEIGHT - BATTERY_BORDER;
        for (int i = 0; i < segments; i++) {
            int thisSegHeight = segmentHeight;
            if (i < remainder) thisSegHeight++;
            segY -= thisSegHeight;
            display.fillRect(batteryX + BATTERY_BORDER, segY,
                             BATTERY_BOX_WIDTH - (2 * BATTERY_BORDER),
                             thisSegHeight, WHITE);
            if (i < BATTERY_SEGMENTS - 1) segY -= BATTERY_SEGMENT_GAP;
        }

        // -------- 在电池下方显示 ADC 原始值 + 电量百分比 --------
        display.setTextSize(1);
        int txtY = batteryY + BATTERY_BOX_HEIGHT + 1;

        // 先显示 ADC 值
        display.setCursor(batteryX - 50, txtY+1);   // 向左移一点，下移一点
        display.print(raw);
        display.print(" ");

        // 再显示百分比
        display.print(percent);
        display.print("%");
    }

    display.display();
}

  void drawDeviceName() {
    display.setRotation(0); // Always landscape for device name/info
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(10, 10);
    display.print("Device: ");
    display.println(deviceDisplayName);
    
    // Draw time instead of BLE status
    display.setCursor(10, 25);
    struct tm timeinfo;
    localtime_r(&sharedData.currentTime, &timeinfo);
    char timeStr[9];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
    display.print("Time: ");
    display.print(timeStr);
    
    display.display();
  }

  // Add this function before sendCommandPacket
  uint8_t calculateCRC8(const String& data) {
    uint8_t crc = 0;
    
    for (char c : data) {
      crc ^= c;
      for (int i = 0; i < 8; i++) {
        if (crc & 0x80) {
          crc = (crc << 1) ^ CRC8_POLY;
        } else {
          crc = (crc << 1);
      }
      }
    }
    
    return crc;
  }

  void sendCommandPacket(const String& type, const String& value) {
    if (!deviceConnected) return;
    
    String data = type + "|" + value;
    uint8_t crc = calculateCRC8(data);
    String packet = "cmd|" + data + "|" + String(crc, HEX) + "|end";
    
    if (pCharacteristic) {
      pCharacteristic->setValue(packet.c_str());
      pCharacteristic->notify();
    }
  }

  void startBLE() {
    if (!bleActive) {
      if (xSemaphoreTake(bleMutex, portMAX_DELAY) == pdTRUE) {
        Serial.println("Starting BLE initialization from scratch...");
        
        // Reset all pointers first
        pServer = nullptr;
        pCharacteristic = nullptr;
        rxCharacteristic = nullptr;
        pAdvertising = nullptr;
        
        // Add delay before initialization to ensure clean state
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Initialize BLE device
        BLEDevice::init(DEVICE_NAME.c_str());
        
        // Add delay after init
        vTaskDelay(pdMS_TO_TICKS(200));
        
        // Create server
        pServer = BLEDevice::createServer();
        if (!pServer) {
          Serial.println("Error: Failed to create BLE server");
          bleActive = false;
          sharedData.bleActive = false;
          xSemaphoreGive(bleMutex);
          return;
        }
        pServer->setCallbacks(new MyServerCallbacks());
        
        BLEService *pService = pServer->createService(SERVICE_UUID);
        if (!pService) {
          Serial.println("Error: Failed to create BLE service");
          bleActive = false;
          sharedData.bleActive = false;
          pServer = nullptr;
          xSemaphoreGive(bleMutex);
          return;
        }
        
        pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
        );
        if (!pCharacteristic) {
          Serial.println("Error: Failed to create TX characteristic");
          bleActive = false;
          sharedData.bleActive = false;
          pServer = nullptr;
          xSemaphoreGive(bleMutex);
          return;
        }
        pCharacteristic->addDescriptor(new BLE2902());
        
        rxCharacteristic = pService->createCharacteristic(
            RX_UUID,
            BLECharacteristic::PROPERTY_WRITE
        );
        if (!rxCharacteristic) {
          Serial.println("Error: Failed to create RX characteristic");
          bleActive = false;
          sharedData.bleActive = false;
          pServer = nullptr;
          pCharacteristic = nullptr;
          xSemaphoreGive(bleMutex);
          return;
        }
        rxCharacteristic->setCallbacks(new MyCallbacks());
        
        pService->start();
        
        // Setup advertising
        pAdvertising = BLEDevice::getAdvertising();
        if (!pAdvertising) {
          Serial.println("Error: Failed to get advertising object");
          bleActive = false;
          sharedData.bleActive = false;
          pServer = nullptr;
          pCharacteristic = nullptr;
          rxCharacteristic = nullptr;
          xSemaphoreGive(bleMutex);
          return;
        }
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->setScanResponse(false);
        pAdvertising->setMinPreferred(0x06);
        pAdvertising->setMinPreferred(0x12);
        
        bleActive = true;
        sharedData.bleActive = true;
        Serial.println("BLE initialization completed successfully!");
        
        xSemaphoreGive(bleMutex);
      }
    }
  }

  void toggleBLE() {
    if (!bleActive) {
      Serial.println("BLE not active -> Starting BLE system...");
      
      // Start BLE from scratch
      startBLE();
      
      // Check if BLE initialization was successful
      if (bleActive) {
        // Start advertising
        if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          try {
            // Reset all transmission state
            bleTransmitting = false;
            lastTransmissionTime = 0;
            deviceConnected = false;
            sharedData.deviceConnected = false;
            
            BLEDevice::startAdvertising();
            bleTransmitting = true;
            Serial.println("BLE advertising started - device discoverable");
          } catch (...) {
            Serial.println("Error starting BLE advertising!");
            bleTransmitting = false;
          }
          xSemaphoreGive(bleMutex);
        }
      } else {
        Serial.println("BLE initialization failed - cannot start advertising");
      }
    } else {
      // Toggle only the transmission/advertising, not the entire BLE system
      if (bleTransmitting) {
        Serial.println("BLE transmitting -> Stopping transmission only...");
        if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          try {
            BLEDevice::stopAdvertising();
            bleTransmitting = false;
            Serial.println("BLE transmission stopped - device not discoverable");
          } catch (...) {
            Serial.println("Error stopping BLE advertising!");
          }
          xSemaphoreGive(bleMutex);
        }
      } else {
        Serial.println("BLE not transmitting -> Starting transmission...");
        if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          try {
            BLEDevice::startAdvertising();
            bleTransmitting = true;
            Serial.println("BLE transmission started - device discoverable");
          } catch (...) {
            Serial.println("Error starting BLE advertising!");
            bleTransmitting = false;
          }
          xSemaphoreGive(bleMutex);
        }
      }
    }
  }

  

  void setup() {
    Serial.begin(115200);
    setCpuFrequencyMhz(80);    // 设置主频为 80 MHz

    // Initialize Preferences FIRST - before any other operations
    preferences.begin("bacon", false);  // false = read/write mode
    
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000); // Set I2C to 100kHz (safe standard speed)

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      while (1);
    }
    
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(10, 10);
    display.print("Initializing...");
      display.display();
      delay(2000);
      
    // Initialize IMU
    LIS.begin(Wire, 0x19);
    delay(200);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_400HZ);  // Increased to 400Hz for very fast response
    LIS.setHighSolution(true);
    
    delay(200);
    Serial.println("IMU initialized");
      
    if (!max31865.begin(MAX31865_3WIRE)) {
      Serial.println("Failed to initialize MAX31865.");
      while (1);
    }
    Serial.println("MAX31865 Initialized");

    // Initialize RTC
    Serial.println("Initializing RTC...");
    if (!checkRTC()) {
      Serial.println("RTC not found! Check wiring and I2C address.");
      while(1) {
        delay(1000);
        if (checkRTC()) break;
      }
    }

    clearStopBit();
    clearCIF();

    // Parse compilation date and time (PC time during upload)
    // __DATE__ format: "MMM DD YYYY" (e.g., "Jul 15 2024")
    // __TIME__ format: "HH:MM:SS" (e.g., "14:30:25")
    char dateStr[] = __DATE__;
    char timeStr[] = __TIME__;
    
    Serial.printf("Raw __DATE__: '%s'\n", dateStr);
    Serial.printf("Raw __TIME__: '%s'\n", timeStr);
    
    // Parse month (MMM to number)
    byte month = 1;
    if (strstr(dateStr, "Jan")) month = 1;
    else if (strstr(dateStr, "Feb")) month = 2;
    else if (strstr(dateStr, "Mar")) month = 3;
    else if (strstr(dateStr, "Apr")) month = 4;
    else if (strstr(dateStr, "May")) month = 5;
    else if (strstr(dateStr, "Jun")) month = 6;
    else if (strstr(dateStr, "Jul")) month = 7;
    else if (strstr(dateStr, "Aug")) month = 8;
    else if (strstr(dateStr, "Sep")) month = 9;
    else if (strstr(dateStr, "Oct")) month = 10;
    else if (strstr(dateStr, "Nov")) month = 11;
    else if (strstr(dateStr, "Dec")) month = 12;
    
    // Parse day (positions 4-5 in "MMM DD YYYY")
    byte day = 0;
    if (dateStr[4] == ' ') {
      day = dateStr[5] - '0';  // Single digit day
    } else {
      day = (dateStr[4] - '0') * 10 + (dateStr[5] - '0');  // Double digit day
    }
    
    // Parse year (positions 7-10 in "MMM DD YYYY" - 11 character string)
    // Debug: print string length and each character
    int dateStrLen = strlen(dateStr);
    Serial.printf("__DATE__ length: %d\n", dateStrLen);
    Serial.printf("Year parsing - chars: '%c' '%c' '%c' '%c'\n", dateStr[7], dateStr[8], dateStr[9], dateStr[10]);
    
    int yearInt = 0;
    if (dateStrLen >= 11 && 
        dateStr[7] >= '0' && dateStr[7] <= '9' && 
        dateStr[8] >= '0' && dateStr[8] <= '9' && 
        dateStr[9] >= '0' && dateStr[9] <= '9' && 
        dateStr[10] >= '0' && dateStr[10] <= '9') {
      yearInt = (dateStr[7] - '0') * 1000 + (dateStr[8] - '0') * 100 + (dateStr[9] - '0') * 10 + (dateStr[10] - '0');
      Serial.printf("Year parsed successfully: %d\n", yearInt);
    } else {
      Serial.println("Error: Invalid year format in __DATE__");
      Serial.printf("String too short or invalid characters at positions 7-10\n");
      yearInt = 2024; // Default to 2024 if parsing fails
    }
    
    // Parse time components
    byte hour = (timeStr[0] - '0') * 10 + (timeStr[1] - '0');
    byte minute = (timeStr[3] - '0') * 10 + (timeStr[4] - '0');
    byte second = (timeStr[6] - '0') * 10 + (timeStr[7] - '0');
    
    // Convert to 2-digit year format (e.g., 2024 -> 24)
    byte year = yearInt - 2000;
    
    Serial.printf("Parsed values - Day: %d, Month: %d, Year: %d, Hour: %d, Minute: %d, Second: %d\n", 
                  day, month, yearInt, hour, minute, second);
    Serial.printf("PC compilation time: %04d/%02d/%02d %02d:%02d:%02d\n", 
                  2000 + year, month, day, hour, minute, second);
    
    // Check if RTC has been initialized before
    bool rtcInitialized = preferences.getBool("rtc_init", false);
    
    if (!rtcInitialized) {
      // Only set RTC to PC compilation time on first build
      Serial.println("First time setup - Setting RTC to PC compilation time...");
      setDateTime(day, month, year, hour, minute, second);
      
      // Verify the RTC was set correctly
      byte rtcHour, rtcMinute, rtcSecond;
      byte rtcDay, rtcMonth, rtcYear;
      if (readTime(rtcHour, rtcMinute, rtcSecond) && readDate(rtcDay, rtcMonth, rtcYear)) {
        Serial.printf("RTC verified: %04d/%02d/%02d %02d:%02d:%02d\n", 
                      2000 + rtcYear, rtcMonth, rtcDay, rtcHour, rtcMinute, rtcSecond);
        
        // Mark RTC as initialized
        preferences.putBool("rtc_init", true);
        Serial.println("RTC initialization flag set - will not reset on future boots");
      } else {
        Serial.println("Warning: Could not verify RTC time after setting");
      }
    } else {
      Serial.println("RTC already initialized - skipping time reset");
    }

    // Initialize shared data with RTC time
    sharedData.currentTime = getRTCTime();

    // Generate device name based on chip ID
    DEVICE_NAME = generateDeviceName();
    
    // Load device name from preferences (already initialized)
    String savedName = preferences.getString("dev_name", DEVICE_NAME);
    deviceDisplayName = savedName;
    Serial.printf("Loaded device name: %s\n", deviceDisplayName.c_str());

    // Print current time and status for debugging
    if (readTime(hour, minute, second) && readDate(day, month, year)) {
      Serial.printf("RTC time: %04d/%02d/%02d %02d:%02d:%02d\n", 
                    2000 + year, month, day, hour, minute, second);
    }

    // Setup buttons
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), button2ISR, FALLING);

    // Create queues
    tempQueue = xQueueCreate(10, sizeof(TempData));
    cmdQueue = xQueueCreate(10, sizeof(CmdData));
    imuQueue = xQueueCreate(10, sizeof(IMUData));

    // Create tasks
    xTaskCreate(tempTask, "TempTask", 2048, NULL, 1, &tempTaskHandle);
    xTaskCreate(displayTask, "DisplayTask", 2048, NULL, 1, &displayTaskHandle);
    xTaskCreate(bleTask, "BLETask", 4096, NULL, 1, &bleTaskHandle);
    xTaskCreate(cmdTask, "CmdTask", 2048, NULL, 1, &cmdTaskHandle);
    xTaskCreate(imuTask, "IMUTask", 2048, NULL, 1, &imuTaskHandle);

    // Initialize BLE mutex
    bleMutex = xSemaphoreCreateMutex();
    if (bleMutex == NULL) {
      Serial.println("Error creating BLE mutex");
      while(1);
    }
    
    // Start BLE by default
    Serial.println("Starting BLE by default...");
    startBLE();
    if (bleActive) {
      BLEDevice::startAdvertising();
      bleTransmitting = true;
      Serial.println("BLE started and advertising by default");
    } else {
      Serial.println("Failed to start BLE by default");
    }
  }
void handleButtons() {
    // Handle button 1 (display toggle)
    if (buttonPressed) {
        buttonPressed = false;
        
        // 唤醒屏幕（如果处于休眠状态）
        if (oledSleeping) {
            OLED_Wake();
            oledSleeping = false;
            lastMovementTime = millis(); // 重置活动计时器
        }
        
        currentDisplayMode = (currentDisplayMode == DISPLAY_TEMP) ? DISPLAY_DEVICE_NAME : DISPLAY_TEMP;
        Serial.println(currentDisplayMode == DISPLAY_TEMP ? "Displaying temperature" : "Displaying device info");
    }

    // Handle button 2 (BLE transmission toggle)
    if (button2Pressed) {
        button2Pressed = false;
      
        // 唤醒屏幕（如果处于休眠状态）
        if (oledSleeping) {
            OLED_Wake();
            oledSleeping = false;
            lastMovementTime = millis(); // 重置活动计时器
        }  
        
        unsigned long currentTime = millis();
        if (currentTime - lastButtonPressTime >= BUTTON_COOLDOWN) {
            lastButtonPressTime = currentTime;
            toggleBLE();
        }
    }
}

  void loop() {
    // Handle button 1 (display toggle)
    handleButtons();

    vTaskDelay(1);
  }


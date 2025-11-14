// Simple OTA Test Script
// Beeps numBeeps times, waits 10 seconds, and repeats
// Includes BLE OTA functionality for testing updates

// Set to 0 to disable BLE and test basic functionality
#define ENABLE_BLE 1

#if ENABLE_BLE
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#endif
#include <esp_ota_ops.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <mbedtls/md5.h>

// Buzzer pin (ESP32 GPIO)
const int buzzerPin = 38;

// Beep timing
const int beepDuration = 100;  // milliseconds
const int beepPause = 200;     // milliseconds between beeps
const int waitTime = 10000;    // 10 seconds between cycles
int numBeeps = 3;              // Number of beeps (can be updated via OTA)

#if ENABLE_BLE
// BLE UUIDs (must match ota_update.py)
#define UPDATE_SERVICE_UUID "5636340f-afc7-47b1-b0a8-15bcb9d7d29a6"
#define UPDATE_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea3"
#define VERSION_CHARACTERISTIC_UUID "c327b077-560f-46a1-8f35-b4ab0332fea2"

// BLE Server variables
BLEServer *ble_server;
BLEService *update_service;
BLECharacteristic *update_characteristic;
BLECharacteristic *version_characteristic;
bool is_device_connected = false;
#endif
bool otaEnabled = true;  // OTA always enabled for test script

// OTA State Management
enum OTAState {
  OTA_IDLE,
  OTA_PREPARING,
  OTA_RECEIVING,
  OTA_VALIDATING,
  OTA_FINALIZING,
  OTA_ERROR
};

OTAState otaState = OTA_IDLE;
esp_ota_handle_t ota_handle = 0;
const esp_partition_t *running_partition = NULL;
const esp_partition_t *update_partition = NULL;
size_t firmware_size = 0;
size_t bytes_received = 0;
uint8_t expected_md5[16] = {0};
uint8_t calculated_md5[16] = {0};
bool md5_received = false;
mbedtls_md5_context *md5_ctx = NULL;  // Initialize as pointer, allocate in setup()
bool md5_initialized = false;

const char* SOFTWARE_VERSION = "1.0.0";
const char* SOFTWARE_BUILD_DATE = __DATE__ " " __TIME__;

// Forward declarations
bool prepareForOTA();
void notifyUpdateProgress(int percentage);
bool validateFirmware();
void handleOTAChunk(uint8_t* data, size_t length);
void saveRollbackInfo();
void checkBootFailure();
bool rollbackOTAUpdate();

#if ENABLE_BLE
// BLE Server callbacks
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    is_device_connected = true;
    Serial.println("Device connected!");
  }

  void onDisconnect(BLEServer *pServer) {
    is_device_connected = false;
    Serial.println("Device disconnected!");
  }
};

// OTA Characteristic callbacks
class UpdateCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    int message_length = pCharacteristic->getLength();
    if (message_length > 0) {
      uint8_t* data = pCharacteristic->getData();
      
      // Handle OTA chunks
      if (otaState == OTA_RECEIVING) {
        handleOTAChunk(data, message_length);
        return;
      }
      
      // Handle commands
      String command = String((char*)data);
      command.trim();
      
      Serial.print("Received command: ");
      Serial.println(command);
      
      if (command == "CHECK_VERSION") {
        String versionInfo = "SW:" + String(SOFTWARE_VERSION) + "|Build:" + String(SOFTWARE_BUILD_DATE);
        version_characteristic->setValue(versionInfo.c_str());
        version_characteristic->notify();
        Serial.println("Version check requested");
      } else if (command == "PREPARE_UPDATE") {
        if (prepareForOTA()) {
          update_characteristic->setValue("UPDATE_PREPARED");
          update_characteristic->notify();
          Serial.println("Device prepared for update");
        } else {
          update_characteristic->setValue("UPDATE_BLOCKED");
          update_characteristic->notify();
          Serial.println("Update preparation blocked");
        }
      } else if (command == "START_UPDATE") {
        if (otaState == OTA_PREPARING) {
          esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
          if (err != ESP_OK) {
            Serial.printf("ERROR: esp_ota_begin failed: %s\n", esp_err_to_name(err));
            otaState = OTA_ERROR;
            update_characteristic->setValue("UPDATE_ERROR");
            update_characteristic->notify();
            return;
          }
          
          otaState = OTA_RECEIVING;
          update_characteristic->setValue("UPDATE_STARTED");
          update_characteristic->notify();
          Serial.println("OTA update started");
          notifyUpdateProgress(0);
        } else {
          update_characteristic->setValue("UPDATE_NOT_PREPARED");
          update_characteristic->notify();
        }
      } else if (command == "FINALIZE_UPDATE") {
        if (otaState == OTA_VALIDATING) {
          if (!validateFirmware()) {
            Serial.println("Firmware validation failed, attempting rollback...");
            otaState = OTA_ERROR;
            esp_ota_abort(ota_handle);
            ota_handle = 0;
            update_characteristic->setValue("UPDATE_VALIDATION_FAILED");
            update_characteristic->notify();
            
            // Attempt rollback
            if (rollbackOTAUpdate()) {
              Serial.println("Rollback successful, rebooting...");
              delay(1000);
              esp_restart();
            }
            return;
          }
          
          otaState = OTA_FINALIZING;
          esp_err_t err = esp_ota_end(ota_handle);
          if (err != ESP_OK) {
            Serial.printf("ERROR: esp_ota_end failed: %s\n", esp_err_to_name(err));
            otaState = OTA_ERROR;
            update_characteristic->setValue("UPDATE_ERROR");
            update_characteristic->notify();
            return;
          }
          
          err = esp_ota_set_boot_partition(update_partition);
          if (err != ESP_OK) {
            Serial.printf("ERROR: esp_ota_set_boot_partition failed: %s\n", esp_err_to_name(err));
            otaState = OTA_ERROR;
            update_characteristic->setValue("UPDATE_ERROR");
            update_characteristic->notify();
            return;
          }
          
          update_characteristic->setValue("UPDATE_COMPLETE");
          update_characteristic->notify();
          Serial.println("OTA update completed, rebooting...");
          notifyUpdateProgress(100);
          delay(1000);
          esp_restart();
        } else {
          update_characteristic->setValue("UPDATE_NOT_READY");
          update_characteristic->notify();
        }
      }
    }
  }
};
#endif

bool prepareForOTA() {
  running_partition = esp_ota_get_running_partition();
  if (running_partition == NULL) {
    Serial.println("ERROR: Cannot get running partition");
    return false;
  }
  
  // Determine update partition (opposite of current)
  if (running_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
  } else if (running_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  } else {
    update_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  }
  
  if (update_partition == NULL) {
    Serial.println("ERROR: Cannot find update partition");
    return false;
  }
  
  if (update_partition == running_partition) {
    Serial.println("ERROR: Cannot update running partition");
    return false;
  }
  
  // Save rollback information before starting update
  saveRollbackInfo();
  
  otaState = OTA_PREPARING;
  firmware_size = 0;
  bytes_received = 0;
  md5_received = false;
  memset(expected_md5, 0, 16);
  memset(calculated_md5, 0, 16);
  
  Serial.printf("Current partition: %s, Update partition: %s\n", 
                running_partition->label, update_partition->label);
  return true;
}

void notifyUpdateProgress(int percentage) {
#if ENABLE_BLE
  if (version_characteristic && is_device_connected) {
    String progressMsg = "UPDATE_PROGRESS:" + String(percentage);
    version_characteristic->setValue(progressMsg.c_str());
    version_characteristic->notify();
    Serial.printf("Update progress: %d%%\n", percentage);
  }
#else
  Serial.printf("Update progress: %d%%\n", percentage);
#endif
}

bool validateFirmware() {
  if (!md5_received) {
    Serial.println("ERROR: MD5 hash not received");
    return false;
  }
  
  if (memcmp(calculated_md5, expected_md5, 16) != 0) {
    Serial.println("ERROR: MD5 validation failed");
    return false;
  }
  
  Serial.println("MD5 validation successful");
  return true;
}

void handleOTAChunk(uint8_t* data, size_t length) {
  if (otaState != OTA_RECEIVING) {
    Serial.println("ERROR: Received chunk but not in RECEIVING state");
    return;
  }
  
  // Check if this is a metadata chunk
  if (length >= 4 && data[0] == 'S' && data[1] == 'I' && data[2] == 'Z' && data[3] == 'E') {
    String sizeStr = String((char*)data + 5);
    firmware_size = sizeStr.toInt();
    Serial.printf("Firmware size received: %d bytes\n", firmware_size);
    return;
  }
  
  if (length >= 3 && data[0] == 'M' && data[1] == 'D' && data[2] == '5') {
    if (length >= 20) {
      String md5Str = String((char*)data + 4);
      for (int i = 0; i < 16; i++) {
        char hex[3] = {md5Str[i*2], md5Str[i*2+1], 0};
        expected_md5[i] = strtol(hex, NULL, 16);
      }
      md5_received = true;
      Serial.println("MD5 hash received");
    }
    return;
  }
  
  // Regular firmware data chunk
  if (ota_handle == 0) {
    Serial.println("ERROR: OTA handle not initialized");
    otaState = OTA_ERROR;
    return;
  }
  
  // Initialize MD5 context for first chunk
  if (!md5_initialized) {
    if (md5_ctx == NULL) {
      md5_ctx = (mbedtls_md5_context*)malloc(sizeof(mbedtls_md5_context));
      if (md5_ctx == NULL) {
        Serial.println("ERROR: Failed to allocate MD5 context");
        otaState = OTA_ERROR;
        return;
      }
    }
    mbedtls_md5_init(md5_ctx);
    mbedtls_md5_starts(md5_ctx);
    md5_initialized = true;
  }
  
  // Update MD5 hash
  mbedtls_md5_update(md5_ctx, data, length);
  
  // Write chunk to OTA partition
  esp_err_t err = esp_ota_write(ota_handle, data, length);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to write OTA chunk: %s\n", esp_err_to_name(err));
    otaState = OTA_ERROR;
    if (md5_ctx != NULL) {
      mbedtls_md5_free(md5_ctx);
      free(md5_ctx);
      md5_ctx = NULL;
    }
    md5_initialized = false;
    return;
  }
  
  bytes_received += length;
  
  // Calculate and notify progress
  if (firmware_size > 0) {
    int progress = (bytes_received * 100) / firmware_size;
    notifyUpdateProgress(progress);
  }
  
  // Finalize MD5 if this is the last chunk
  if (firmware_size > 0 && bytes_received >= firmware_size) {
    if (md5_ctx != NULL) {
      mbedtls_md5_finish(md5_ctx, calculated_md5);
      mbedtls_md5_free(md5_ctx);
      free(md5_ctx);
      md5_ctx = NULL;
    }
    md5_initialized = false;
    Serial.println("Firmware reception complete, validating...");
    otaState = OTA_VALIDATING;
  }
}

// Save rollback information to NVS
void saveRollbackInfo() {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.println("ERROR: Failed to open NVS for rollback info");
    return;
  }
  
  // Save current partition subtype
  uint8_t current_subtype = running_partition->subtype;
  nvs_set_u8(nvs_handle, "rollback_subtype", current_subtype);
  
  // Save boot count (will be incremented on next boot)
  uint8_t boot_count = 0;
  nvs_get_u8(nvs_handle, "boot_count", &boot_count);
  boot_count++;
  nvs_set_u8(nvs_handle, "boot_count", boot_count);
  
  // Set rollback flag to false initially
  nvs_set_u8(nvs_handle, "rollback_flag", 0);
  
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  
  Serial.printf("Rollback info saved: subtype=%d, boot_count=%d\n", current_subtype, boot_count);
}

// Check for boot failures and rollback if needed
void checkBootFailure() {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.println("WARNING: Failed to open NVS for boot check (first boot?)");
    return;
  }
  
  uint8_t boot_count = 0;
  err = nvs_get_u8(nvs_handle, "boot_count", &boot_count);
  // If key doesn't exist, boot_count stays 0 (first boot) - that's fine
  
  Serial.printf("Boot count: %d\n", boot_count);
  
  // If boot count is high, it means we've been rebooting repeatedly (boot failure)
  if (boot_count > 3) {
    Serial.println("WARNING: High boot count detected, attempting rollback...");
    
    // Reset boot count first
    nvs_set_u8(nvs_handle, "boot_count", 0);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    // Attempt rollback (only if we're running from OTA partition)
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running && (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 || 
                    running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)) {
      if (rollbackOTAUpdate()) {
        Serial.println("Rollback successful, rebooting...");
        delay(1000);
        esp_restart();
      }
    } else {
      Serial.println("Not running from OTA partition, skipping rollback");
    }
    return;  // Exit early if rollback attempted
  } else {
    // Successful boot, reset boot count
    nvs_set_u8(nvs_handle, "boot_count", 0);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
  }
}

// Rollback to previous partition
bool rollbackOTAUpdate() {
  Serial.println("Attempting OTA rollback...");
  
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("ota", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    Serial.println("ERROR: Failed to open NVS for rollback");
    return false;
  }
  
  uint8_t rollback_subtype;
  err = nvs_get_u8(nvs_handle, "rollback_subtype", &rollback_subtype);
  if (err != ESP_OK) {
    Serial.println("ERROR: Failed to get rollback partition info");
    nvs_close(nvs_handle);
    return false;
  }
  
  // Find the rollback partition
  const esp_partition_t *rollback_partition = NULL;
  if (rollback_subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
    rollback_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  } else if (rollback_subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) {
    rollback_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
  } else {
    rollback_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
  }
  
  if (rollback_partition == NULL) {
    Serial.println("ERROR: Cannot find rollback partition");
    nvs_close(nvs_handle);
    return false;
  }
  
  // Set boot partition to rollback partition
  err = esp_ota_set_boot_partition(rollback_partition);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to set boot partition: %s\n", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return false;
  }
  
  // Clear rollback flag
  nvs_set_u8(nvs_handle, "rollback_flag", 0);
  nvs_set_u8(nvs_handle, "boot_count", 0);
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  
  Serial.printf("Rollback successful to partition: %s\n", rollback_partition->label);
  return true;
}

void setup() {
  // Start Serial immediately - test if it works
  Serial.begin(115200);
  delay(3000);  // Even longer delay for Serial to stabilize
  Serial.println("=== LOOP START ===");

  Serial.print("start TEST setup ");
  // Send multiple test messages
  for(int i = 0; i < 5; i++) {
    Serial.print("TEST ");
    Serial.println(i);
    delay(200);
  }
  
  Serial.println("\n\n=== OTA TEST STARTING ===");
  delay(500);
  Serial.println("OTA Test Script Starting...");
  delay(500);
  Serial.printf("Will beep %d times, wait 10 seconds, and repeat.\n", numBeeps);
  delay(500);
  Serial.println("Serial communication working!");
  delay(500);
  
  // Initialize NVS
  Serial.println("Initializing NVS...");
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("NVS needs erase, erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  Serial.println("NVS initialized");
  
  // Check for boot failures and rollback if needed
  Serial.println("Checking boot failure...");
  checkBootFailure();
  Serial.println("Boot check complete");
  
  // Setup buzzer pin
  Serial.println("Setting up buzzer pin...");
  delay(500);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  Serial.println("Buzzer pin ready");
  delay(500);
  
  // Test beep to confirm basic functionality
  Serial.println("Testing buzzer...");
  digitalWrite(buzzerPin, HIGH);
  delay(50);
  digitalWrite(buzzerPin, LOW);
  Serial.println("Buzzer test complete");
  delay(1000);
  
#if ENABLE_BLE
  // Setup BLE - do this LAST after everything else is working
  Serial.println("Setting up Bluetooth Low Energy...");
  delay(2000); // Very long delay before BLE init
  
  Serial.println("Calling BLEDevice::init()...");
  delay(500);
  
  // Initialize BLE - this is where crashes often happen
  BLEDevice::init("ESP32 Toilet");
  delay(500); // Wait after init
  Serial.println("BLE initialized successfully!");
  delay(500);
  
  Serial.println("Creating BLE server...");
  Serial.flush();
  delay(100);
  ble_server = BLEDevice::createServer();
  Serial.println("BLE server created");
  Serial.flush();
  delay(100);
  
  Serial.println("Setting server callbacks...");
  Serial.flush();
  ble_server->setCallbacks(new ServerCallbacks());
  Serial.println("Server callbacks set");
  Serial.flush();
  delay(100);
  
  // Create update service
  Serial.println("Creating update service...");
  Serial.flush();
  delay(100);
  update_service = ble_server->createService(UPDATE_SERVICE_UUID);
  Serial.println("Update service created");
  Serial.flush();
  delay(100);
  
  // Create update characteristic
  Serial.println("Creating update characteristic...");
  update_characteristic = update_service->createCharacteristic(
    UPDATE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  update_characteristic->setValue("OTA_READY");
  update_characteristic->setCallbacks(new UpdateCharacteristicCallbacks());
  Serial.println("Update characteristic created");
  
  // Create version characteristic
  Serial.println("Creating version characteristic...");
  version_characteristic = update_service->createCharacteristic(
    VERSION_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  String versionInfo = "SW:" + String(SOFTWARE_VERSION) + "|Build:" + String(SOFTWARE_BUILD_DATE);
  version_characteristic->setValue(versionInfo.c_str());
  Serial.println("Version characteristic created");
  
  // Start service
  Serial.println("Starting service...");
  update_service->start();
  Serial.println("Service started");
  
  // Start advertising
  Serial.println("Setting up advertising...");
  BLEAdvertising *pAdvertising = ble_server->getAdvertising();
  pAdvertising->addServiceUUID(UPDATE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  Serial.println("Starting advertising...");
  Serial.flush();
  delay(100);
  BLEDevice::startAdvertising();
  delay(200); // Wait after starting advertising
  Serial.println("Advertising started");
  Serial.flush();
  delay(100);
  
  Serial.print("ESP32 MAC ADDRESS: ");
  Serial.flush();
  delay(100);
  Serial.println(BLEDevice::getAddress().toString());
  Serial.flush();
  delay(100);
  Serial.println("BLE OTA service ready!");
  Serial.flush();
#else
  Serial.println("BLE disabled for testing");
#endif

  Serial.println("Setup complete. Starting beep cycle...");
  Serial.flush();
}

void loop() {
  // Beep numBeeps times
  Serial.println("=== LOOP START ===");
  Serial.printf("Beeping %d times...\n", numBeeps);
  Serial.flush();
  
  for (int i = 0; i < numBeeps; i++) {
    Serial.print("Beep ");
    Serial.println(i + 1);
    Serial.flush();
    digitalWrite(buzzerPin, HIGH);
    delay(beepDuration);
    digitalWrite(buzzerPin, LOW);
    delay(beepPause);
  }
  
  // Wait 10 seconds
  Serial.println("Waiting 10 seconds...");
  Serial.flush();
  for (int i = 0; i < 10; i++) {
    delay(1000);
    Serial.print(".");
    Serial.flush();
  }
  Serial.println();
  Serial.println("Starting next cycle...");
  Serial.flush();
}

#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <DualMAX14870MotorShield.h>

// Custom I2C instance
TwoWire myI2C = TwoWire(0);

// MCP23017 setup
Adafruit_MCP23X17 mcp;

// Button pins (MCP23017 side)
const int button1Pin = 7;
const int button2Pin = 15;

// LED pins
const int ledPins[] = {9, 13, 14, 10, 1, 6, 11, 12, 8, 0, 2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);

// M3 Motor pins
const uint8_t M3DIR_PIN = 2;     // GPIO2 (M3DIR)
const uint8_t M3PWM_PIN = 41;    // GPIO41 (M3PWM)
const uint8_t M3NEN_PIN = 47;    // GPIO47 (M3NEN)
const uint8_t M3NFAULT_PIN = 39; // GPIO39 (M3NFAULT)

// M3 Motor shield instance
DualMAX14870MotorShield motors3(M3DIR_PIN, M3PWM_PIN, M3DIR_PIN, M3PWM_PIN, M3NEN_PIN, M3NFAULT_PIN);

// Test variables
unsigned long testStartTime = 0;
bool directionForward = true;
int testCycle = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("M3 Motor Test Starting...");

  // ✅ Use GPIO6 for SDA and GPIO7 for SCL
  myI2C.begin(6, 7, 100000);

  // Init MCP23017 with custom I2C bus
  if (!mcp.begin_I2C(0x20, &myI2C)) {
    Serial.println("Error initializing MCP23017!");
    while (1);
  } else {
    Serial.println("MCP23017 Initialized Successfully.");
  }

  // Setup button pins on expander
  mcp.pinMode(button1Pin, INPUT_PULLUP);
  mcp.pinMode(button2Pin, INPUT_PULLUP);

  // Setup LED pins on expander
  for (int i = 0; i < totalLeds; i++) {
    mcp.pinMode(ledPins[i], OUTPUT);
  }

  // Enable M3 motor shield
  motors3.enableDrivers();
  Serial.println("M3 Motor Shield Enabled");

  Serial.println("Setup complete.");
  Serial.println("M3 Motor Test Instructions:");
  Serial.println("- Press Button 1: Start continuous M3 test");
  Serial.println("- Press Button 2: Stop M3 test");
  Serial.println("- M3 will run 5s forward, 5s reverse, repeat");
}

void loop() {
  // Read button states
  bool button1Pressed = (mcp.digitalRead(button1Pin) == LOW);
  bool button2Pressed = (mcp.digitalRead(button2Pin) == LOW);

  // Button 1: Start test
  if (button1Pressed) {
    Serial.println("Button 1 Pressed - Starting M3 Test");
    startM3Test();
  }
  
  // Button 2: Stop test
  if (button2Pressed) {
    Serial.println("Button 2 Pressed - Stopping M3 Test");
    stopM3Test();
  }

  // Run M3 test cycle
  runM3TestCycle();

  delay(100);
}

void startM3Test() {
  testStartTime = millis();
  testCycle = 0;
  directionForward = true;
  Serial.println("M3 Test Started");
  
  // Flash LEDs to indicate test start
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < totalLeds; j++) {
      mcp.digitalWrite(ledPins[j], HIGH);
    }
    delay(200);
    for (int j = 0; j < totalLeds; j++) {
      mcp.digitalWrite(ledPins[j], LOW);
    }
    delay(200);
  }
}

void stopM3Test() {
  motors3.setM1Speed(0);
  testStartTime = 0;
  Serial.println("M3 Test Stopped");
  
  // Turn off all LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
}

void runM3TestCycle() {
  if (testStartTime == 0) return; // Test not started
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - testStartTime;
  
  // Calculate which 5-second period we're in
  int period = (elapsedTime / 5000) % 2; // 0 = forward, 1 = reverse
  
  // Change direction if needed
  if (period == 0 && !directionForward) {
    directionForward = true;
    motors3.setM1Speed(400); // Forward
    Serial.println("M3 Direction: FORWARD");
    testCycle++;
    
    // Flash first LED for forward
    mcp.digitalWrite(ledPins[0], HIGH);
    delay(100);
    mcp.digitalWrite(ledPins[0], LOW);
  } else if (period == 1 && directionForward) {
    directionForward = false;
    motors3.setM1Speed(-400); // Reverse
    Serial.println("M3 Direction: REVERSE");
    
    // Flash second LED for reverse
    mcp.digitalWrite(ledPins[1], HIGH);
    delay(100);
    mcp.digitalWrite(ledPins[1], LOW);
  }
  
  // Check for motor fault
  if (motors3.getFault()) {
    Serial.println("M3 FAULT DETECTED!");
    stopM3Test();
    
    // Flash all LEDs for fault
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < totalLeds; j++) {
        mcp.digitalWrite(ledPins[j], HIGH);
      }
      delay(200);
      for (int j = 0; j < totalLeds; j++) {
        mcp.digitalWrite(ledPins[j], LOW);
      }
      delay(200);
    }
  }
  
  // Print status every 2 seconds
  static unsigned long lastStatusTime = 0;
  if (currentTime - lastStatusTime > 2000) {
    lastStatusTime = currentTime;
    Serial.print("M3 Test Cycle: ");
    Serial.print(testCycle);
    Serial.print(", Direction: ");
    Serial.print(directionForward ? "FORWARD" : "REVERSE");
    Serial.print(", Elapsed: ");
    Serial.print(elapsedTime / 1000);
    Serial.println("s");
  }
}
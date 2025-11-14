#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// Custom I2C instance
TwoWire myI2C = TwoWire(0);

// MCP23017 setup
Adafruit_MCP23X17 mcp;

// Button pins (MCP23017 side)
const int button1Pin = 7;   // MCP23017 pin 7
const int button2Pin = 15;  // MCP23017 pin 15

// Microswitch pins (ESP32 side)
const int microswitchClosePin = 19;   // GPIO42 (RXD0) //36
const int microswitchOpenPin = 20;    // GPIO43 (TXD0) //37

// Buzzer pin
const int buzzerPin = 38;        // GPIO38 (BUZ)

// LED pins for visual feedback
const int ledPins[] = {9, 13, 14, 10, 1, 6, 11, 12, 8, 0, 2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Microswitch Test Starting...");

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

  // Setup microswitch pins
  pinMode(microswitchClosePin, INPUT_PULLUP);
  pinMode(microswitchOpenPin, INPUT_PULLUP);
  
  // Setup buzzer pin
  pinMode(buzzerPin, OUTPUT);

  Serial.println("Setup complete.");
  Serial.println("Microswitch Test Instructions:");
  Serial.println("- Close Switch 1 (Close): Buzzer beeps ONCE");
  Serial.println("- Close Switch 2 (Open): Buzzer beeps TWICE");
  Serial.println("- Press Button 1: Run LED test sequence");
  Serial.println("- Press Button 2: Test all switches");

  // Test buzzer immediately
  Serial.println("Testing buzzer...");
  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzerPin, HIGH);
    Serial.println("Buzzer ON");
    delay(1000);
    digitalWrite(buzzerPin, LOW);
    Serial.println("Buzzer OFF");
    delay(1000);
  }
  Serial.println("Buzzer test complete");

  runLEDTestSequence();
  runLEDTestSequence();
}

void loop() {
  // Read button states
  bool button1Pressed = (mcp.digitalRead(button1Pin) == LOW);
  bool button2Pressed = (mcp.digitalRead(button2Pin) == LOW);

  // Read microswitch states
  bool switch1Closed = (digitalRead(microswitchClosePin) == LOW);
  bool switch2Closed = (digitalRead(microswitchOpenPin) == LOW);

  // Handle microswitch 1 (Close switch) - 1 beep
  if (switch1Closed) {
    Serial.println("Switch 1 (Close) - CLOSED");
    beepBuzzer(1);
    flashLEDs(1); // Flash first LED
    delay(500); // Prevent multiple triggers
  }

  // Handle microswitch 2 (Open switch) - 2 beeps
  if (switch2Closed) {
    Serial.println("Switch 2 (Open) - CLOSED");
    beepBuzzer(2);
    flashLEDs(2); // Flash first two LEDs
    delay(500); // Prevent multiple triggers
  }

  // Button 1: LED test sequence
  if (button1Pressed) {
    Serial.println("Button 1 Pressed - Running LED Test Sequence");
    flashLEDs(1);
  }
  
  // Button 2: Test all switches
  if (button2Pressed) {
    Serial.println("Button 2 Pressed - Testing All Switches");
    flashLEDs(2);
    //testAllSwitches();
  }

  delay(50); // Small delay to prevent button bounce
}

void beepBuzzer(int beepCount) {
  for (int i = 0; i < beepCount; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    delay(200);
  }
}

void flashLEDs(int ledCount) {
  // Turn off all LEDs first
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
  
  // Flash the specified number of LEDs
  for (int i = 0; i < ledCount && i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], HIGH);
  }
  delay(1000);
  
  // Turn off LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
}

void runLEDTestSequence() {
  Serial.println("=== LED Test Sequence ===");
  
  // Turn off all LEDs first
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
  delay(200);
  
  // Test each LED individually
  for (int ledIndex = 0; ledIndex < totalLeds; ledIndex++) {
    Serial.print("Testing LED ");
    Serial.print(ledIndex);
    Serial.print(" (MCP pin ");
    Serial.print(ledPins[ledIndex]);
    Serial.println(")");
    
    mcp.digitalWrite(ledPins[ledIndex], HIGH);
    delay(100);
    mcp.digitalWrite(ledPins[ledIndex], LOW);
    delay(100);
  }
  
  Serial.println("LED Test Sequence Complete!");
}

void testAllSwitches() {
  Serial.println("=== Switch Test ===");
  
  // Turn off all LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
  
  // Test Switch 1
  Serial.println("Testing Switch 1 (Close) - Please close the switch");
  Serial.println("Waiting for Switch 1...");
  
  while (digitalRead(microswitchClosePin) == HIGH) {
    delay(100);
  }
  
  Serial.println("Switch 1 CLOSED!");
  beepBuzzer(1);
  flashLEDs(1);
  delay(1000);
  
  // Test Switch 2
  Serial.println("Testing Switch 2 (Open) - Please close the switch");
  Serial.println("Waiting for Switch 2...");
  
  while (digitalRead(microswitchOpenPin) == HIGH) {
    delay(100);
  }
  
  Serial.println("Switch 2 CLOSED!");
  beepBuzzer(2);
  flashLEDs(2);
  delay(1000);
  
  Serial.println("Switch Test Complete!");
}
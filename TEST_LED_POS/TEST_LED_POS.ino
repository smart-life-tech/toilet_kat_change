#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// Custom I2C instance
TwoWire myI2C = TwoWire(0);

// MCP23017 setup
Adafruit_MCP23X17 mcp;

// Button pins (MCP23017 side)
const int button1Pin = 7;
const int button2Pin = 15;

// Buzzer pin (ESP32 GPIO)
const int buzzerPin = 38;

// LED pins (MCP23017 side) - same array as Test_LEDs.ino
const int ledPins[] = {9, 13, 14, 10, 1, 6, 11, 12, 8, 0,  2, 3, 4, 5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);

// LED state tracking
bool allLEDsOn = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("LED Position Test Starting...");

  // Setup buzzer pin
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // ✅ Use GPIO6 for SDA and GPIO7 for SCL
  myI2C.begin(6, 7, 100000);

  // Init MCP23017 with custom I2C bus
  if (!mcp.begin_I2C(0x20, &myI2C)) {
    Serial.println("Error initializing MCP23017!");
    Serial.println("Buzzer will beep every second to indicate MCP failure.");
    while (1) {
      beepBuzzer();
      delay(1000);
    }
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

  Serial.println("Setup complete.");
  Serial.println("This test will flash each LED a number of times equal to its position in the array.");
  Serial.println("Array positions: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13");
  Serial.println("Press Button 1 to start the position test sequence.");
  Serial.println("Press Button 2 to toggle all LEDs ON/OFF.");
}

void loop() {
  // Read button states
  bool button1Pressed = (mcp.digitalRead(button1Pin) == LOW);
  bool button2Pressed = (mcp.digitalRead(button2Pin) == LOW);

  if (button1Pressed) {
    Serial.println("Button 1 Pressed - Starting LED Position Test");
    runLEDPositionTest();
  }
  
  if (button2Pressed) {
    if (allLEDsOn) {
      Serial.println("Button 2 Pressed - Turning All LEDs OFF");
      turnAllLEDsOff();
    } else {
      Serial.println("Button 2 Pressed - Turning All LEDs ON");
      turnAllLEDsOn();
    }
  }

  delay(100); // Small delay to prevent button bounce
}

void runLEDPositionTest() {
  Serial.println("=== LED Position Test ===");
  
  // Turn off all LEDs first
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
  }
  delay(1000);
  
  // Flash each LED according to its array position
  for (int ledIndex = 0; ledIndex < totalLeds; ledIndex++) {
    Serial.print("Testing LED at array position ");
    Serial.print(ledIndex);
    Serial.print(" (MCP pin ");
    Serial.print(ledPins[ledIndex]);
    Serial.println(")");
    
    // Flash the LED a number of times equal to its position
    for (int flashCount = 0; flashCount <= ledIndex; flashCount++) {
      mcp.digitalWrite(ledPins[ledIndex], HIGH);
      delay(300);
      mcp.digitalWrite(ledPins[ledIndex], LOW);
      delay(300);
    }
    
    delay(500); // Pause between LEDs
  }
  
  Serial.println("LED Position Test Complete!");
  delay(2000);
}

void turnAllLEDsOn() {
  Serial.println("=== Turning All LEDs ON ===");
  
  // Turn on all LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], HIGH);
    Serial.print("LED ");
    Serial.print(i);
    Serial.print(" (MCP pin ");
    Serial.print(ledPins[i]);
    Serial.println(") ON");
  }
  
  allLEDsOn = true;
  Serial.println("All LEDs are now ON!");
  Serial.println("Press Button 1 to run position test, or Button 2 again to turn them off.");
}

void turnAllLEDsOff() {
  Serial.println("=== Turning All LEDs OFF ===");
  
  // Turn off all LEDs
  for (int i = 0; i < totalLeds; i++) {
    mcp.digitalWrite(ledPins[i], LOW);
    Serial.print("LED ");
    Serial.print(i);
    Serial.print(" (MCP pin ");
    Serial.print(ledPins[i]);
    Serial.println(") OFF");
  }
  
  allLEDsOn = false;
  Serial.println("All LEDs are now OFF!");
  Serial.println("Press Button 1 to run position test, or Button 2 again to turn them on.");
}

void beepBuzzer() {
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
}
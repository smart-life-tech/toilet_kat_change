#include <Wire.h>
#include <Adafruit_MCP23X17.h>

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

// Buzzer pin (ESP32 side)
const int buzzerPin = 38;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("MCP23017 Diagnostic Test Starting...");

  // Test I2C communication
  Serial.println("Testing I2C communication...");
  
  // ✅ Use GPIO6 for SDA and GPIO7 for SCL
  myI2C.begin(6, 7, 100000);
  
  // Scan for I2C devices
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++ ) {
    myI2C.beginTransmission(address);
    error = myI2C.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check wiring: SDA=GPIO6, SCL=GPIO7");
    Serial.println("Check MCP23017 power and address jumpers");
    while(1);
  }

  // Try to initialize MCP23017
  Serial.println("Attempting to initialize MCP23017...");
  if (!mcp.begin_I2C(0x20, &myI2C)) {
    Serial.println("ERROR: Could not initialize MCP23017!");
    Serial.println("Possible issues:");
    Serial.println("1. Wrong I2C address (should be 0x20)");
    Serial.println("2. MCP23017 not powered");
    Serial.println("3. I2C wiring issues");
    Serial.println("4. MCP23017 faulty");
    while(1);
  } else {
    Serial.println("SUCCESS: MCP23017 initialized!");
  }

  // Setup pins
  Serial.println("Setting up pins...");
  mcp.pinMode(button1Pin, INPUT_PULLUP);
  mcp.pinMode(button2Pin, INPUT_PULLUP);
  
  for (int i = 0; i < totalLeds; i++) {
    mcp.pinMode(ledPins[i], OUTPUT);
  }
  
  // Setup buzzer
  pinMode(buzzerPin, OUTPUT);
  
  Serial.println("Setup complete. Starting tests...");
  
  // Test buzzer first (ESP32 direct)
  Serial.println("Testing buzzer (ESP32 direct)...");
  for (int i = 0; i < 3; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    delay(200);
  }
  Serial.println("Buzzer test complete");
  
  // Test LEDs
  Serial.println("Testing LEDs...");
  for (int cycle = 0; cycle < 3; cycle++) {
    Serial.print("LED Cycle ");
    Serial.print(cycle + 1);
    Serial.println(" of 3");
    
    // Turn on all LEDs
    for (int i = 0; i < totalLeds; i++) {
      mcp.digitalWrite(ledPins[i], HIGH);
    }
    delay(500);
    
    // Turn off all LEDs
    for (int i = 0; i < totalLeds; i++) {
      mcp.digitalWrite(ledPins[i], LOW);
    }
    delay(500);
  }
  Serial.println("LED test complete");
}

void loop() {
  // Read button states
  bool button1Pressed = (mcp.digitalRead(button1Pin) == LOW);
  bool button2Pressed = (mcp.digitalRead(button2Pin) == LOW);
  
  // Debug output every 2 seconds
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 2000) {
    lastDebugTime = millis();
    Serial.print("Button1: ");
    Serial.print(mcp.digitalRead(button1Pin));
    Serial.print(" Button2: ");
    Serial.println(mcp.digitalRead(button2Pin));
  }
  
  // Button 1 pressed
  if (button1Pressed) {
    Serial.println("Button 1 PRESSED!");
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    
    // Flash first LED
    mcp.digitalWrite(ledPins[0], HIGH);
    delay(1000);
    mcp.digitalWrite(ledPins[0], LOW);
  }
  
  // Button 2 pressed
  if (button2Pressed) {
    Serial.println("Button 2 PRESSED!");
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    delay(200);
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    
    // Flash first two LEDs
    mcp.digitalWrite(ledPins[0], HIGH);
    mcp.digitalWrite(ledPins[1], HIGH);
    delay(1000);
    mcp.digitalWrite(ledPins[0], LOW);
    mcp.digitalWrite(ledPins[1], LOW);
  }
  
  delay(50);
}
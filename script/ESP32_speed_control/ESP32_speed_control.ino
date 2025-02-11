#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 95   // Adjusted minimum pulse length (maps to -90 degrees)
#define SERVO_MAX 510  // Adjusted maximum pulse length (maps to +90 degrees)
#define NUM_SERVOS 16  // Number of servo channels

// Arrays to track current servo positions and target positions
int currentPositions[NUM_SERVOS];
int targetPositions[NUM_SERVOS];
int speeds[NUM_SERVOS]; // Array to track speed for each servo

void setup() {
  Serial.begin(115200); // Use a higher baud rate for ESP32
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz

  // Initialize servo positions and speeds
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPositions[i] = 0;  // Default position at 0 degrees
    targetPositions[i] = 0;   // Default target position at 0 degrees
    speeds[i] = 5;            // Default speed (adjust as needed)
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read incoming command
    handleCommands(command);
  }

  // Smoothly move servos toward their target positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (currentPositions[i] != targetPositions[i]) {
      // Calculate step size based on speed
      int step = speeds[i];
      if (abs(targetPositions[i] - currentPositions[i]) < step) {
        currentPositions[i] = targetPositions[i]; // Close enough, snap to target
      } else {
        // Move incrementally toward target
        if (currentPositions[i] < targetPositions[i]) {
          currentPositions[i] += step;
        } else {
          currentPositions[i] -= step;
        }
      }

      // Convert position to pulse width and set servo
      int pulseWidth = map(currentPositions[i], -90, 90, SERVO_MIN, SERVO_MAX);
      pwm.setPWM(i, 0, pulseWidth);

      // Print current position and pulse width
      Serial.printf("Channel: %d, Position: %d degrees, Pulse Width: %d\n", i, currentPositions[i], pulseWidth);
    }
  }

  delay(20); // Adjust delay for smoother movement
}

void handleCommands(String command) {
  // Command format: "ch1,pos1,speed1;ch2,pos2,speed2;..." (e.g., "13,45,10;14,-30,5")
  int start = 0;
  while (start < command.length()) {
    int semicolonIndex = command.indexOf(';', start);
    if (semicolonIndex == -1) semicolonIndex = command.length();

    String subCommand = command.substring(start, semicolonIndex);
    int firstComma = subCommand.indexOf(',');
    int secondComma = subCommand.indexOf(',', firstComma + 1);
    if (firstComma > 0 && secondComma > firstComma) {
      int channel = subCommand.substring(0, firstComma).toInt();
      int position = subCommand.substring(firstComma + 1, secondComma).toInt();
      int speed = subCommand.substring(secondComma + 1).toInt();

      if (channel >= 0 && channel < NUM_SERVOS && position >= -90 && position <= 90 && speed > 0) {
        // Update target position and speed
        targetPositions[channel] = position;
        speeds[channel] = speed;

        // Print command acknowledgment
        Serial.printf("Set Channel: %d, Target Position: %d degrees, Speed: %d\n", channel, position, speed);
      } else {
        Serial.println("Invalid command or out-of-range values.");
      }
    }
    start = semicolonIndex + 1;
  }
}

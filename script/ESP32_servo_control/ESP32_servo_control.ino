#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 95  // Minimum pulse length
#define SERVO_MAX 510  // Maximum pulse length
#define NUM_SERVOS 16  // Number of servo channels

// Array to track current servo positions
int currentPositions[NUM_SERVOS];

void setup() {
  Serial.begin(115200); // Use a higher baud rate for ESP#define SERVO_MIN 95  // Minimum pulse length
#define SERVO_MAX 510  // Maximum pulse length
#define NUM_SERVOS 16  // Number of servo channels32
  pwm.begin();
  pwm.setPWMFreq(50); // Set frequency to 50 Hz

  // Initialize servo positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPositions[i] = 0; // Default position at 0 degrees
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read incoming command
    handleCommand(command);
  }
}

void handleCommand(String command) {
  // Command format: "ch,pos" (e.g., "13,45" for channel 13 to 45 degrees)
  int commaIndex = command.indexOf(',');
  if (commaIndex > 0) {
    int channel = command.substring(0, commaIndex).toInt();
    int position = command.substring(commaIndex + 1).toInt();

    if (channel >= 0 && channel < NUM_SERVOS && position >= -90 && position <= 90) {
      int pulseWidth = map(position, -90, 90, SERVO_MIN, SERVO_MAX);
      pwm.setPWM(channel, 0, pulseWidth);

      // Update current position
      currentPositions[channel] = position;

      // Print current position and pulse width
      Serial.printf("Channel: %d, Position: %d degrees, Pulse Width: %d\n", channel, position, pulseWidth);
    } else {
      Serial.println("Invalid command or out-of-range values.");
    }
  } else {
    Serial.println("Invalid command format.");
  }
}

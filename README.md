#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Servo.h>
// Pin definitions for IR sensors
#define LEFT_IR_PIN 8
#define MIDDLE_IR_PIN 9
#define RIGHT_IR_PIN 10
Servo myservo;
// Motor driver pins
#define IN1 2 // Left motor forward
#define IN2 3 // Left motor backward
#define IN3 5 // Right motor forward
#define IN4 4// Right motor backward
#define pump 13

// Create an instance of the MLX90614 object for flame temperature sensing
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Setup the motors and sensors
void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("Flame Detection and Movement Control");

  // Initialize IR sensor pins
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(MIDDLE_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  // Initialize motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(pump, OUTPUT);

  myservo.attach(12);
  myservo.write(90);

  // Initialize MLX90614 sensor
  Wire.begin();  // Initialize I2C bus
  if (!mlx.begin(0x5A)) {
    Serial.println("Error initializing MLX90614. Check connections!");
    while (true); // Stop if initialization fails
  }
}
void sweepServo() {
  for (int pos = 50; pos <= 130; pos += 1) {
    myservo.write(pos);
    delay(10);
  }
  for (int pos = 130; pos >= 50; pos -= 1) {
    myservo.write(pos);
    delay(10);
  }
}

void put_off_fire() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(pump, HIGH);
  sweepServo();  // Sweep the servo while extinguishing the fire
  digitalWrite(pump, LOW);
}

// Main loop
void loop() {
  float objectTemp = mlx.readObjectTempC();
  Serial.print("Flame Temperature: ");
  Serial.print(objectTemp);
  Serial.println(" °C");
  if (objectTemp > 37.0) {
    Serial.println("Temperature exceeds 31°C. Stopping motors.");
    put_off_fire();
    return;
  }
  // Read the values from the IR sensors (LOW = detected fire)
  int leftSensorValue = digitalRead(LEFT_IR_PIN);
  int middleSensorValue = digitalRead(MIDDLE_IR_PIN);
  int rightSensorValue = digitalRead(RIGHT_IR_PIN);

  // Print the sensor values to the Serial Monitor
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print("\tMiddle Sensor: ");
  Serial.print(middleSensorValue);
  Serial.print("\tRight Sensor: ");
  Serial.println(rightSensorValue);

  // Check which sensor is detecting fire (assuming LOW means fire detected)
  if (leftSensorValue == LOW) {
    Serial.println("Left sensor is detecting fire!");
    moveTowardsFire("left");
  }
  else if (middleSensorValue == LOW) {
    Serial.println("Middle sensor is detecting fire!");
    moveTowardsFire("middle");
  }
  else if (rightSensorValue == LOW) {
    Serial.println("Right sensor is detecting fire!");
    moveTowardsFire("right");
  }
  else {
    Serial.println("No fire detected.");
    stopMovement();
  }
  // Add a short delay for better readability of the output
  delay(500);  // Delay for 0.5 seconds
}

// Function to move the robot towards the fire
void moveTowardsFire(String direction) {
  if (direction == "left") {
    // Move the robot left
    digitalWrite(IN1, LOW); // Left motor forward
    digitalWrite(IN2, LOW);  // Left motor off
    digitalWrite(IN3, LOW); // Right motor forward
    digitalWrite(IN4, HIGH);  // Right motor off
    Serial.println("Moving left to the fire!");
  }
  else if (direction == "middle") {
    // Move the robot forward
    digitalWrite(IN1, LOW); // Left motor forward
    digitalWrite(IN2, HIGH);  // Left motor off backward
    digitalWrite(IN3, LOW); // Right motor forward
    digitalWrite(IN4, HIGH);  // Right motor off backward
    Serial.println("Moving forward to the fire!");
  }
  else if (direction == "right") {
    // Move the robot right
    digitalWrite(IN1, LOW); // Left motor forward
    digitalWrite(IN2, HIGH);  // Left motor off
    digitalWrite(IN3, LOW); // Right motor forward
    digitalWrite(IN4, LOW);  // Right motor off
    Serial.println("Moving right to the fire!");
  }
  
  // Add a small delay to move toward the fire
  delay(500); // Move for 1 second
  stopMovement(); // Stop after moving toward the fire
}

// Function to stop all motor movement
void stopMovement() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Stopping movement!");
}

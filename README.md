# Line Following Robot

This project is an Arduino-based line-following robot using IR sensors and DC motors. The robot moves forward, turns left or right, or stops based on the signals received from its infrared sensors.

## Components Used
- **Arduino Board**
- **L298N Motor Driver Module**
- **DC Motors** (2x)
- **IR Sensors** (2x - Left and Right)
- **Wheels and Chassis**
- **Power Supply (Battery pack)**

## Circuit Connections
### Motor A:
- ENA -> Pin **11**
- IN1 -> Pin **10**
- IN2 -> Pin **9**

### Motor B:
- ENB -> Pin **6**
- IN3 -> Pin **8**
- IN4 -> Pin **7**

### IR Sensors:
- Left IR Sensor -> Pin **2**
- Right IR Sensor -> Pin **3**

## Code Explanation
The robot operates based on the input received from the IR sensors:
1. If both sensors detect the line (**LOW**), the robot moves **forward**.
2. If only the left sensor detects the line (**LOW, HIGH**), the robot **turns right**.
3. If only the right sensor detects the line (**HIGH, LOW**), the robot **turns left**.
4. If neither sensor detects the line (**HIGH, HIGH**), the robot **stops**.

## Functions in the Code
- `moveForward()`: Moves the robot forward.
- `turnRight()`: Turns the robot to the right.
- `turnLeft()`: Turns the robot to the left.
- `stopMotors()`: Stops the robot.

## How to Use
1. Connect the components as per the circuit diagram.
2. Upload the provided code to the Arduino board.
3. Place the robot on a track with a black line on a white surface.
4. Power the robot and observe its movement.

## Code
```cpp
// Motor A
const int ENA = 11;
const int IN1 = 10;
const int IN2 = 9;
#define s 100 // Base speed
#define t 130 // Turning speed

// Motor B
const int ENB = 6;
const int IN3 = 8;
const int IN4 = 7;

// IR Sensors
const int IRSensorLeft = 2;
const int IRSensorRight = 3;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IRSensorLeft, INPUT);
  pinMode(IRSensorRight, INPUT);
  Serial.begin(9600);
}

void loop() {
  bool leftSensor = digitalRead(IRSensorLeft);
  bool rightSensor = digitalRead(IRSensorRight);
  Serial.print("Left Sensor: ");
  Serial.print(leftSensor);
  Serial.print(" | Right Sensor: ");
  Serial.println(rightSensor);

  if (leftSensor == LOW && rightSensor == LOW) {
    moveForward();
  } else if (leftSensor == LOW && rightSensor == HIGH) {
    turnRight();
  } else if (leftSensor == HIGH && rightSensor == LOW) {
    turnLeft();
  } else {
    stopMotors();
  }

  delay(100);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, s);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, s);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, t);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, t);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, t);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, t);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
```

## License
This project is open-source and free to use.


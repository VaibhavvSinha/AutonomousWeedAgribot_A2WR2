#define ENC_IN_RIGHT_A 2
#define ENC_IN_LEFT_A 3

const float pi = 3.14;
const float wr = 0.139;  // wheel radius in meters
const int n_ticks_pr = 700;  // number of pulses per revolution

int pwm_straight = 100;
int pwm_turn_fast = 60;
int pwm_turn_slow = 60;

volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;

// Define parameters
const float target_distance = 5.0;  // Distance to move in each straight path
const int turn_delay = 10000;  // Delay after turning (adjust as needed)

void setup() {
  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);

  // Set initial conditions for the robot
  setLeftWheelDirection(true);  // Set initial direction for the left wheel (true for forward)
  setRightWheelDirection(true); // Set initial direction for the right wheel (true for forward)
  stopRobot();  // Stop the robot initially

  // Allow some time for the robot to stabilize
  delay(1000);  // Adjust the delay duration as needed
}
int i=0;
void loop() {
  
  while(i==0){
    moveStraight(target_distance);  // Move straight for the specified distance
  delay(1000);  // Delay before turning
  turnRight();  // Turn right
  delay(1000);  // Delay before moving straight again
  moveStraight(target_distance);  // Move straight for the specified distance
  delay(1000);  // Delay before turning left
  turnLeft();  // Turn left
  stopRobot();  // Stop the robot after turning left
  i=1;
}
  }
  

void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

void left_wheel_pulse() {
  left_wheel_pulse_count++;
}

// Function to move the robot straight for a specified distance
void moveStraight(float distance) {
  resetWheelCounters();  // Reset wheel counters
  setLeftWheelDirection(true);  // Set initial direction for the left wheel (true for forward)
  setRightWheelDirection(true);
  analogWrite(9, pwm_straight);  // Set speed for the left wheel
  analogWrite(10, pwm_straight);  // Set speed for the right wheel
  while (true) {
    // Calculate distance traveled for each wheel
    float left_distance = (2.0 * pi * wr * left_wheel_pulse_count) / n_ticks_pr;
    float right_distance = (2.0 * pi * wr * right_wheel_pulse_count) / n_ticks_pr;

    // Average the distances to get the overall distance
    float average_distance = (left_distance + right_distance) / 2.0;

    // Check if the robot has traveled the specified distance
    if (average_distance >= distance) {
      stopRobot();  // Stop the robot
      break;
    }
  }
}

// Function to turn the robot right
void turnRight() {
  setLeftWheelDirection(true);  // Set left wheel to forward
  setRightWheelDirection(false);  // Set right wheel to forward
  analogWrite(9, pwm_turn_slow);  // Set speed for the left wheel (faster)
  analogWrite(10, pwm_turn_fast);  // Set speed for the right wheel (slower)
  delay(turn_delay);
  stopRobot();
}

// Function to turn the robot left
void turnLeft() {
  setLeftWheelDirection(false);  // Set left wheel to forward
  setRightWheelDirection(true);  // Set right wheel to forward
  analogWrite(9, pwm_turn_fast);  // Set speed for the left wheel (slower)
  analogWrite(10, pwm_turn_slow);  // Set speed for the right wheel (faster)
  delay(turn_delay);
  stopRobot();
}

// Function to set the direction of the left wheel (true for forward, false for backward)
void setLeftWheelDirection(bool forward) {
  digitalWrite(5, forward ? LOW : HIGH);
}

// Function to set the direction of the right wheel (true for forward, false for backward)
void setRightWheelDirection(bool forward) {
  digitalWrite(6, forward ? LOW : HIGH);
}

// Function to stop the robot
void stopRobot() {
  analogWrite(9, 0);  // left
  analogWrite(10, 0);  // right
}

// Function to reset wheel pulse counters
void resetWheelCounters() {
  left_wheel_pulse_count = 0;
  right_wheel_pulse_count = 0;
}

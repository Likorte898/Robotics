/**
 * Advanced Line Follower for Complex Paths
 * Features: PID control, intersection handling, dynamic thresholding, speed adaptation
 * Tinkercad Compatible
 */

// Motor Control Pins
#define AIN1 13
#define AIN2 12
#define BIN1 8
#define BIN2 9
#define PWMA 11
#define PWMB 10
#define STBY 5

// Sensor Configuration
const int sensorPins[] = {A0, A1, A2, A3, A4}; // Left to Right
int sensorValues[5];
int sensorThresholds[5];
bool sensorState[5];

// PID Control
float Kp = 12.0, Ki = 0.02, Kd = 3.0;
int baseSpeed = 160;
int maxSpeed = 255;
int previousError = 0;
int integral = 0;
int derivative = 0;

// Robot States
#define STATE_FOLLOWING 0
#define STATE_INTERSECTION 1
#define STATE_LOST 2
int robotState = STATE_FOLLOWING;

// Timing Control
unsigned long intersectionTimer = 0;
const int intersectionDelay = 500;

// Button Control
#define BUTTON_PIN 7
bool robotActive = false;

void setup() {
  // Motor Pin Initialization
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Button Setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Automatic Threshold Calibration
  autoCalibrate();
  
  Serial.begin(9600);
}

void autoCalibrate() {
  // Dynamic threshold calibration
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  
  for(int i = 0; i < 5; i++) {
    int minVal = 1023;
    int maxVal = 0;
    
    // Quick calibration sweep
    for(int j = 0; j < 100; j++) {
      int val = analogRead(sensorPins[i]);
      minVal = min(minVal, val);
      maxVal = max(maxVal, val);
      delay(5);
    }
    sensorThresholds[i] = (minVal + maxVal) / 2;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  handleButton();
  readSensors();
  
  if(robotActive) {
    switch(robotState) {
      case STATE_FOLLOWING:
        followLine();
        checkSpecialCases();
        break;
        
      case STATE_INTERSECTION:
        handleIntersection();
        break;
        
      case STATE_LOST:
        recoverLine();
        break;
    }
  } else {
    stopMotors();
  }
}

void handleButton() {
    static unsigned long lastPress = 0;
    // Fixed: Added closing parenthesis for digitalRead()
    if (!digitalRead(BUTTON_PIN)) {  // <-- Added missing ')'
      if (millis() - lastPress > 300) {
        robotActive = !robotActive;
        lastPress = millis();
        Serial.println(robotActive ? "START" : "STOP");
      }
    }
  }  // Properly closed function

void readSensors() {
  for(int i = 0; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    sensorState[i] = raw > sensorThresholds[i];
    sensorValues[i] = raw;
  }
}

void checkSpecialCases() {
  // Intersection detection (all center sensors on line)
  if(sensorState[1] && sensorState[2] && sensorState[3]) {
    robotState = STATE_INTERSECTION;
    intersectionTimer = millis();
  }
  // Line lost detection
  else if(!sensorState[0] && !sensorState[1] && !sensorState[2] && 
          !sensorState[3] && !sensorState[4]) {
    robotState = STATE_LOST;
  }
}

void followLine() {
  int error = calculateError();
  applyPID(error);
}

int calculateError() {
  int weights[] = {-4, -2, 0, 2, 4}; // Weighted error calculation
  int errorSum = 0;
  int sensorCount = 0;

  for(int i = 0; i < 5; i++) {
    if(sensorState[i]) {
      errorSum += weights[i] * 1000; // Scale error
      sensorCount++;
    }
  }

  if(sensorCount == 0) return previousError;
  return errorSum / sensorCount;
}

void applyPID(int error) {
  // Adaptive speed based on error magnitude
  int currentSpeed = baseSpeed - constrain(abs(error)/20, 0, 80);
  currentSpeed = constrain(currentSpeed, 100, maxSpeed);

  integral += error;
  integral = constrain(integral, -1000, 1000); // Anti-windup
  derivative = error - previousError;

  int pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  pidOutput = constrain(pidOutput, -currentSpeed, currentSpeed);

  int leftSpeed = currentSpeed - pidOutput;
  int rightSpeed = currentSpeed + pidOutput;

  setMotors(leftSpeed, rightSpeed);
  previousError = error;
}

void handleIntersection() {
  // Drive straight through intersection
  setMotors(baseSpeed, baseSpeed);
  
  if(millis() - intersectionTimer > intersectionDelay) {
    robotState = STATE_FOLLOWING;
  }
}

void recoverLine() {
  // Spiral search pattern
  static int searchSpeed = 100;
  setMotors(-searchSpeed, searchSpeed);
  
  // Check for line during search
  for(int i = 0; i < 5; i++) {
    if(sensorState[i]) {
      robotState = STATE_FOLLOWING;
      searchSpeed = 100;
      return;
    }
  }
  
  searchSpeed += 10;
  searchSpeed = constrain(searchSpeed, 100, 200);
}

void setMotors(int left, int right) {
  // Left motor control
  left = constrain(left, -255, 255);
  analogWrite(PWMA, abs(left));
  digitalWrite(AIN1, left > 0 ? HIGH : LOW);
  digitalWrite(AIN2, left > 0 ? LOW : HIGH);

  // Right motor control
  right = constrain(right, -255, 255);
  analogWrite(PWMB, abs(right));
  digitalWrite(BIN1, right > 0 ? HIGH : LOW);
  digitalWrite(BIN2, right > 0 ? LOW : HIGH);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

/**
 * Advanced Line Follower for Complex Paths
 * Features: PID control, intersection handling, dynamic thresholding, speed adaptation
 * Tinkercad Compatible
 */

// Motor Control Pins (L298N Driver Connections)
#define AIN1 13  // Left Motor Direction 1
#define AIN2 12  // Left Motor Direction 2
#define BIN1 8   // Right Motor Direction 1
#define BIN2 9   // Right Motor Direction 2
#define PWMA 11  // Left Motor Speed Control
#define PWMB 10  // Right Motor Speed Control
#define STBY 5   // Standby Mode (HIGH = Active)

// Sensor Configuration (5-Point Array - Left to Right Sensors)
const int sensorPins[] = {A0, A1, A2, A3, A4}; // Analog pins for sensors
int sensorValues[5];           // Raw sensor readings
int sensorThresholds[5];       // Calibration thresholds
bool sensorState[5];          // Boolean detection state (true = line detected)

// PID Control Parameters
float Kp = 12.0, Ki = 0.02, Kd = 3.0; // Tuning coefficients
int baseSpeed = 160;           // Base motor speed (0-255)
int maxSpeed = 255;            // Maximum speed limit
int previousError = 0;         // Previous error value for derivative term
int integral = 0;              // Integral accumulation
int derivative = 0;            // Derivative calculation

// Robot State Machine Definitions
#define STATE_FOLLOWING 0      // Normal line following
#define STATE_INTERSECTION 1   // Detected intersection
#define STATE_LOST 2           // Line lost, searching mode
int robotState = STATE_FOLLOWING;

// Timing Control for Intersections
unsigned long intersectionTimer = 0; // Time tracking for intersection handling
const int intersectionDelay = 500;  // Duration to drive straight at intersection (ms)

// Start/Stop Button Control
#define BUTTON_PIN 7           // Physical button for activation
bool robotActive = false;     // System enable flag

void setup() {
  // Motor Pin Initialization
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);    // Enable motor driver

  // Button Setup with Internal Pull-Up Resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Automatic Threshold Calibration
  autoCalibrate();             // Initialize sensor thresholds
  
  Serial.begin(9600);          // For debugging output
}

void autoCalibrate() {
  // Dynamic threshold calibration routine
  digitalWrite(LED_BUILTIN, HIGH);  // Indicate calibration in progress
  delay(1000);                      // Stabilize sensors
  
  for(int i = 0; i < 5; i++) {      // For each sensor
    int minVal = 1023;              // Initialize min/max trackers
    int maxVal = 0;
    
    // Collect calibration samples
    for(int j = 0; j < 100; j++) {
      int val = analogRead(sensorPins[i]);
      minVal = min(minVal, val);    // Update min reading
      maxVal = max(maxVal, val);    // Update max reading
      delay(5);                     // Sample delay
    }
    sensorThresholds[i] = (minVal + maxVal) / 2;  // Set midpoint threshold
  }
  digitalWrite(LED_BUILTIN, LOW);   // Calibration complete
}

void loop() {
  handleButton();       // Check start/stop button state
  readSensors();        // Read all sensor values
  
  if(robotActive) {
    switch(robotState) {
      case STATE_FOLLOWING:
        followLine();         // Standard line following
        checkSpecialCases();  // Detect intersections/line loss
        break;
        
      case STATE_INTERSECTION:
        handleIntersection(); // Navigate detected intersection
        break;
        
      case STATE_LOST:
        recoverLine();        // Search pattern for lost line
        break;
    }
  } else {
    stopMotors();         // Safety stop when inactive
  }
}

void handleButton() {
  static unsigned long lastPress = 0;
  // Debounced button handler with edge detection
  if (!digitalRead(BUTTON_PIN)) {  // Button pressed (LOW)
    if (millis() - lastPress > 300) {  // 300ms debounce
      robotActive = !robotActive;      // Toggle system state
      lastPress = millis();            // Update last press time
      Serial.println(robotActive ? "START" : "STOP");
    }
  }
}

void readSensors() {
  // Convert analog readings to binary states
  for(int i = 0; i < 5; i++) {
    int raw = analogRead(sensorPins[i]);
    sensorState[i] = raw > sensorThresholds[i];  // True = on line
    sensorValues[i] = raw;  // Store raw value for debugging
  }
}

void checkSpecialCases() {
  // Intersection Detection (3 center sensors active)
  if(sensorState[1] && sensorState[2] && sensorState[3]) {
    robotState = STATE_INTERSECTION;
    intersectionTimer = millis();  // Start intersection timer
  }
  // Line Lost Detection (All sensors inactive)
  else if(!sensorState[0] && !sensorState[1] && !sensorState[2] && 
          !sensorState[3] && !sensorState[4]) {
    robotState = STATE_LOST;
  }
}

void followLine() {
  int error = calculateError();  // Get position error
  applyPID(error);               // Apply PID correction
}

int calculateError() {
  // Weighted average error calculation (left=-4 to right=+4)
  int weights[] = {-4, -2, 0, 2, 4};  // Sensor position weights
  int errorSum = 0;
  int sensorCount = 0;

  for(int i = 0; i < 5; i++) {
    if(sensorState[i]) {
      errorSum += weights[i] * 1000;  // Scale factor for precision
      sensorCount++;
    }
  }

  // Maintain previous error if no sensors detect line
  if(sensorCount == 0) return previousError;
  return errorSum / sensorCount;  // Average error value
}

void applyPID(int error) {
  // Adaptive speed control based on error magnitude
  int currentSpeed = baseSpeed - constrain(abs(error)/20, 0, 80);
  currentSpeed = constrain(currentSpeed, 100, maxSpeed);  // Speed limits

  integral += error;  // Accumulate integral term
  integral = constrain(integral, -1000, 1000); // Anti-windup protection
  derivative = error - previousError;  // Calculate derivative

  // PID output calculation
  int pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  pidOutput = constrain(pidOutput, -currentSpeed, currentSpeed);  // Output limits

  // Differential motor control
  int leftSpeed = currentSpeed - pidOutput;
  int rightSpeed = currentSpeed + pidOutput;

  setMotors(leftSpeed, rightSpeed);  // Apply calculated speeds
  previousError = error;  // Save for next iteration
}

void handleIntersection() {
  // Continue straight through intersection
  setMotors(baseSpeed, baseSpeed);
  
  // Return to following after delay
  if(millis() - intersectionTimer > intersectionDelay) {
    robotState = STATE_FOLLOWING;
  }
}

void recoverLine() {
  // Spiral search pattern for lost line
  static int searchSpeed = 100;  // Starting search speed
  setMotors(-searchSpeed, searchSpeed);  // Rotate in place
  
  // Check for line detection during search
  for(int i = 0; i < 5; i++) {
    if(sensorState[i]) {
      robotState = STATE_FOLLOWING;  // Line found!
      searchSpeed = 100;             // Reset speed
      return;
    }
  }
  
  // Gradually increase search radius
  searchSpeed += 10;
  searchSpeed = constrain(searchSpeed, 100, 200);  // Speed limits
}

void setMotors(int left, int right) {
  // Left Motor Control
  left = constrain(left, -255, 255);
  analogWrite(PWMA, abs(left));  // Set speed (0-255)
  // Direction control
  digitalWrite(AIN1, left > 0 ? HIGH : LOW);
  digitalWrite(AIN2, left > 0 ? LOW : HIGH);

  // Right Motor Control
  right = constrain(right, -255, 255);
  analogWrite(PWMB, abs(right));  // Set speed (0-255)
  // Direction control
  digitalWrite(BIN1, right > 0 ? HIGH : LOW);
  digitalWrite(BIN2, right > 0 ? LOW : HIGH);
}

void stopMotors() {
  // Safely stop both motors
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

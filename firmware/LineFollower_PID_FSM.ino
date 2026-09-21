/*
  =============================================================================
  Autonomous Differential-Drive Line Following Robot with PID & Dual FSM
  Course: ME0551 - Robotics | German Jordanian University (GJU)
  Team: Ramez AlMasadeh, Abdullah AlBakri, Ahmad Alamir, Yazan Kanakri
  =============================================================================
*/

// --- Motor Driver Pins (L298N) ---
const int ENA = 5;   // PWM Left Motor
const int IN1 = 7;   // Left Direction 1
const int IN2 = 8;   // Left Direction 2
const int IN3 = 9;   // Right Direction 1
const int IN4 = 10;  // Right Direction 2
const int ENB = 6;   // PWM Right Motor

// --- Ultrasonic Sensor Pins (HC-SR04) ---
const int TRIG_PIN = 11;
const int ECHO_PIN = 12;

// --- IR Sensor Array Pins ---
const int SENSOR_PINS[5] = {A0, A1, A2, A3, A4};
float sensorFiltered[5] = {0, 0, 0, 0, 0};
const float EMA_ALPHA = 0.35; // Smoothing factor for Exponential Moving Average

// --- Speed Configuration ---
const int BASE_SPEED          = 125;
const int SLIGHT_SPEED        = 110;
const int PIVOT_ACQUIRE_SPEED = 155;
const int PIVOT_CENTER_SPEED  = 135;
const int MAX_SPEED           = 180;
const int MIN_SPEED           = 80;

// --- PID Controller Parameters ---
float Kp = 16.0;
float Ki = 0.1;
float Kd = 12.0;

float error = 0;
float lastError = 0;
float integral = 0;

// --- Turn State Machine ---
enum TurnState { TURN_NONE, TURN_LEFT, TURN_RIGHT };
enum TurnPhase { ACQUIRE, CENTERING };

TurnState turnState = TURN_NONE;
TurnPhase turnPhase = ACQUIRE;

// --- Obstacle Avoidance State Machine ---
enum AvoidPhase { AVOID_NONE, AVOID_RIGHT, AVOID_FORWARD, AVOID_LEFT, AVOID_RECOVER };
AvoidPhase avoidPhase = AVOID_NONE;
unsigned long avoidStartMs = 0;

const int OBST_CM = 5;                  // Obstacle detection threshold (cm)
const unsigned long AVOID_R_MS = 320;   // Right pivot duration (ms)
const unsigned long AVOID_F_MS = 450;   // Forward bypass duration (ms)
const unsigned long AVOID_L_MS = 490;   // Left pivot duration (ms)

const unsigned long OBST_CHECK_MS = 60; // Non-blocking ultrasonic polling interval
unsigned long lastObstCheckMs = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    sensorFiltered[i] = analogRead(SENSOR_PINS[i]);
  }

  Serial.begin(115200);
  delay(1000); // 1-second start delay for safety
}

void loop() {
  unsigned long now = millis();

  // 1. Check for obstacles periodically without blocking the PID loop
  if (now - lastObstCheckMs >= OBST_CHECK_MS) {
    lastObstCheckMs = now;
    if (avoidPhase == AVOID_NONE) {
      float dist = readUltrasonic();
      if (dist > 0 && dist <= OBST_CM) {
        avoidPhase = AVOID_RIGHT;
        avoidStartMs = now;
      }
    }
  }

  // 2. State Machine Execution
  if (avoidPhase != AVOID_NONE) {
    handleObstacleAvoidance(now);
  } else {
    updateSensorsEMA();
    forwardPID();
  }
}

void updateSensorsEMA() {
  for (int i = 0; i < 5; i++) {
    int raw = analogRead(SENSOR_PINS[i]);
    sensorFiltered[i] = (EMA_ALPHA * raw) + ((1.0 - EMA_ALPHA) * sensorFiltered[i]);
  }
}

float getError() {
  float weightedSum = 0;
  float totalIntensity = 0;
  float weights[5] = {-2.0, -1.0, 0.0, 1.0, 2.0};

  for (int i = 0; i < 5; i++) {
    weightedSum += weights[i] * sensorFiltered[i];
    totalIntensity += sensorFiltered[i];
  }

  if (totalIntensity < 50.0) return lastError;
  return (weightedSum / totalIntensity) * 10.0;
}

void forwardPID() {
  error = getError() / 2.0;

  if (abs(error) < 0.5) error = 0;

  integral += error;
  integral = constrain(integral, -60, 60);

  float derivative = constrain(error - lastError, -2.0, 2.0);
  lastError = error;

  float pid = (Kp * error) + (Ki * integral) + (Kd * derivative);
  pid = constrain(pid, -35, 35);

  int leftSpeed  = constrain((int)(BASE_SPEED - pid), MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED + pid), MIN_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed, true, true);
}

void handleObstacleAvoidance(unsigned long now) {
  unsigned long elapsed = now - avoidStartMs;

  switch (avoidPhase) {
    case AVOID_RIGHT:
      setMotors(PIVOT_ACQUIRE_SPEED, PIVOT_ACQUIRE_SPEED, true, false);
      if (elapsed >= AVOID_R_MS) {
        avoidPhase = AVOID_FORWARD;
        avoidStartMs = now;
      }
      break;

    case AVOID_FORWARD:
      setMotors(BASE_SPEED, BASE_SPEED, true, true);
      if (elapsed >= AVOID_F_MS) {
        avoidPhase = AVOID_LEFT;
        avoidStartMs = now;
      }
      break;

    case AVOID_LEFT:
      setMotors(PIVOT_ACQUIRE_SPEED, PIVOT_ACQUIRE_SPEED, false, true);
      if (elapsed >= AVOID_L_MS) {
        avoidPhase = AVOID_RECOVER;
        avoidStartMs = now;
      }
      break;

    case AVOID_RECOVER:
      updateSensorsEMA();
      setMotors(SLIGHT_SPEED, SLIGHT_SPEED, true, true);
      if (sensorFiltered[2] > 500) {
        avoidPhase = AVOID_NONE;
        integral = 0;
      }
      break;

    default:
      avoidPhase = AVOID_NONE;
      break;
  }
}

void setMotors(int leftPwm, int rightPwm, bool leftFwd, bool rightFwd) {
  digitalWrite(IN1, leftFwd ? HIGH : LOW);
  digitalWrite(IN2, leftFwd ? LOW : HIGH);
  digitalWrite(IN3, rightFwd ? HIGH : LOW);
  digitalWrite(IN4, rightFwd ? LOW : HIGH);

  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);
}

float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 15000);
  if (duration == 0) return -1;
  return (duration * 0.0343) / 2.0;
}
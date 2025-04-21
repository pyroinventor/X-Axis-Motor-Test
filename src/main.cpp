#include <Arduino.h>

// Pin Definitions
const int STEP_PIN = 26;       // PUL+ (Step positive)
const int STEP_PIN_N = 14;     // PUL- (Step negative)
const int DIR_PIN = 27;        // DIR+ (Direction positive)
const int DIR_PIN_N = 12;      // DIR- (Direction negative)
const int EN_PIN = 25;         // ENA+ (Enable positive)
const int EN_PIN_N = 13;       // ENA- (Enable negative)
const int LIMIT_SW = 33;       // NC Limit Switch pin
const int HOME_BACKOFF = 500;  // Steps to back off after hitting limit
const int HOMING_SPEED = 1000; // Microseconds between steps during homing

// Motor Configuration
const int TOTAL_STEPS = 2000; // Set this to match your DIP switch setting (400-25000)

// Position Tracking
long currentPosition = 0;         // Current position in steps (signed)
bool isHomed = false;             // Homing status flag
const long MAX_POSITION = 112000; // Maximum allowed position in steps (560mm)
const long MIN_POSITION = 0;      // Minimum allowed position in steps

// Motion Parameters
const int STEP_DELAY_US = 200; // Microseconds between steps (controls speed)
const bool CLOCKWISE = false;  // Direction definition
const bool COUNTER_CLOCKWISE = true;

const int MIN_STEP_DELAY = 100;  // Minimum microseconds between steps (max speed)
const int MAX_STEP_DELAY = 2000; // Maximum microseconds between steps (start speed)
const int ACCEL_STEPS = 100;     // Number of steps for acceleration/deceleration

// Add after existing configuration constants
const float STEPS_PER_MM = 200.0; // 2000 steps / 10mm = 200 steps/mm
const float MAX_POSITION_MM = MAX_POSITION / STEPS_PER_MM;
const float MIN_POSITION_MM = MIN_POSITION / STEPS_PER_MM;

void setup()
{
  Serial.begin(115200);

  // Configure motor control pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(STEP_PIN_N, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_PIN_N, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(EN_PIN_N, OUTPUT);
  pinMode(LIMIT_SW, INPUT_PULLUP); // Enable internal pullup for NC switch

  // Initialize pins (differential signals are complementary)
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP_PIN_N, HIGH);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(DIR_PIN_N, HIGH);
  digitalWrite(EN_PIN, HIGH);  // Active LOW
  digitalWrite(EN_PIN_N, LOW); // Complementary to EN_PIN

  Serial.println("Stepper Motor Test Program");
  Serial.println("Commands:");
  Serial.println("E: Enable motor");
  Serial.println("D: Disable motor");
  Serial.println("C: Rotate clockwise 1 revolution");
  Serial.println("R: Rotate counter-clockwise 1 revolution");
  Serial.println("H: Home axis"); // Add to command list
  Serial.println("P: Report current position");
}

// Add this function to check limit switch
bool isLimitTriggered()
{
  return digitalRead(LIMIT_SW) == HIGH; // Switch is NC, so HIGH means triggered
}

void setDirection(bool clockwise)
{
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  digitalWrite(DIR_PIN_N, clockwise ? LOW : HIGH); // Complementary signal
  delayMicroseconds(10);                           // Allow direction signal to settle
}

void step()
{
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN_N, LOW); // Complementary signal
  delayMicroseconds(10);         // Minimum pulse width for DM556
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP_PIN_N, HIGH); // Complementary signal
  delayMicroseconds(STEP_DELAY_US);
}

int calculateStepDelay(int currentStep, int totalSteps)
{
  // Acceleration phase
  if (currentStep < ACCEL_STEPS)
  {
    return map(currentStep, 0, ACCEL_STEPS, MAX_STEP_DELAY, MIN_STEP_DELAY);
  }
  // Deceleration phase
  else if (currentStep > (totalSteps - ACCEL_STEPS))
  {
    return map(totalSteps - currentStep, 0, ACCEL_STEPS, MAX_STEP_DELAY, MIN_STEP_DELAY);
  }
  // Constant speed phase
  else
  {
    return MIN_STEP_DELAY;
  }
}

void rotateWithAccel(bool clockwise, int steps)
{
  // Only check limits if we're homed
  if (isHomed)
  {
    // Invert the position calculation logic since CLOCKWISE is defined as false
    long targetPosition = currentPosition + (clockwise ? -steps : steps);
    if (targetPosition > MAX_POSITION || targetPosition < MIN_POSITION)
    {
      Serial.println("Error: Movement would exceed limits");
      return;
    }
  }

  setDirection(clockwise);

  for (int i = 0; i < steps; i++)
  {
    if (isLimitTriggered())
    {
      Serial.println("Limit switch triggered!");
      isHomed = false;
      return;
    }

    int stepDelay = calculateStepDelay(i, steps);
    step();

    // Update position tracking with inverted logic
    if (isHomed)
    {
      currentPosition += (clockwise ? -1 : 1);
    }
  }

  // Modify position reporting in rotateWithAccel()
  if (isHomed)
  {
    Serial.print("Move complete - Position: ");
    Serial.print(currentPosition);
    Serial.print(" steps (");
    Serial.print(currentPosition / STEPS_PER_MM);
    Serial.println(" mm)");
  }
  else
  {
    Serial.println("Move complete - Position unknown");
  }
}

// Modify homing complete message
void homeAxis()
{
  Serial.println("Homing axis...");
  digitalWrite(EN_PIN, LOW);

  // Move towards limit switch
  setDirection(COUNTER_CLOCKWISE); // Changed direction to match movement commands
  while (!isLimitTriggered())
  {
    step();
  }

  delay(500);

  // Back off from limit switch
  setDirection(CLOCKWISE); // Changed direction to match movement commands
  for (int i = 0; i < HOME_BACKOFF; i++)
  {
    step();
  }

  currentPosition = 0;
  isHomed = true;
  Serial.println("Homing complete - Position: 0 steps (0 mm)");
}

void loop()
{
  if (Serial.available())
  {
    char cmd = Serial.read();

    switch (toupper(cmd))
    {
    case 'E': // Enable motor
      digitalWrite(EN_PIN, LOW);
      Serial.println("Motor enabled");
      break;

    case 'D': // Disable motor
      digitalWrite(EN_PIN, HIGH);
      Serial.println("Motor disabled");
      break;

    case 'C': // Clockwise rotation
      Serial.println("Rotating clockwise 1 revolution");
      rotateWithAccel(CLOCKWISE, TOTAL_STEPS); // Changed to match direction definition
      break;

    case 'R': // Counter-clockwise rotation
      Serial.println("Rotating counter-clockwise 1 revolution");
      rotateWithAccel(COUNTER_CLOCKWISE, TOTAL_STEPS); // Changed to match direction definition
      break;

    case 'H': // Home axis
      homeAxis();
      break;

    // Modify position reporting in loop() for 'P' command
    case 'P': // Report current position
      Serial.print("Current position: ");
      Serial.print(currentPosition);
      Serial.print(" steps (");
      Serial.print(currentPosition / STEPS_PER_MM);
      Serial.print(" mm), Homed: ");
      Serial.println(isHomed ? "Yes" : "No");
      break;
    }
  }
}
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
const int STEP_DELAY_US = 200;        // Microseconds between steps (controls speed)
const bool CLOCKWISE = true;          // Changed from false to true
const bool COUNTER_CLOCKWISE = false; // Changed from true to false

const int MIN_STEP_DELAY = 100;  // Minimum microseconds between steps (max speed)
const int MAX_STEP_DELAY = 2000; // Maximum microseconds between steps (start speed)
const int ACCEL_STEPS = 100;     // Number of steps for acceleration/deceleration

// Add after existing configuration constants
const float STEPS_PER_MM = 200.0; // 2000 steps / 10mm = 200 steps/mm
const float MAX_POSITION_MM = MAX_POSITION / STEPS_PER_MM;
const float MIN_POSITION_MM = MIN_POSITION / STEPS_PER_MM;

// Add at the top with other constants
const int DEBOUNCE_DELAY = 50; // Increased debounce time to 50ms

void setup()
{
  Serial.begin(115200);

  // Configure motor control pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(STEP_PIN_N, OUTPUT);
  pinMode(DIR_PIN, OUTPUT_OPEN_DRAIN);   // Changed to open-drain with pullup
  pinMode(DIR_PIN_N, OUTPUT_OPEN_DRAIN); // Changed to open-drain with pullup
  pinMode(EN_PIN, OUTPUT);
  pinMode(EN_PIN_N, OUTPUT);
  pinMode(LIMIT_SW, INPUT_PULLUP); // Enable internal pullup resistor

  // Enable pull-up resistors for direction pins
  digitalWrite(DIR_PIN, HIGH);   // Enable internal pullup
  digitalWrite(DIR_PIN_N, HIGH); // Enable internal pullup

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
  Serial.println("G<num>: Go to position in mm (e.g. G200)");
}

// Replace the existing isLimitTriggered function
bool isLimitTriggered()
{
  static bool lastState = false;
  static unsigned long lastDebounceTime = 0;

  // For NC switch, LOW means triggered (switch is opened)
  bool currentState = digitalRead(LIMIT_SW) == LOW;

  // If the state changed, reset debounce timer
  if (currentState != lastState)
  {
    lastDebounceTime = millis();
    lastState = currentState;
  }

  // Only accept the reading if it's been stable for the debounce delay
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    return currentState;
  }

  return false; // Default to not triggered if still debouncing
}

void setDirection(bool clockwise)
{
  if (clockwise)
  {
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(DIR_PIN_N, LOW);
  }
  else
  {
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(DIR_PIN_N, HIGH);
  }
  delayMicroseconds(10);
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
  // Debug output
  Serial.print("Direction: ");
  Serial.println(clockwise ? "CLOCKWISE" : "COUNTER_CLOCKWISE");

  // Only check limits if we're homed
  if (isHomed)
  {
    long targetPosition = currentPosition + (clockwise ? steps : -steps);
    if (targetPosition > MAX_POSITION || targetPosition < MIN_POSITION)
    {
      Serial.println("Error: Movement would exceed limits");
      return;
    }
  }

  // Set direction before movement
  setDirection(clockwise);
  delayMicroseconds(50); // Added delay for direction change to settle

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

    if (isHomed)
    {
      currentPosition += (clockwise ? 1 : -1);
    }
  }

  // Position reporting
  if (isHomed)
  {
    Serial.print("Move complete - Position: ");
    Serial.print(currentPosition);
    Serial.print(" steps (");
    Serial.print(currentPosition / STEPS_PER_MM);
    Serial.println(" mm)");
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

// Add this function before loop()
void moveToPosition(float targetMm)
{
  if (!isHomed)
  {
    Serial.println("Error: Please home axis first");
    return;
  }

  long targetSteps = (long)(targetMm * STEPS_PER_MM);
  if (targetSteps > MAX_POSITION || targetSteps < MIN_POSITION)
  {
    Serial.println("Error: Target position out of range");
    return;
  }

  long stepsToMove = targetSteps - currentPosition;
  if (stepsToMove == 0)
  {
    Serial.println("Already at target position");
    return;
  }

  bool direction = stepsToMove > 0;
  rotateWithAccel(direction, abs(stepsToMove));
}
void processCommand(String command)
{
  command.trim(); // Remove leading/trailing whitespace
  char cmd = toupper(command[0]);
  String param = command.substring(1); // Get everything after the command letter

  switch (cmd)
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
    rotateWithAccel(CLOCKWISE, TOTAL_STEPS);
    break;

  case 'R': // Counter-clockwise rotation
    Serial.println("Rotating counter-clockwise 1 revolution");
    rotateWithAccel(COUNTER_CLOCKWISE, TOTAL_STEPS);
    break;

  case 'H': // Home axis
    homeAxis();
    break;

  case 'P': // Report current position
    Serial.print("Current position: ");
    Serial.print(currentPosition);
    Serial.print(" steps (");
    Serial.print(currentPosition / STEPS_PER_MM);
    Serial.print(" mm), Homed: ");
    Serial.println(isHomed ? "Yes" : "No");
    break;

  case 'G': // Go to position in mm
    if (param.length() > 0)
    {
      float targetPos = param.toFloat();
      if (targetPos >= MIN_POSITION_MM && targetPos <= MAX_POSITION_MM)
      {
        Serial.print("Moving to position: ");
        Serial.print(targetPos);
        Serial.println("mm");
        moveToPosition(targetPos);
      }
      else
      {
        Serial.println("Error: Position out of range");
      }
    }
    else
    {
      Serial.println("Error: Invalid position format");
    }
    break;

  default:
    Serial.println("Unknown command");
    break;
  }
}
void loop()
{
  static String inputBuffer = "";

  while (Serial.available())
  {
    char c = Serial.read();

    // Process on newline
    if (c == '\n' || c == '\r')
    {
      if (inputBuffer.length() > 0)
      {
        processCommand(inputBuffer);
        inputBuffer = ""; // Clear buffer after processing
      }
    }
    else
    {
      inputBuffer += c; // Add character to buffer
    }
  }
}

// Add this new function to handle command processing

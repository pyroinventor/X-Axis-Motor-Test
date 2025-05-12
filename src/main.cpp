#include <Arduino.h>

// Pin Definitions
const int STEP_PIN = 23;      // PUL+ (Step positive)
const int STEP_PIN_N = 16;    // PUL- (Step negative)
const int DIR_PIN = 17;       // DIR+ (Direction positive)
const int DIR_PIN_N = 15;     // DIR- (Direction negative)
const int EN_PIN = 19;        // ENA+ (Enable positive)
const int EN_PIN_N = 18;      // ENA- (Enable negative)
const int LIMIT_SW = 4;       // NC Limit Switch pin
const int HOME_BACKOFF = 500; // Steps to back off after hitting limit
const int HOMING_SPEED = 500; // Microseconds between steps during homing

// Motor Configuration
const int TOTAL_STEPS = 2000; // Set this to match your DIP switch setting (400-25000)

// Position Tracking
long currentPosition = 0;         // Current position in steps (signed)
bool isHomed = false;             // Homing status flag
const long MAX_POSITION = 112000; // Maximum allowed position in steps (560mm)
const long MIN_POSITION = 0;      // Minimum allowed position in steps

// Motion Parameters
const int STEP_DELAY_US = 100;        // Microseconds between steps (controls speed)
const bool CLOCKWISE = true;          // Changed from false to true
const bool COUNTER_CLOCKWISE = false; // Changed from true to false

const int MIN_STEP_DELAY = 150;  // Increased for smoother motion
const int MAX_STEP_DELAY = 2000; // Starting speed (unchanged)
const int ACCEL_STEPS = 1000;    // Increased for longer acceleration period

const float STEPS_PER_MM = 200.0; // 2000 steps / 10mm = 200 steps/mm
const float MAX_POSITION_MM = MAX_POSITION / STEPS_PER_MM;
const float MIN_POSITION_MM = MIN_POSITION / STEPS_PER_MM;

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

  String startupMsg = "Stepper Motor Test Program\nCommands:\n"
                      "E: Enable motor\n"
                      "D: Disable motor\n"
                      "C: Rotate clockwise 1 revolution\n"
                      "R: Rotate counter-clockwise 1 revolution\n"
                      "H: Home axis\n"
                      "P: Report current position\n"
                      "G<num>: Go to position in mm (e.g. G200)";

  Serial.println(startupMsg);
}

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

void step(int delayUs = STEP_DELAY_US)
{
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP_PIN_N, LOW); // Complementary signal
  delayMicroseconds(10);         // Minimum pulse width for DM556
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP_PIN_N, HIGH); // Complementary signal
  delayMicroseconds(delayUs);     // Use passed-in delay value
}

float cubicBezier(float t, float p0, float p1, float p2, float p3)
{
  float oneMinusT = 1.0f - t;
  float oneMinusT2 = oneMinusT * oneMinusT;
  float t2 = t * t;
  return oneMinusT2 * oneMinusT * p0 +
         3.0f * oneMinusT2 * t * p1 +
         3.0f * oneMinusT * t2 * p2 +
         t2 * t * p3;
}

int calculateStepDelay(int currentStep, int totalSteps)
{
  // Control points for the Bezier curve - adjusted for more linear acceleration
  const float p0 = 0.0f;  // Start point
  const float p1 = 0.65f; // First control point moved up for more initial acceleration
  const float p2 = 0.85f; // Second control point moved down for more linear curve
  const float p3 = 1.0f;  // End point

  if (currentStep < ACCEL_STEPS)
  {
    // Acceleration phase
    float t = (float)currentStep / ACCEL_STEPS;
    float factor = cubicBezier(t, p0, p1, p2, p3);
    return MAX_STEP_DELAY - (factor * (MAX_STEP_DELAY - MIN_STEP_DELAY));
  }
  else if (currentStep > (totalSteps - ACCEL_STEPS))
  {
    // Deceleration phase
    float t = (float)(totalSteps - currentStep) / ACCEL_STEPS;
    float factor = cubicBezier(t, p0, p1, p2, p3);
    return MAX_STEP_DELAY - (factor * (MAX_STEP_DELAY - MIN_STEP_DELAY));
  }
  else
  {
    // Constant speed phase
    return MIN_STEP_DELAY;
  }
}

void sendResponse(const String &message)
{
  Serial.println(message);
}

void rotateWithAccel(bool clockwise, int steps)
{
  // Debug output
  sendResponse("Direction: " + String(clockwise ? "CLOCKWISE" : "COUNTER_CLOCKWISE"));

  // Only check limits if we're homed
  if (isHomed)
  {
    long targetPosition = currentPosition + (clockwise ? steps : -steps);
    if (targetPosition > MAX_POSITION || targetPosition < MIN_POSITION)
    {
      sendResponse("Error: Movement would exceed limits");
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
    step(stepDelay); // Pass the calculated delay to step function

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

void homeAxis()
{
  Serial.println("Homing axis...");
  digitalWrite(EN_PIN, LOW);

  // Move towards limit switch
  setDirection(COUNTER_CLOCKWISE); // Changed direction to match movement commands
  while (!isLimitTriggered())
  {
    step(HOMING_SPEED);
  }

  delay(500);

  // Back off from limit switch
  setDirection(CLOCKWISE); // Changed direction to match movement commands
  for (int i = 0; i < HOME_BACKOFF; i++)
  {
    step(HOMING_SPEED);
  }

  currentPosition = 0;
  isHomed = true;
  Serial.println("Homing complete - Position: 0 steps (0 mm)");
}

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
    sendResponse("Motor enabled");
    break;

  case 'D': // Disable motor
    digitalWrite(EN_PIN, HIGH);
    sendResponse("Motor disabled");
    break;

  case 'C': // Clockwise rotation
    sendResponse("Rotating clockwise 1 revolution");
    rotateWithAccel(CLOCKWISE, TOTAL_STEPS);
    break;

  case 'R': // Counter-clockwise rotation
    sendResponse("Rotating counter-clockwise 1 revolution");
    rotateWithAccel(COUNTER_CLOCKWISE, TOTAL_STEPS);
    break;

  case 'H': // Home axis
    homeAxis();
    break;

  case 'P': // Report current position
    sendResponse("Current position: " + String(currentPosition) + " steps (" + String(currentPosition / STEPS_PER_MM) + " mm), Homed: " + (isHomed ? "Yes" : "No"));
    break;

  case 'G': // Go to position in mm
    if (param.length() > 0)
    {
      float targetPos = param.toFloat();
      if (targetPos >= MIN_POSITION_MM && targetPos <= MAX_POSITION_MM)
      {
        sendResponse("Moving to position: " + String(targetPos) + "mm");
        moveToPosition(targetPos);
      }
      else
      {
        sendResponse("Error: Position out of range");
      }
    }
    else
    {
      sendResponse("Error: Invalid position format");
    }
    break;

  default:
    sendResponse("Unknown command");
    break;
  }
}

void loop()
{
  static String usbBuffer = "";
  static String btBuffer = "";

  // Handle USB Serial
  while (Serial.available())
  {
    char c = Serial.read();

    // Process on newline
    if (c == '\n' || c == '\r')
    {
      if (usbBuffer.length() > 0)
      {
        processCommand(usbBuffer);
        usbBuffer = ""; // Clear buffer after processing
      }
    }
    else
    {
      usbBuffer += c; // Add character to buffer
    }
  }
}

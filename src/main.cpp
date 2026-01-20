#include <Arduino.h>
#include <Wire.h>
#include <SCServo.h>
#include <m5_unit_joystick2.hpp>

// Serial pins for STS3215 communication
#define STS_TX_PIN 15
#define STS_RX_PIN 13
#define STS_BAUDRATE 1000000

// I2C pins for joystick
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 1

// PWM Servo settings
#define PWM_SERVO_PIN 10
#define BTN_ANGLE_70 3
#define BTN_ANGLE_110 4

// Joystick settings (16-bit ADC: 0-65535, center ~32768)
#define JOY_CENTER 32768
#define JOY_DEADZONE 3000
#define JOY_Y_ENABLE_DEADZONE 12000 // X axis deadzone for enabling Y axis (more lenient)

// Servo IDs
#define SERVO1_ID 1
#define SERVO2_ID 2
#define SERVO3_ID 3

// Speed settings (0-3000)
#define SERVO1_SPEED_MAX 1500
#define SERVO2_SPEED_MAX 1000 // 速度アップ依頼があるやつ
#define SERVO3_SPEED 2000
#define SERVO_ACC 200

// Multi-turn position control
#define MAX_STEP_PER_COMMAND 4096

// SERVO1 position control settings
#define SERVO1_STEP_AMOUNT 3000    // Steps per max trigger (1 rotation)
#define SERVO1_MAX_THRESHOLD 64535 // Joystick value to trigger (near 65535)
#define SERVO1_MIN_THRESHOLD 1000  // Joystick value to trigger (near 0)
#define SERVO1_SLOW_SPEED 300      // Slow movement speed
#define SERVO1_SLOW_STEPS 100      // Steps per slow movement
#define SERVO1_HOLD_TIME 1000      // ms to wait before slow movement

// SERVO3 position control settings
#define SERVO3_TARGET 34500 // Target position in steps (約8.8回転)

// Load monitoring
#define LOAD_CHECK_INTERVAL 200 // ms
#define LOAD_LIMIT 500          // 0-1000

SMS_STS sts;
M5UnitJoystick2 joystick;

// Previous speed to detect changes
int16_t prevSpeed2 = 0;
int prevPWMAngle = -1;

// Joystick axis locking (prevent simultaneous X/Y operation)
bool yAxisActive = false; // True when Y axis is being used

// SERVO1 multi-turn position tracking
long servo1TotalPos = 0;
int servo1LastPos = 0;
bool servo1MaxTriggered = false;   // True when max position triggered
int8_t servo1MaxDirection = 0;     // Direction of max trigger (-1 or 1)
unsigned long servo1HoldStart = 0; // When hold started
bool servo1Holding = false;        // True when holding in middle position
uint8_t servo1MaxCount = 0;        // Debounce counter for max detection
#define SERVO1_MAX_DEBOUNCE 3      // Require 3 consecutive max readings

// SERVO1 origin and limits
long servo1Origin = 0;              // Origin position (set by holding both buttons)
bool servo1OriginSet = false;       // True when origin has been set
#define SERVO1_LIMIT_PLUS 100       // Max steps in + direction from origin
#define SERVO1_LIMIT_MINUS -33100   // Max steps in - direction from origin
unsigned long bothButtonsStart = 0; // When both buttons started being held
bool bothButtonsHeld = false;       // True when both buttons are being held
#define ORIGIN_SET_HOLD_TIME 3000   // 3 seconds to set origin
long servo1CommandedPos = 0;        // Cumulative commanded position (for limit tracking)

// Servo 3 state:
// 0=idle, 1=reverse(homing), 2=forward(homing complete)
// 3=moving to target, 4=wait at target, 5=returning to origin
int servo3Phase = 0;
unsigned long servo3StartTime = 0;
long servo3TotalPos = 0;
int servo3LastPos = 0;
bool servo3Moving = false; // True while servo is moving (load > threshold)
#define SERVO3_INIT_SPEED 500
#define SERVO3_LOAD_LIMIT 300
#define SERVO3_MOVE_LOAD_THRESHOLD 50 // Load threshold to detect movement
#define SERVO3_FORWARD_TIME 1000      // ms (homing forward time)
#define SERVO3_WAIT_TIME 2000         // ms (wait time at target)

// Load monitoring state
unsigned long lastLoadCheck = 0;
bool servoOverLoad[3] = {false, false, false};
int8_t overLoadDirection[3] = {0, 0, 0}; // Direction when overload occurred: 1=positive, -1=negative

// Multi-turn position control: move steps (can exceed 4096)
void moveSteps(u8 id, long steps, u16 speed, u8 acc)
{
  if (steps == 0)
    return;

  int direction = (steps > 0) ? 1 : -1;
  long remaining = abs(steps);

  while (remaining > 0)
  {
    int chunk = (remaining > MAX_STEP_PER_COMMAND) ? MAX_STEP_PER_COMMAND : remaining;
    sts.WritePosEx(id, chunk * direction, speed, acc);
    remaining -= chunk;
    delay(2);
  }
}

// Update total position from servo encoder (handles wrap-around)
void updateTotalPos(u8 id, int currentPos, int &lastPos, long &totalPos)
{
  int delta = currentPos - lastPos;
  if (delta > 2048)
    delta -= 4096;
  if (delta < -2048)
    delta += 4096;
  totalPos += delta;
  lastPos = currentPos;
}

// Check and clamp Servo1 movement within limits (returns adjusted steps)
// Uses servo1CommandedPos for tracking (not affected by position read timing)
long clampServo1Movement(long requestedSteps)
{
  if (!servo1OriginSet)
  {
    return requestedSteps; // No limits if origin not set
  }

  // Use commanded position for limit calculation (more reliable than read position)
  long commandedRelativePos = servo1CommandedPos - servo1Origin;
  long targetRelativePos = commandedRelativePos + requestedSteps;

  // Only clamp in the direction we're moving
  if (requestedSteps > 0)
  {
    // Moving in + direction - only check + limit
    if (targetRelativePos > SERVO1_LIMIT_PLUS)
    {
      long clampedSteps = SERVO1_LIMIT_PLUS - commandedRelativePos;
      if (clampedSteps <= 0)
      {
        return 0; // Already at or past + limit
      }
      servo1CommandedPos += clampedSteps;
      return clampedSteps;
    }
  }
  else if (requestedSteps < 0)
  {
    // Moving in - direction - only check - limit
    if (targetRelativePos < SERVO1_LIMIT_MINUS)
    {
      long clampedSteps = SERVO1_LIMIT_MINUS - commandedRelativePos;
      if (clampedSteps >= 0)
      {
        return 0; // Already at or past - limit
      }
      servo1CommandedPos += clampedSteps;
      return clampedSteps;
    }
  }

  // Within limits - update commanded position
  servo1CommandedPos += requestedSteps;
  return requestedSteps;
}

// Servo 3 home return (原点復帰)
void servo3Home()
{
  servo3Phase = 1;
  sts.WriteSpe(SERVO3_ID, -SERVO3_INIT_SPEED, SERVO_ACC);
  Serial.println("Servo 3: Homing (reverse, waiting for load)");
}

// PWM Servo angle to duty cycle conversion
void setPWMServoAngle(int angle)
{
  // Servo pulse: 500-2500us for 0-180 degrees
  // At 50Hz (20ms period), 14-bit resolution (16383)
  int pulseUs = map(angle, 0, 180, 500, 2500);
  int duty = (pulseUs * 16383) / 20000;
  ledcWrite(PWM_SERVO_PIN, duty);
  Serial.print("PWM Servo: ");
  Serial.print(angle);
  Serial.println(" deg");
}

int16_t mapJoystickToSpeed(int32_t joyValue, int16_t maxSpeed)
{
  int32_t offset = joyValue - JOY_CENTER;

  // Apply dead zone
  if (abs(offset) < JOY_DEADZONE)
  {
    return 0;
  }

  // Map to speed (-maxSpeed to +maxSpeed)
  if (offset > 0)
  {
    offset -= JOY_DEADZONE;
    return map(offset, 0, 32767 - JOY_DEADZONE, 0, maxSpeed);
  }
  else
  {
    offset += JOY_DEADZONE;
    return map(offset, -(32768 - JOY_DEADZONE), 0, -maxSpeed, 0);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("STS3215 Joystick2 Controller");

  // Initialize PWM Servo
  if (ledcAttach(PWM_SERVO_PIN, 50, 14)) // 50Hz, 14-bit resolution
  {
    Serial.println("PWM Servo initialized on GPIO10");
    setPWMServoAngle(90); // Initial position: center
  }
  else
  {
    Serial.println("ERROR: Failed to attach PWM on GPIO10!");
  }

  // Initialize buttons
  pinMode(BTN_ANGLE_70, INPUT_PULLUP);
  pinMode(BTN_ANGLE_110, INPUT_PULLUP);

  // Initialize joystick
  if (!joystick.begin(&Wire, JOYSTICK2_ADDR, I2C_SDA_PIN, I2C_SCL_PIN))
  {
    Serial.println("Joystick2 not found!");
  }
  else
  {
    Serial.print("Joystick2 found! FW ver: ");
    Serial.println(joystick.get_firmware_version());
  }

  // Initialize serial for STS3215
  Serial1.begin(STS_BAUDRATE, SERIAL_8N1, STS_RX_PIN, STS_TX_PIN);
  sts.pSerial = &Serial1;
  delay(100);

  // Ping each servo to check communication
  Serial.println("Checking servo communication...");
  int servoIds[] = {SERVO1_ID, SERVO2_ID, SERVO3_ID};
  for (int i = 0; i < 3; i++)
  {
    int id = servoIds[i];
    int result = sts.Ping(id);
    Serial.print("Servo ");
    Serial.print(id);
    if (result != -1)
    {
      Serial.print(": OK (ID=");
      Serial.print(result);
      Serial.println(")");
    }
    else
    {
      Serial.println(": NOT FOUND");
    }
    delay(50);
  }

  // Disable angle limits and set Mode 3 for SERVO1 only
  // SERVO3 will be set to Mode 3 after homing completes
  Serial.println("Servo 1: Setting up multi-turn position control...");
  sts.unLockEprom(SERVO1_ID);
  sts.writeByte(SERVO1_ID, 9, 0);  // Min angle limit low byte
  sts.writeByte(SERVO1_ID, 10, 0); // Min angle limit high byte
  sts.writeByte(SERVO1_ID, 11, 0); // Max angle limit low byte
  sts.writeByte(SERVO1_ID, 12, 0); // Max angle limit high byte
  sts.LockEprom(SERVO1_ID);
  sts.writeByte(SERVO1_ID, 33, 3); // Mode 3 (Step mode)
  delay(50);

  // SERVO3: Disable angle limits but stay in wheel mode for homing
  Serial.println("Servo 3: Disabling angle limits (wheel mode for homing)...");
  sts.unLockEprom(SERVO3_ID);
  sts.writeByte(SERVO3_ID, 9, 0);
  sts.writeByte(SERVO3_ID, 10, 0);
  sts.writeByte(SERVO3_ID, 11, 0);
  sts.writeByte(SERVO3_ID, 12, 0);
  sts.LockEprom(SERVO3_ID);
  sts.WheelMode(SERVO3_ID); // Keep wheel mode for homing
  delay(50);

  // SERVO2 stays in wheel mode
  sts.WheelMode(SERVO2_ID);
  Serial.println("Servo 2: Wheel mode");

  // Initialize position tracking
  servo1LastPos = sts.ReadPos(SERVO1_ID);
  servo1TotalPos = 0;
  servo3LastPos = sts.ReadPos(SERVO3_ID);
  servo3TotalPos = 0;

  for (int i = 0; i < 3; i++)
  {
    int id = servoIds[i];
    int pos = sts.ReadPos(id);
    Serial.print("Servo ");
    Serial.print(id);
    Serial.print(": Pos=");
    Serial.println(pos);
  }

  // Servo 3 home return
  servo3Home();

  Serial.println("Initialization complete");
}

void loop()
{
  // Check for both buttons held (origin setting)
  bool btn1Pressed = (digitalRead(BTN_ANGLE_70) == LOW);
  bool btn2Pressed = (digitalRead(BTN_ANGLE_110) == LOW);

  if (btn1Pressed && btn2Pressed)
  {
    if (!bothButtonsHeld)
    {
      bothButtonsHeld = true;
      bothButtonsStart = millis();
    }
    else if (millis() - bothButtonsStart >= ORIGIN_SET_HOLD_TIME)
    {
      // Set current position as origin
      servo1Origin = servo1TotalPos;
      servo1CommandedPos = servo1TotalPos; // Initialize commanded position
      servo1OriginSet = true;
      bothButtonsHeld = false; // Reset to avoid repeated triggering
      Serial.print("Servo 1: Origin set at ");
      Serial.println(servo1Origin);
    }
  }
  else
  {
    bothButtonsHeld = false;
  }

  // PWM servo: 45 when btn1 only, 135 when btn2 only, otherwise 87
  int targetAngle = 87; // 調整依頼のあるやつ
  if (btn1Pressed && !btn2Pressed)
  {
    targetAngle = 45;
  }
  else if (btn2Pressed && !btn1Pressed)
  {
    targetAngle = 135;
  }
  if (targetAngle != prevPWMAngle)
  {
    setPWMServoAngle(targetAngle);
    prevPWMAngle = targetAngle;
  }

  uint16_t adc_x = JOY_CENTER;
  uint16_t adc_y = JOY_CENTER;

  // Read joystick ADC values (16-bit)
  joystick.get_joy_adc_16bits_value_xy(&adc_x, &adc_y);

  // SERVO2: Y axis speed control (wheel mode)
  int16_t speed2 = mapJoystickToSpeed(adc_y, SERVO2_SPEED_MAX);

  // Check button state (use already-read values)
  bool buttonPressed = btn1Pressed || btn2Pressed;

  // SERVO1: X axis position control
  // Update total position tracking
  int servo1CurrentPos = sts.ReadPos(SERVO1_ID);
  updateTotalPos(SERVO1_ID, servo1CurrentPos, servo1LastPos, servo1TotalPos);

  // Check if joystick is at center (for both axes)
  int32_t offsetX = abs((int32_t)adc_x - JOY_CENTER);
  int32_t offsetY = abs((int32_t)adc_y - JOY_CENTER);
  bool atCenterX = (offsetX < JOY_DEADZONE);
  bool atCenterY = (offsetY < JOY_DEADZONE);
  bool fullyAtCenter = atCenterX && atCenterY;

  // Reset Y axis lock when joystick fully returns to center
  if (fullyAtCenter)
  {
    yAxisActive = false;
  }

  // Check if joystick is at max
  bool atMaxRight = (adc_x > SERVO1_MAX_THRESHOLD);
  bool atMaxLeft = (adc_x < SERVO1_MIN_THRESHOLD);

  if (atCenterX)
  {
    // Reset trigger when X returned to center
    servo1MaxTriggered = false;
    servo1MaxDirection = 0;
    servo1Holding = false;
    servo1HoldStart = 0;
    servo1MaxCount = 0;
    // Note: Don't sync commandedPos here - it would bypass limit protection
  }
  else if (!servo1MaxTriggered && !buttonPressed && !yAxisActive)
  {
    if (atMaxRight)
    {
      servo1MaxCount++;
      if (servo1MaxCount >= SERVO1_MAX_DEBOUNCE)
      {
        // Max right triggered - move negative (inverted)
        long steps = clampServo1Movement(-SERVO1_STEP_AMOUNT);
        if (steps != 0)
        {
          moveSteps(SERVO1_ID, steps, SERVO1_SPEED_MAX, SERVO_ACC);
          Serial.print("Servo 1: Move ");
          Serial.println(steps);
        }
        servo1MaxTriggered = true;
        servo1MaxDirection = -1;
        servo1Holding = false;
        servo1MaxCount = 0;
      }
    }
    else if (atMaxLeft)
    {
      servo1MaxCount++;
      if (servo1MaxCount >= SERVO1_MAX_DEBOUNCE)
      {
        // Max left triggered - move positive (inverted)
        long steps = clampServo1Movement(SERVO1_STEP_AMOUNT);
        if (steps != 0)
        {
          moveSteps(SERVO1_ID, steps, SERVO1_SPEED_MAX, SERVO_ACC);
          Serial.print("Servo 1: Move ");
          Serial.println(steps);
        }
        servo1MaxTriggered = true;
        servo1MaxDirection = 1;
        servo1Holding = false;
        servo1MaxCount = 0;
      }
    }
    else
    {
      // Not at max - reset debounce counter
      servo1MaxCount = 0;
      // Middle position - start hold timer or slow move
      if (!servo1Holding)
      {
        servo1Holding = true;
        servo1HoldStart = millis();
      }
      else if (millis() - servo1HoldStart >= SERVO1_HOLD_TIME)
      {
        // Slow movement after 1 second hold (inverted)
        int8_t direction = (adc_x > JOY_CENTER) ? -1 : 1;
        long steps = clampServo1Movement(SERVO1_SLOW_STEPS * direction);
        if (steps != 0)
        {
          moveSteps(SERVO1_ID, steps, SERVO1_SLOW_SPEED, SERVO_ACC);
        }
        servo1HoldStart = millis(); // Reset for continuous slow movement
      }
    }
  }

  // Only use Y axis if X axis is mostly centered (more lenient than SERVO1 deadzone)
  bool xNearCenter = (offsetX < JOY_Y_ENABLE_DEADZONE);
  if (!xNearCenter)
  {
    speed2 = 0;
  }

  // Reset overload flag only when joystick returns to center
  if (speed2 == 0 && servoOverLoad[1])
  {
    servoOverLoad[1] = false;
    overLoadDirection[1] = 0;
  }
  // Update Servo 2 if speed changed (block same direction as overload, load sign is opposite)
  if (servoOverLoad[1] && ((speed2 > 0 && overLoadDirection[1] < 0) || (speed2 < 0 && overLoadDirection[1] > 0)))
  {
    speed2 = 0;
  }
  // Set Y axis active flag when Y axis is being used
  if (speed2 != 0)
  {
    yAxisActive = true;
  }

  if (speed2 != prevSpeed2)
  {
    sts.WriteSpe(SERVO2_ID, speed2, SERVO_ACC);
    prevSpeed2 = speed2;
  }

  // SERVO3: Position control
  // Update total position tracking
  int servo3CurrentPos = sts.ReadPos(SERVO3_ID);
  updateTotalPos(SERVO3_ID, servo3CurrentPos, servo3LastPos, servo3TotalPos);

  // Servo 3: Homing forward complete (phase 2) - switch to position mode
  if (servo3Phase == 2 && millis() - servo3StartTime >= SERVO3_FORWARD_TIME)
  {
    sts.WriteSpe(SERVO3_ID, 0, SERVO_ACC);
    delay(100);

    // Set current position as origin (0)
    sts.CalibrationOfs(SERVO3_ID);
    delay(50);

    // Switch to Mode 3 (Step mode) for position control
    sts.writeByte(SERVO3_ID, 33, 3);
    delay(50);
    Serial.println("Servo 3: Switched to position mode (Mode 3)");

    servo3LastPos = sts.ReadPos(SERVO3_ID);
    servo3TotalPos = 0;
    servo3Phase = 0;
    Serial.print("Servo 3: Homing complete, TotalPos=");
    Serial.println(servo3TotalPos);
  }

  // Servo 3: Joystick button triggers movement sequence
  // Block if any other action is happening (buttons pressed, joystick not centered)
  bool anyActivity = buttonPressed || !fullyAtCenter || (speed2 != 0);
  if (servo3Phase == 0 && joystick.get_button_value() == 0 && !anyActivity)
  {
    // Move to target position using position control
    moveSteps(SERVO3_ID, SERVO3_TARGET, SERVO3_SPEED, SERVO_ACC);
    servo3Phase = 3;
    servo3Moving = false; // Wait for load to rise before detecting completion
    servo3StartTime = millis();
    Serial.print("Servo 3: Moving to ");
    Serial.println(SERVO3_TARGET);
  }

  // Servo 3: Wait for movement to complete (phase 3)
  // Detect completion when load drops below threshold
  if (servo3Phase == 3)
  {
    int load3 = abs(sts.ReadLoad(SERVO3_ID));
    // Once we see high load, we're moving
    if (load3 > SERVO3_MOVE_LOAD_THRESHOLD)
    {
      servo3Moving = true;
    }
    // If we were moving and now load is low, movement complete
    if (servo3Moving && load3 < SERVO3_MOVE_LOAD_THRESHOLD)
    {
      servo3Phase = 4;
      servo3StartTime = millis();
      Serial.println("Servo 3: At target, waiting...");
    }
  }

  // Servo 3: Wait phase (phase 4)
  if (servo3Phase == 4 && millis() - servo3StartTime >= SERVO3_WAIT_TIME)
  {
    // Return to origin - use SERVO3_TARGET instead of tracked position
    moveSteps(SERVO3_ID, -SERVO3_TARGET, SERVO3_SPEED, SERVO_ACC);
    servo3Phase = 5;
    servo3Moving = false; // Wait for load to rise before detecting completion
    servo3StartTime = millis();
    Serial.println("Servo 3: Returning to origin");
  }

  // Servo 3: Wait for return to complete (phase 5)
  // Detect completion when load drops below threshold
  if (servo3Phase == 5)
  {
    int load3 = abs(sts.ReadLoad(SERVO3_ID));
    if (load3 > SERVO3_MOVE_LOAD_THRESHOLD)
    {
      servo3Moving = true;
    }
    if (servo3Moving && load3 < SERVO3_MOVE_LOAD_THRESHOLD)
    {
      servo3Phase = 0;
      servo3Moving = false;
      servo3TotalPos = 0;
      servo3LastPos = sts.ReadPos(SERVO3_ID);
      Serial.println("Servo 3: Back at origin");
    }
  }

  // Load monitoring every 200ms
  if (millis() - lastLoadCheck >= LOAD_CHECK_INTERVAL)
  {
    lastLoadCheck = millis();
    int servoIds[] = {SERVO1_ID, SERVO2_ID, SERVO3_ID};

    // Show position info
    Serial.print("S1: TotalPos=");
    Serial.print(servo1TotalPos);
    Serial.print(" | S3: TotalPos=");
    Serial.print(servo3TotalPos);
    Serial.print(" Phase=");
    Serial.print(servo3Phase);
    Serial.print(" | Load: ");
    for (int i = 0; i < 3; i++)
    {
      int load = sts.ReadLoad(servoIds[i]);
      Serial.print(servoIds[i]);
      Serial.print("=");
      Serial.print(load);
      Serial.print(" ");
      // Servo 3: Check load to trigger forward rotation (homing phase 1)
      if (i == 2 && servo3Phase == 1 && abs(load) > SERVO3_LOAD_LIMIT)
      {
        servo3Phase = 2;
        servo3StartTime = millis();
        sts.WriteSpe(SERVO3_ID, SERVO3_INIT_SPEED, SERVO_ACC);
        Serial.println();
        Serial.println("*** Servo 3: Load triggered, Forward ***");
      }
    }
    Serial.println();
  }

  delay(20);
}

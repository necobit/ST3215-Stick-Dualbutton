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

// Servo IDs
#define SERVO1_ID 1
#define SERVO2_ID 2
#define SERVO3_ID 3

// Speed settings (0-3000)
#define SERVO1_SPEED_MAX 1500
#define SERVO2_SPEED_MAX 500 // 速度アップ依頼があるやつ
#define SERVO3_SPEED 3000
#define SERVO_ACC 200

// Multi-turn position control
#define MAX_STEP_PER_COMMAND 4096

// SERVO1 position control settings
#define SERVO1_STEP_AMOUNT 4096      // Steps per max trigger (1 rotation)
#define SERVO1_MAX_THRESHOLD 60000   // Joystick value to trigger (near 65535)
#define SERVO1_MIN_THRESHOLD 5000    // Joystick value to trigger (near 0)
#define SERVO1_SLOW_SPEED 300        // Slow movement speed
#define SERVO1_SLOW_STEPS 100        // Steps per slow movement
#define SERVO1_HOLD_TIME 1000        // ms to wait before slow movement

// SERVO3 position control settings
#define SERVO3_TARGET 36000          // Target position in steps (約8.8回転)

// Load monitoring
#define LOAD_CHECK_INTERVAL 200 // ms
#define LOAD_LIMIT 500          // 0-1000

SMS_STS sts;
M5UnitJoystick2 joystick;

// Previous speed to detect changes
int16_t prevSpeed2 = 0;
int prevPWMAngle = -1;

// SERVO1 multi-turn position tracking
long servo1TotalPos = 0;
int servo1LastPos = 0;
bool servo1MaxTriggered = false;      // True when max position triggered
int8_t servo1MaxDirection = 0;        // Direction of max trigger (-1 or 1)
unsigned long servo1HoldStart = 0;    // When hold started
bool servo1Holding = false;           // True when holding in middle position

// Servo 3 state:
// 0=idle, 1=reverse(homing), 2=forward(homing complete)
// 3=moving to target, 4=wait at target, 5=returning to origin
int servo3Phase = 0;
unsigned long servo3StartTime = 0;
long servo3TotalPos = 0;
int servo3LastPos = 0;
#define SERVO3_INIT_SPEED 500
#define SERVO3_LOAD_LIMIT 300
#define SERVO3_FORWARD_TIME 1000 // ms (homing forward time)
#define SERVO3_WAIT_TIME 2000    // ms (wait time at target)
#define SERVO3_MOVE_SPEED 3000   // Position mode speed

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

  // Disable angle limits and set Mode 3 for SERVO1 and SERVO3
  int positionServoIds[] = {SERVO1_ID, SERVO3_ID};
  for (int i = 0; i < 2; i++)
  {
    int id = positionServoIds[i];
    Serial.print("Servo ");
    Serial.print(id);
    Serial.println(": Setting up multi-turn position control...");

    // Disable angle limits
    sts.unLockEprom(id);
    sts.writeByte(id, 9, 0);  // Min angle limit low byte
    sts.writeByte(id, 10, 0); // Min angle limit high byte
    sts.writeByte(id, 11, 0); // Max angle limit low byte
    sts.writeByte(id, 12, 0); // Max angle limit high byte
    sts.LockEprom(id);

    // Set Mode 3 (Step mode)
    sts.writeByte(id, 33, 3);
    delay(50);
  }

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
  // PWM servo: 70 when btn3, 110 when btn4, otherwise 90
  int targetAngle = 90; // 調整依頼のあるやつ
  if (digitalRead(BTN_ANGLE_70) == LOW)
  {
    targetAngle = 45;
  }
  else if (digitalRead(BTN_ANGLE_110) == LOW)
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

  // Check button state
  bool buttonPressed = (digitalRead(BTN_ANGLE_70) == LOW) || (digitalRead(BTN_ANGLE_110) == LOW);

  // SERVO1: X axis position control
  // Update total position tracking
  int servo1CurrentPos = sts.ReadPos(SERVO1_ID);
  updateTotalPos(SERVO1_ID, servo1CurrentPos, servo1LastPos, servo1TotalPos);

  // Check if joystick is at center
  int32_t offsetX = abs((int32_t)adc_x - JOY_CENTER);
  bool atCenter = (offsetX < JOY_DEADZONE);

  // Check if joystick is at max
  bool atMaxRight = (adc_x > SERVO1_MAX_THRESHOLD);
  bool atMaxLeft = (adc_x < SERVO1_MIN_THRESHOLD);

  if (atCenter)
  {
    // Reset trigger when returned to center
    servo1MaxTriggered = false;
    servo1MaxDirection = 0;
    servo1Holding = false;
    servo1HoldStart = 0;
  }
  else if (!servo1MaxTriggered && !buttonPressed)
  {
    if (atMaxRight)
    {
      // Max right triggered - move positive
      moveSteps(SERVO1_ID, SERVO1_STEP_AMOUNT, SERVO1_SPEED_MAX, SERVO_ACC);
      servo1MaxTriggered = true;
      servo1MaxDirection = 1;
      servo1Holding = false;
      Serial.print("Servo 1: Move +");
      Serial.println(SERVO1_STEP_AMOUNT);
    }
    else if (atMaxLeft)
    {
      // Max left triggered - move negative
      moveSteps(SERVO1_ID, -SERVO1_STEP_AMOUNT, SERVO1_SPEED_MAX, SERVO_ACC);
      servo1MaxTriggered = true;
      servo1MaxDirection = -1;
      servo1Holding = false;
      Serial.print("Servo 1: Move -");
      Serial.println(SERVO1_STEP_AMOUNT);
    }
    else
    {
      // Middle position - start hold timer or slow move
      if (!servo1Holding)
      {
        servo1Holding = true;
        servo1HoldStart = millis();
      }
      else if (millis() - servo1HoldStart >= SERVO1_HOLD_TIME)
      {
        // Slow movement after 1 second hold
        int8_t direction = (adc_x > JOY_CENTER) ? 1 : -1;
        moveSteps(SERVO1_ID, SERVO1_SLOW_STEPS * direction, SERVO1_SLOW_SPEED, SERVO_ACC);
        servo1HoldStart = millis(); // Reset for continuous slow movement
      }
    }
  }

  // Only use Y axis if X axis is at center (avoid simultaneous movement)
  if (!atCenter)
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
    servo3LastPos = sts.ReadPos(SERVO3_ID);
    servo3TotalPos = 0;
    servo3Phase = 0;
    Serial.print("Servo 3: Homing complete, TotalPos=");
    Serial.println(servo3TotalPos);
  }

  // Servo 3: Joystick button triggers movement sequence
  // Block if any other action is happening (buttons pressed, X axis active)
  bool anyActivity = buttonPressed || !atCenter || (speed2 != 0);
  if (servo3Phase == 0 && joystick.get_button_value() == 0 && !anyActivity)
  {
    // Move to target position using position control
    moveSteps(SERVO3_ID, SERVO3_TARGET, SERVO3_MOVE_SPEED, SERVO_ACC);
    servo3Phase = 3;
    servo3StartTime = millis();
    Serial.print("Servo 3: Moving to ");
    Serial.println(SERVO3_TARGET);
  }

  // Servo 3: Wait for movement to complete (phase 3)
  if (servo3Phase == 3)
  {
    // Check if close to target
    if (abs(servo3TotalPos - SERVO3_TARGET) < 100 || millis() - servo3StartTime > 10000)
    {
      servo3Phase = 4;
      servo3StartTime = millis();
      Serial.print("Servo 3: At target, TotalPos=");
      Serial.print(servo3TotalPos);
      Serial.println(", waiting...");
    }
  }

  // Servo 3: Wait phase (phase 4)
  if (servo3Phase == 4 && millis() - servo3StartTime >= SERVO3_WAIT_TIME)
  {
    // Return to origin
    moveSteps(SERVO3_ID, -servo3TotalPos, SERVO3_MOVE_SPEED, SERVO_ACC);
    servo3Phase = 5;
    servo3StartTime = millis();
    Serial.print("Servo 3: Returning to origin from ");
    Serial.println(servo3TotalPos);
  }

  // Servo 3: Wait for return to complete (phase 5)
  if (servo3Phase == 5)
  {
    // Check if close to origin
    if (abs(servo3TotalPos) < 100 || millis() - servo3StartTime > 10000)
    {
      servo3Phase = 0;
      Serial.print("Servo 3: Back at origin, TotalPos=");
      Serial.println(servo3TotalPos);
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

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

// Load monitoring
#define LOAD_CHECK_INTERVAL 200 // ms
#define LOAD_LIMIT 500          // 0-1000

SMS_STS sts;
M5UnitJoystick2 joystick;

// Previous speed to detect changes
int16_t prevSpeed1 = 0;
int16_t prevSpeed2 = 0;
int prevPWMAngle = -1;

// Servo 3 state:
// 0=idle, 1=reverse(homing), 2=forward(homing complete)
// 3=forward rotation, 4=wait 2sec, 5=reverse rotation, 6=wait before re-home
int servo3Phase = 0;
unsigned long servo3StartTime = 0;
int servo3HomePos = 0;
int servo3LastPos = 0;
int servo3RotationCount = 0;
int servo3ExtraStartPos = 0;
#define SERVO3_INIT_SPEED 500
#define SERVO3_LOAD_LIMIT 300
#define SERVO3_FORWARD_TIME 1000 // ms (homing forward time)
#define SERVO3_ROTATIONS 8       // Number of full rotations for button action
#define SERVO3_EXTRA_STEPS 2800  // Extra steps after rotations (2048 = half rotation, 0 = none)
#define SERVO3_WAIT_TIME 2000    // ms (wait time after rotation)
#define SERVO3_RUN_SPEED 3000    // Wheel mode speed for rotation

// Load monitoring state
unsigned long lastLoadCheck = 0;
bool servoOverLoad[3] = {false, false, false};
int8_t overLoadDirection[3] = {0, 0, 0}; // Direction when overload occurred: 1=positive, -1=negative

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

  // Disable angle limits for Servo 3 (enable multi-turn)
  sts.unLockEprom(SERVO3_ID);
  sts.writeWord(SERVO3_ID, 9, 0); // Min Limit = 0
  sts.writeWord(SERVO3_ID, 10, 0);
  sts.writeWord(SERVO3_ID, 11, 0); // Max Limit = 0
  sts.writeWord(SERVO3_ID, 12, 0);
  sts.LockEprom(SERVO3_ID);
  Serial.println("Servo 3: Angle limits disabled");

  // Set servos to wheel mode (continuous rotation)
  Serial.println("Setting wheel mode...");
  for (int i = 0; i < 3; i++)
  {
    int id = servoIds[i];
    sts.WheelMode(id);
    delay(50);

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

  // Calculate speeds from joystick position
  // X axis (left/right) controls Servo 1
  // Y axis (up/down) controls Servo 2
  int16_t speed1 = -mapJoystickToSpeed(adc_x, SERVO1_SPEED_MAX); // Inverted
  int16_t speed2 = mapJoystickToSpeed(adc_y, SERVO2_SPEED_MAX);

  // Only move the axis with larger offset from center
  int32_t offsetX = abs((int32_t)adc_x - JOY_CENTER);
  int32_t offsetY = abs((int32_t)adc_y - JOY_CENTER);
  if (offsetX > offsetY)
  {
    speed2 = 0;
  }
  else
  {
    speed1 = 0;
  }

  // Update Servo 1 if speed changed (skip if button pressed or same direction as overload)
  bool buttonPressed = (digitalRead(BTN_ANGLE_70) == LOW) || (digitalRead(BTN_ANGLE_110) == LOW);
  if (buttonPressed)
  {
    speed1 = 0;
  }
  // Reset overload flag only when joystick returns to center
  if (speed1 == 0 && servoOverLoad[0])
  {
    servoOverLoad[0] = false;
    overLoadDirection[0] = 0;
  }
  // Block same direction as overload, allow reverse (load sign is opposite to speed)
  if (servoOverLoad[0] && ((speed1 > 0 && overLoadDirection[0] < 0) || (speed1 < 0 && overLoadDirection[0] > 0)))
  {
    speed1 = 0;
  }
  if (speed1 != prevSpeed1)
  {
    sts.WriteSpe(SERVO1_ID, speed1, SERVO_ACC);
    prevSpeed1 = speed1;
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

  // Servo 3: Homing forward complete (phase 2) - switch to position mode
  if (servo3Phase == 2 && millis() - servo3StartTime >= SERVO3_FORWARD_TIME)
  {
    sts.WriteSpe(SERVO3_ID, 0, SERVO_ACC);
    delay(100);

    // Set current position as 0
    sts.CalibrationOfs(SERVO3_ID);
    delay(50);
    servo3HomePos = sts.ReadPos(SERVO3_ID);
    servo3Phase = 0;
    Serial.print("Servo 3: Homing complete, HomePos=");
    Serial.println(servo3HomePos);
  }

  // Servo 3: Joystick button triggers rotation sequence
  // Block if any other action is happening (buttons pressed, servos moving)
  bool anyActivity = buttonPressed || (speed1 != 0) || (speed2 != 0);
  if (servo3Phase == 0 && joystick.get_button_value() == 0 && !anyActivity)
  {
    // Start forward rotation with wheel mode
    servo3RotationCount = 0;
    servo3LastPos = sts.ReadPos(SERVO3_ID);
    sts.WriteSpe(SERVO3_ID, SERVO3_RUN_SPEED, SERVO_ACC);
    servo3Phase = 3;
    servo3StartTime = millis();
    Serial.print("Servo 3: Forward ");
    Serial.print(SERVO3_ROTATIONS);
    Serial.println(" rotations");
  }

  // Servo 3: Count rotations during forward (phase 3)
  if (servo3Phase == 3)
  {
    int currentPos = sts.ReadPos(SERVO3_ID);
    // Detect wrap-around (position jumps from ~4095 to ~0)
    if (servo3LastPos > 3000 && currentPos < 1000)
    {
      servo3RotationCount++;
      Serial.print("Servo 3: Rotation ");
      Serial.println(servo3RotationCount);

      // After full rotations, record position for extra steps
      if (servo3RotationCount == SERVO3_ROTATIONS)
      {
        servo3ExtraStartPos = currentPos;
      }
    }
    servo3LastPos = currentPos;

    // Check if full rotations + extra steps complete
    if (servo3RotationCount >= SERVO3_ROTATIONS)
    {
      int extraMoved = (currentPos - servo3ExtraStartPos + 4096) % 4096;
      if (extraMoved >= SERVO3_EXTRA_STEPS || SERVO3_EXTRA_STEPS == 0)
      {
        sts.WriteSpe(SERVO3_ID, 0, SERVO_ACC);
        servo3Phase = 4;
        servo3StartTime = millis();
        Serial.println("Servo 3: Waiting 2sec");
      }
    }
  }

  // Servo 3: Wait phase (phase 4)
  if (servo3Phase == 4 && millis() - servo3StartTime >= SERVO3_WAIT_TIME)
  {
    // Start reverse rotation
    servo3RotationCount = 0;
    servo3LastPos = sts.ReadPos(SERVO3_ID);
    servo3ExtraStartPos = servo3LastPos; // For extra steps tracking
    sts.WriteSpe(SERVO3_ID, -SERVO3_RUN_SPEED, SERVO_ACC);
    servo3Phase = 5;
    servo3StartTime = millis();
    Serial.println("Servo 3: Reverse");
  }

  // Servo 3: Count rotations during reverse (phase 5)
  // Reverse 1 less rotation, then re-home will bring it back slowly
  if (servo3Phase == 5)
  {
    int currentPos = sts.ReadPos(SERVO3_ID);
    // Detect wrap-around (position jumps from ~0 to ~4095)
    if (servo3LastPos < 1000 && currentPos > 3000)
    {
      servo3RotationCount++;
      Serial.print("Servo 3: Reverse rotation ");
      Serial.println(servo3RotationCount);
    }
    servo3LastPos = currentPos;

    // Stop after (SERVO3_ROTATIONS - 1) rotations, let re-home do the rest
    if (servo3RotationCount >= SERVO3_ROTATIONS - 1)
    {
      // Stop and wait before re-home
      sts.WriteSpe(SERVO3_ID, 0, SERVO_ACC);
      servo3Phase = 6;
      servo3StartTime = millis();
      Serial.println("Servo 3: Waiting before re-home");
    }
  }

  // Servo 3: Wait before re-home (phase 6)
  if (servo3Phase == 6 && millis() - servo3StartTime >= 1000)
  {
    servo3Home();
    Serial.println("Servo 3: Re-homing");
  }

  // Load monitoring every 200ms
  if (millis() - lastLoadCheck >= LOAD_CHECK_INTERVAL)
  {
    lastLoadCheck = millis();
    int servoIds[] = {SERVO1_ID, SERVO2_ID, SERVO3_ID};

    // Show Servo 3 position info
    int servo3Pos = sts.ReadPos(SERVO3_ID);
    Serial.print("S3: Pos=");
    Serial.print(servo3Pos);
    Serial.print(" Home=");
    Serial.print(servo3HomePos);
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
      // Servo 3: Check load to trigger forward rotation
      if (i == 2 && servo3Phase == 1 && abs(load) > SERVO3_LOAD_LIMIT)
      {
        servo3Phase = 2;
        servo3StartTime = millis();
        sts.WriteSpe(SERVO3_ID, SERVO3_INIT_SPEED, SERVO_ACC);
        Serial.println();
        Serial.println("*** Servo 3: Load triggered, Forward 1000ms ***");
      }
      // Servo 1: Overload protection (skip Servo 2)
      else if (i == 0 && abs(load) > LOAD_LIMIT)
      {
        servoOverLoad[i] = true;
        overLoadDirection[i] = (load > 0) ? 1 : -1; // Record direction
        sts.WriteSpe(servoIds[i], 0, SERVO_ACC);
        Serial.println();
        Serial.print("*** Servo ");
        Serial.print(servoIds[i]);
        Serial.print(" OVER LOAD STOP! Load=");
        Serial.print(load);
        Serial.println(" ***");
        prevSpeed1 = 0;
      }
      // Note: overload flag is reset only when joystick returns to center (speed == 0)
    }
    Serial.println();
  }

  delay(20);
}

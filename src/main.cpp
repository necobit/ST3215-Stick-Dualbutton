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

// Speed settings (0-3000)
#define SERVO_SPEED_MAX 1500
#define SERVO_ACC 50

SMS_STS sts;
M5UnitJoystick2 joystick;

// Previous speed to detect changes
int16_t prevSpeed1 = 0;
int16_t prevSpeed2 = 0;
int prevPWMAngle = -1;

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

int16_t mapJoystickToSpeed(int32_t joyValue)
{
  int32_t offset = joyValue - JOY_CENTER;

  // Apply dead zone
  if (abs(offset) < JOY_DEADZONE)
  {
    return 0;
  }

  // Map to speed (-SERVO_SPEED_MAX to +SERVO_SPEED_MAX)
  if (offset > 0)
  {
    offset -= JOY_DEADZONE;
    return map(offset, 0, 32767 - JOY_DEADZONE, 0, SERVO_SPEED_MAX);
  }
  else
  {
    offset += JOY_DEADZONE;
    return map(offset, -(32768 - JOY_DEADZONE), 0, -SERVO_SPEED_MAX, 0);
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
  int servoIds[] = {SERVO1_ID, SERVO2_ID};
  for (int i = 0; i < 2; i++)
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

  // Set servos to wheel mode (continuous rotation)
  Serial.println("Setting wheel mode...");
  for (int i = 0; i < 2; i++)
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

  Serial.println("Initialization complete");
}

void loop()
{
  // PWM servo: 70 when btn3, 110 when btn4, otherwise 90
  int targetAngle = 90;
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
  int16_t speed1 = mapJoystickToSpeed(adc_x);
  int16_t speed2 = mapJoystickToSpeed(adc_y);

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

  // Update Servo 1 if speed changed (skip if button 3 or 4 pressed)
  bool buttonPressed = (digitalRead(BTN_ANGLE_70) == LOW) || (digitalRead(BTN_ANGLE_110) == LOW);
  if (buttonPressed)
  {
    speed1 = 0;
  }
  if (speed1 != prevSpeed1)
  {
    sts.WriteSpe(SERVO1_ID, speed1, SERVO_ACC);
    prevSpeed1 = speed1;

    Serial.print("Servo 1: X=");
    Serial.print(adc_x);
    Serial.print(" -> Speed=");
    Serial.println(speed1);
  }

  // Update Servo 2 if speed changed
  if (speed2 != prevSpeed2)
  {
    sts.WriteSpe(SERVO2_ID, speed2, SERVO_ACC);
    prevSpeed2 = speed2;

    Serial.print("Servo 2: Y=");
    Serial.print(adc_y);
    Serial.print(" -> Speed=");
    Serial.println(speed2);
  }

  delay(20);
}

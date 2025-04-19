#include <Arduino.h>
#include <IBusBM.h>

IBusBM ibus;

// Define the number of channels
const uint8_t NUM_CHANNELS = 10; // Adjust this value based on your system

// Pin Definitions
const uint8_t RELAY1_PIN = 2;
const uint8_t RELAY2_PIN = 3;
const uint8_t MOTOR1_PWM = 5;
const uint8_t MOTOR1_DIR = 6;
const uint8_t MOTOR2_PWM = 9;
const uint8_t MOTOR2_DIR = 10;
// PIN 19 is used for IBUS RX, so we can't use it for PWM its used for RX by the rc receiver

// State variables
bool lightsOn = false;
bool emergencyStop = false;
bool brakesEngaged = false;
int lastSpeed1 = 0;
int lastSpeed2 = 0;
bool switchStates[NUM_CHANNELS] = {false}; // Array to store switch states

// Channel mapping
const uint8_t THROTTLE_CH = 0; // Channel 1
const uint8_t STEERING_CH = 2; // Channel 3
const uint8_t LIGHTS_CH = 4;   // Channel 5
const uint8_t ESTOP_CH = 5;    // Channel 6

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = ibus.readChannel(channelInput);
  return (ch < 100) ? defaultValue : map(ch, 1000, 2000, minLimit, maxLimit);
}

bool readSwitch(byte channelInput, bool defaultValue)
{
  return readChannel(channelInput, 0, 100, defaultValue ? 100 : 0) > 50;
}

void controlMotor(uint8_t pwmPin, uint8_t dirPin, int speed)
{
  if (emergencyStop || brakesEngaged)
    speed = 0;

  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

void updateLights()
{
  bool shouldLightsBeOn = readSwitch(LIGHTS_CH, false);
  if (shouldLightsBeOn != lightsOn)
  {
    digitalWrite(RELAY1_PIN, shouldLightsBeOn ? HIGH : LOW);
    digitalWrite(RELAY2_PIN, shouldLightsBeOn ? HIGH : LOW);
    lightsOn = shouldLightsBeOn;
  }
}

void updateEStop()
{
  emergencyStop = readSwitch(ESTOP_CH, false);
  if (emergencyStop)
  {
    brakesEngaged = true;
    controlMotor(MOTOR1_PWM, MOTOR1_DIR, 0);
    controlMotor(MOTOR2_PWM, MOTOR2_DIR, 0);
  }
}

void applyBrakes()
{
  if (abs(lastSpeed1) > 10 || abs(lastSpeed2) > 10)
  {
    brakesEngaged = true;
    // Active braking (short circuit motors)
    digitalWrite(MOTOR1_DIR, LOW);
    digitalWrite(MOTOR2_DIR, LOW);
    analogWrite(MOTOR1_PWM, 255);
    analogWrite(MOTOR2_PWM, 255);
    delay(50);
    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR2_PWM, 0);
    brakesEngaged = false;
  }
}

void sendTelemetry()
{
  unsigned long ts = millis();
  Serial.print("{");
  // Timestamp
  Serial.print("\"t\":");
  Serial.print(ts);

  // Raw channels as array of objects
  Serial.print(",\"ch_raw\":[");
  for (uint8_t i = 0; i < NUM_CHANNELS; i++)
  {
    Serial.print("{\"ch");
    Serial.print(i + 1);
    Serial.print("\":");
    Serial.print((i >= 4 && i <= 8 ? readChannel(i, -100, 100, 0) : readSwitch(i, false)));
    Serial.print("}");
    if (i < NUM_CHANNELS - 1)
      Serial.print(",");
  }
  Serial.print("]");

  // Switch states
  Serial.print(",\"sw\":[");
  for (uint8_t i = 4; i < 8; i++)
  {
    Serial.print(readSwitch(i, false) ? 1 : 0);
    if (i < 7)
      Serial.print(',');
  }
  Serial.print("]");

  // Lights, estop, brakes
  Serial.print(",\"lights\":");
  Serial.print(lightsOn ? "true" : "false");
  Serial.print(",\"estop\":");
  Serial.print(emergencyStop ? "true" : "false");
  Serial.print(",\"brakes\":");
  Serial.print(brakesEngaged ? "true" : "false");

  // Motor speeds
  Serial.print(",\"motors\":[");
  Serial.print(lastSpeed1);
  Serial.print(',');
  Serial.print(lastSpeed2);
  Serial.print("]");

  // Status
  Serial.print(",\"status\":\"");
  if (emergencyStop)
    Serial.print("EMERGENCY_STOP");
  else if (brakesEngaged)
    Serial.print("BRAKING");
  else if (lastSpeed1 != 0 || lastSpeed2 != 0)
    Serial.print("MOVING");
  else
    Serial.print("IDLE");
  Serial.print("\"");

  Serial.println("}");
}

void setup()
{
  Serial.begin(115200);
  ibus.begin(Serial1);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);

  // Initialize outputs
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
}

void loop()
{
  updateEStop();
  updateLights();

  if (!emergencyStop)
  {
    int throttle = readChannel(THROTTLE_CH, -255, 255, 0);
    int steering = readChannel(STEERING_CH, -255, 255, 0);

    // Mix steering and throttle for differential drive
    int motor1Speed = constrain(throttle + steering, -255, 255);
    int motor2Speed = constrain(throttle - steering, -255, 255);

    // Apply acceleration limiting
    motor1Speed = constrain(motor1Speed, lastSpeed1 - 20, lastSpeed1 + 20);
    motor2Speed = constrain(motor2Speed, lastSpeed2 - 20, lastSpeed2 + 20);

    controlMotor(MOTOR1_PWM, MOTOR1_DIR, motor1Speed);
    controlMotor(MOTOR2_PWM, MOTOR2_DIR, motor2Speed);

    lastSpeed1 = motor1Speed;
    lastSpeed2 = motor2Speed;

    // Auto-brake when centered
    brakesEngaged = (throttle == 0 && steering == 0);
    if (brakesEngaged)
      applyBrakes();
  }

  sendTelemetry();
  delay(20);
}

// Example Serial Telemetry JSON Structure : {
//   "t" : 123456, // Timestamp in ms
//   "ch_raw" : [  // Raw channels as objects
//     {"ch1" : 1500}, {"ch2" : 1490}, {"ch3" : 1520}, ...
//   ],
//   "sw" : [ 0, 1, 0, 1 ], // Switch states for channels 5–8
//   "lights" : true,
//   "estop" : false,
//   "brakes" : false,
//   "motors" : [ 120, -90 ], // Motor1 & Motor2 PWM speeds
//   "status" : "MOVING"
// }

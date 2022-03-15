#include <Arduino.h>
#include "QuadEncoder.h"
#include <PIDController.h>
#include <movingAvg.h>

QuadEncoder myEnc1(1, 5, 4, 0); // Encoder on channel 1 of 4 available
                                // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder myEnc2(2, 2, 3, 0); // Encoder on channel 2 of 4 available
                                // Phase A (pin2), PhaseB(pin3), Pullups Req(0)

long int currentMillis = 0;
long int previousMillis = 0;

long int leftWheel_ticks = 0;
long int rightWheel_ticks = 0;

const int in1 = 9;
const int in2 = 8;

// Motor B connections (right)
// const int enB = 10;
const int in3 = 28;
const int in4 = 29;

int setpoint = 0;

float radius = 0.03; // 3 cm

float wheel_circumference = 2 * 3.1415 * radius;
float tickPerRot = 468.6;
int enc1Val = 0;
int enc2Val = 0;
float rightWheel_mPerS = 0;
float leftWheel_mPerS = 0;
int output = 0;

PIDController rightWheel_pid;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  // speedInterrupt.begin(calcSpeed, 100000);
  Serial.begin(9600);
  Serial.setTimeout(50);
  // while (!Serial && millis() < 4000)
  //   ;

  /* Initialize the ENC module. */
  myEnc1.setInitConfig();
  myEnc1.EncConfig.revolutionCountCondition = ENABLE;
  myEnc1.EncConfig.positionModulusValue = 0;
  myEnc1.init();

  myEnc2.setInitConfig();
  myEnc2.EncConfig.revolutionCountCondition = ENABLE;
  myEnc2.EncConfig.positionInitialValue = 0;
  myEnc2.init();

  // motor pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  rightWheel_pid.begin(); // initialize the PID instance

  rightWheel_pid.tune(24, 8, 20); // Tune the PID, arguments: kP, kI, kD
  rightWheel_pid.limit(0, 255);   // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
}
double dataPoint_reverse[5];
int avgRPM = 0;
void loop()
{

  float sampleTime = 5; // ms

  currentMillis = millis();

  setpoint = 0;
  if (abs(setpoint) > 0 && abs(setpoint) < 100)
  {
    rightWheel_pid.tune(30, 3, 0);
  }
  else if (abs(setpoint) >= 100 && abs(setpoint) < 200)
  {
    rightWheel_pid.tune(35, 4, 0);
  }
  else if (abs(setpoint) >= 200 && abs(setpoint) < 300)
  {
    rightWheel_pid.tune(40, 3.5, 0);
  }
  else if (abs(setpoint) >= 300)
  {
    rightWheel_pid.tune(45, 4, 0);
  }

  if (currentMillis - previousMillis > sampleTime)
  {
    previousMillis = currentMillis;
    enc1Val = myEnc1.read(); // right encoder
    enc2Val = myEnc2.read(); // left encoder
    // rightWheel_mPerS = (wheel_circumference / tickPerRot * enc1Val) * 1 / (sampleTime / 1000);
    // leftWheel_mPerS = (wheel_circumference / tickPerRot * enc2Val) * 1 / (sampleTime / 1000);

    float rpm_right = float(enc1Val) / tickPerRot * 60.0 * 1 / (sampleTime / 1000);
    float rpm_left = float(enc2Val) / tickPerRot * 60.0 * 1 / (sampleTime / 1000);

    rightWheel_ticks = rightWheel_ticks + enc1Val;
    leftWheel_ticks = leftWheel_ticks + enc2Val;

    myEnc1.write(0);
    myEnc2.write(0);

    if (setpoint >= 0)
    {
      for (int i = 1; i < 5; i++)
      {
        dataPoint_reverse[i - 1] = dataPoint_reverse[i];
      }
      dataPoint_reverse[4] = rpm_right;
      for (int i = 0; i < 5; i++)
      {
        avgRPM = avgRPM + dataPoint_reverse[i];
      }
      avgRPM = avgRPM / 5;
      rightWheel_pid.setpoint(setpoint);

      output = rightWheel_pid.compute(avgRPM);

      // analogWrite(in3, int(avgPWM)); // right reverse
      analogWrite(in3, output); // right reverse
      analogWrite(in4, 0);      // right forward
      Serial.print(rightWheel_ticks);
      Serial.print("   -   ");
      Serial.print(setpoint);
      Serial.print("   -   ");
      Serial.print(avgRPM);
      Serial.print("   -   ");
      Serial.print("output ");
      Serial.println(output);
    }

    else if (setpoint < 0)
    {
      for (int i = 1; i < 5; i++)
      {
        dataPoint_reverse[i - 1] = dataPoint_reverse[i];
      }
      dataPoint_reverse[4] = rpm_right;
      for (int i = 0; i < 5; i++)
      {
        avgRPM = avgRPM + dataPoint_reverse[i];
      }
      avgRPM = avgRPM / 5;
      rightWheel_pid.setpoint(abs(setpoint));
      output = rightWheel_pid.compute(abs(avgRPM));
      analogWrite(in3, 0);      // right reverse
      analogWrite(in4, output); // right forward

      Serial.print(setpoint);
      Serial.print("   -   ");
      Serial.print(rpm_right);
      Serial.print("   -   ");
      Serial.print("output ");
      Serial.println(output);
    }
  }
}
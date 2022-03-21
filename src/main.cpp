#include <Arduino.h>
#include "QuadEncoder.h"
#include <PIDController.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Keep track of the number of wheel ticks
std_msgs::Int64 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int64 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

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

PIDController rightWheel_pid;
PIDController leftWheel_pid;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapPwm(float x, float out_min, float out_max)
{
    return x * (out_max - out_min) + out_min;
}

void cmdVelCb(const geometry_msgs::Twist &msg)
{
    float x = msg.linear.x;   // m/s
    float th = msg.angular.z; // rad/s
    float spd_left, spd_right;

    if (x == 0 && th == 0)
    {
        moving = 0;
        // drive.setSpeeds(0, 0);
        return;
    }

    /* Indicate that we are moving */
    moving = 1;

    if (x == 0)
    {
        // Turn in place
        spd_right = th * wheelTrack / 2.0;
        spd_left = -spd_right;
    }
    else if (th == 0)
    {
        // Pure forward/backward motion
        spd_left = spd_right = x;
    }
    else
    {
        // Rotation about a point in space
        spd_left = x - th * wheelTrack / 2.0;
        spd_right = x + th * wheelTrack / 2.0;
    }
}

double dataPoint_reverse[5];
int avgRPM = 0;
uint16_t motor_driver(int16_t setpoint_right, int16_t setpoint_left, float rpm_right, float rpm_left)
{
    int leftWheel_output = 0;
    int rightWheel_output = 0;

    // "Adaptive" PID controller
    if (abs(setpoint_right) > 0 && abs(setpoint_right) < 100)
    {
        rightWheel_pid.tune(30, 3, 0);
    }
    else if (abs(setpoint_right) >= 100 && abs(setpoint_right) < 200)
    {
        rightWheel_pid.tune(35, 4.5, 0);
    }
    else if (abs(setpoint_right) >= 200 && abs(setpoint_right) < 300)
    {
        rightWheel_pid.tune(40, 3.5, 0);
    }
    else if (abs(setpoint_right) >= 300)
    {
        rightWheel_pid.tune(45, 4, 0);
    }

    if (setpoint_right > 0)
    {
        /*
        for (int i = 1; i < 5; i++)
        {
            dataPoint_reverse[i - 1] = dataPoint_reverse[i];
        }
        dataPoint_reverse[4] = rpm;
        for (int i = 0; i < 5; i++)
        {
            avgRPM = avgRPM + dataPoint_reverse[i];
        }
        avgRPM = avgRPM / 5;
        */
        rightWheel_pid.setpoint(setpoint_right);

        rightWheel_output = rightWheel_pid.compute(rpm_right);
    }

    // "Adaptive" PID controller
    if (abs(setpoint_left) > 0 && abs(setpoint_left) < 100)
    {
        leftWheel_pid.tune(30, 3, 0);
    }
    else if (abs(setpoint_left) >= 100 && abs(setpoint_left) < 200)
    {
        leftWheel_pid.tune(35, 4.5, 0);
    }
    else if (abs(setpoint_left) >= 200 && abs(setpoint_left) < 300)
    {
        leftWheel_pid.tune(40, 3.5, 0);
    }
    else if (abs(setpoint_left) >= 300)
    {
        leftWheel_pid.tune(45, 4, 0);
    }

    if (setpoint_left > 0)
    {
        /*
        for (int i = 1; i < 5; i++)
        {
            dataPoint_reverse[i - 1] = dataPoint_reverse[i];
        }
        dataPoint_reverse[4] = rpm;
        for (int i = 0; i < 5; i++)
        {
            avgRPM = avgRPM + dataPoint_reverse[i];
        }
        avgRPM = avgRPM / 5;
        */
        leftWheel_pid.setpoint(setpoint_left);

        leftWheel_output = leftWheel_pid.compute(rpm_left);
    }

    if (setpoint_left > 0)
    {
        analogWrite(in1, leftWheel_output); // left reverse
        analogWrite(in2, 0);                // right forward
    }
    if (setpoint_right > 0)
    {
        analogWrite(in3, rightWheel_output); // right reverse
        analogWrite(in4, 0);                 // right forward
    }
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

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

    leftWheel_pid.begin(); // initialize the PID instance

    leftWheel_pid.tune(24, 8, 20); // Tune the PID, arguments: kP, kI, kD
    leftWheel_pid.limit(0, 255);   // Limit the PID output between 0 and 255, this is important to get rid of integral windup!

    // ROS Setup
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(rightPub);
    nh.advertise(leftPub);
}

void loop()
{

    float sampleTime = 5; // ms

    currentMillis = millis();

    if (currentMillis - previousMillis > sampleTime)
    {
        previousMillis = currentMillis;
        enc1Val = myEnc1.read(); // right encoder
        enc2Val = myEnc2.read(); // left encoder
        // rightWheel_mPerS = (wheel_circumference / tickPerRot * enc1Val) * 1 / (sampleTime / 1000);
        // leftWheel_mPerS = (wheel_circumference / tickPerRot * enc2Val) * 1 / (sampleTime / 1000);

        float rpm_right = float(enc1Val) / tickPerRot * 60.0 * 1 / (sampleTime / 1000);
        float rpm_left = float(enc2Val) / tickPerRot * 60.0 * 1 / (sampleTime / 1000);

        right_wheel_tick_count.data = right_wheel_tick_count.data + enc1Val;
        left_wheel_tick_count.data = left_wheel_tick_count.data + enc2Val;

        rightPub.publish(&right_wheel_tick_count);
        leftPub.publish(&left_wheel_tick_count);

        myEnc1.write(0);
        myEnc2.write(0);
        motor_driver(000, 000, rpm_right, rpm_left);
    }

    nh.spinOnce();
}
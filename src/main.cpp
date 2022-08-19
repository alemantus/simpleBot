#include <Arduino.h>
#include "QuadEncoder.h"
#include <PIDController.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "main.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Keep track of the number of wheel ticks
std_msgs::Int64 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int64 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::Int16 left_wheel_pwm;
ros::Publisher leftPWMPub("left_pwm", &left_wheel_pwm);

std_msgs::Int16 right_wheel_pwm;
ros::Publisher rightPWMPub("right_pwm", &right_wheel_pwm);

std_msgs::Float32 bat_voltage;
ros::Publisher batVoltagePub("bat/voltage", &bat_voltage);

std_msgs::Float32 bat_current;
ros::Publisher batCurrentPub("bat/current", &bat_current);

QuadEncoder myEnc1(1, 4, 5, 0); // Encoder on channel 1 of 4 available
                                // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder myEnc2(2, 3, 2, 0); // Encoder on channel 2 of 4 available
                                // Phase A (pin2), PhaseB(pin3), Pullups Req(0)

PIDController rightWheel_pid;
PIDController leftWheel_pid;

SetPointInfo leftPID, rightPID;

void setSpeed(int leftMotor, int rightMotor)
{
    rightPWMPub.publish(rightMotor);
    leftPWMPub.publish(leftMotor);

    if (leftMotor < 0 && rightMotor < 0)
    {
        analogWrite(in1, abs(leftMotor)); // left reverse
        analogWrite(in2, 0);
        analogWrite(in3, abs(rightMotor)); // right reverse
        analogWrite(in4, 0);               // right forward
    }
    else if (leftMotor >= 0 && rightMotor < 0)
    {
        analogWrite(in1, 0); // left reverse
        analogWrite(in2, leftMotor);
        analogWrite(in3, abs(rightMotor)); // right reverse
        analogWrite(in4, 0);               // right forward
    }
    else if (leftMotor < 0 && rightMotor >= 0)
    {
        analogWrite(in1, abs(leftMotor)); // left reverse
        analogWrite(in2, 0);
        analogWrite(in3, 0);          // right reverse
        analogWrite(in4, rightMotor); // right forward
    }
    else if (leftMotor >= 0 && rightMotor >= 0)
    {
        analogWrite(in1, 0); // left reverse
        analogWrite(in2, leftMotor);
        analogWrite(in3, 0);          // right reverse
        analogWrite(in4, rightMotor); // right forward
    }
}

/* Convert meters per second to ticks per time frame */
int SpeedToTicks(float v)
{
    return int(v * tickPerRot / (PID_RATE * 3.1415926535897932384626433832795 * wheelDiameter));
}

void cmdVelCb(const geometry_msgs::Twist &msg)
{
    float x = msg.linear.x;   // m/s
    float th = msg.angular.z; // rad/s
    float spd_left, spd_right;

    /* Reset the auto stop timer */
    // lastMotorCommand = millis();

    if (x == 0 && th == 0)
    {
        moving = 0;
        // drive.setSpeeds(0, 0);
        setSpeed(0, 0);
        return;
    }

    /* Indicate that we are moving */
    moving = 1;

    if (x == 0)
    {
        // Turn in place
        spd_right = th * baseWidth / 2.0;
        spd_left = -spd_right;
    }
    else if (th == 0)
    {
        // Pure forward/backward motion
        spd_left = x;
        spd_right = x;
    }
    else
    {
        // Rotation about a point in space
        spd_left = x - th * baseWidth / 2.0;
        spd_right = x + th * baseWidth / 2.0;
    }

    /* Set the target speeds in meters per second */
    leftPID.TargetSpeed = spd_left;
    rightPID.TargetSpeed = spd_right;

    /* Convert speeds to encoder ticks per frame */
    leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
    rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);
}
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo *p)
{
    long Perror;
    long output;

    Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);

    // Derivative error is the delta Perror
    output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
    p->PrevErr = Perror;
    p->PrevEnc = p->Encoder;

    output += p->output;

    if (output >= MAXOUTPUT)
        output = MAXOUTPUT;
    else if (output <= -MAXOUTPUT)
        output = -MAXOUTPUT;
    else
        p->Ierror += Perror;

    p->output = output;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
    /* Read the encoders */
    leftPID.Encoder = myEnc2.read();  // left encoder
    rightPID.Encoder = myEnc1.read(); // right encoder

    /* Record the time that the readings were taken */
    // odomInfo.encoderTime = millis();
    // odomInfo.encoderStamp = nh.now();

    /* If we're not moving there is nothing more to do */
    if (!moving)
        return;

    /* Compute PID update for each motor */
    doPID(&leftPID);
    doPID(&rightPID);

    /* Set the motor speeds accordingly */
    // drive.setSpeeds(leftPID.output, rightPID.output);
    setSpeed(leftPID.output, rightPID.output);
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &cmdVelCb);

void setup()
{
    // Ensures ROS is running before we move on
    while (!nh.connected())
    {
        nh.spinOnce();
    }

    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
        while (1)
        {
            delay(10);
        }
    }
    ina219.setCalibration_32V_1A();
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
    /*
        rightWheel_pid.begin(); // initialize the PID instance

        rightWheel_pid.tune(24, 8, 20); // Tune the PID, arguments: kP, kI, kD
        rightWheel_pid.limit(0, 255);   // Limit the PID output between 0 and 255, this is important to get rid of integral windup!

        leftWheel_pid.begin(); // initialize the PID instance

        leftWheel_pid.tune(24, 8, 20); // Tune the PID, arguments: kP, kI, kD
        leftWheel_pid.limit(0, 255);   // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
    */
    // ROS Setup
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.advertise(rightPub);
    nh.advertise(leftPub);
    nh.advertise(rightPWMPub);
    nh.advertise(leftPWMPub);
    nh.advertise(batVoltagePub);
    nh.advertise(batCurrentPub);
    nh.subscribe(subCmdVel);
}

void loop()
{
    // float sampleTime = 5; // ms

    // currentMillis = millis();

    // Get PID params or set default value
    // nh.getParam("/rosparam/Ki", Ki);
    // nh.getParam("/rosparam/Kd", Kd);
    // nh.getParam("/rosparam/Kp", Kp);
    // nh.getParam("/rosparam/Ko", Ko);
    nh.getParam("/rosparam/Kp", &Kp, 1);
    nh.getParam("/rosparam/Ki", &Ki, 1);
    nh.getParam("/rosparam/Kd", &Kd, 1);
    nh.getParam("/rosparam/Ko", &Ko, 1);
    nh.getParam("/rosparam/PID", &PID_INTERVAL, 1);

    /*
    if (!nh.getParam("/rosparam/Kp", &Kp, 1))
    {
        Kp = 20;
        // nh.setParam("rosparam/Kp", 20);
    }
    if (!nh.getParam("/rosparam/Ki", &Ki, 1))
    {
        // nh.setParam("rosparam/Ki", 0);
        Ki = 0;
    }
    if (!nh.getParam("/rosparam/Kd", &Kd, 1))
    {
        // nh.setParam("rosparam/Kd", 0);
        Kd = 0;
    }
    if (!nh.getParam("/rosparam/Ko", &Ko, 1))
    {
        // nh.setParam("rosparam/K0", 120);
        Ko = 120;
    }
    */
    bat_voltage.data = ina219.getBusVoltage_V();
    bat_current.data = ina219.getPower_mW();

    // Publish tick counter for odom
    if (millis() > nextOdom)
    {
        right_wheel_tick_count.data = myEnc1.read(); // right encoder
        left_wheel_tick_count.data = myEnc2.read();  // left encoder
        rightPub.publish(&right_wheel_tick_count);
        leftPub.publish(&left_wheel_tick_count);
        batVoltagePub.publish(&bat_voltage);
        batCurrentPub.publish(&bat_current);
        nextOdom += ODOM_INTERVAL;
    }

    // Update PID controller and motor speed
    if (millis() > nextPID)
    {
        updatePID();
        nextPID += PID_INTERVAL;
    }

    // spin ROS
    nh.spinOnce();
}
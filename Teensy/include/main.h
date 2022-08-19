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
float wheelDiameter = 0.06;
float tickPerRot = 468.6;
float baseWidth = 0.22;
float ticksPerMeter = tickPerRot / (PI * wheelDiameter);

/* PID Parameters */
// int Kp = 5;
// int Kd = 0;
// int Ki = 2;
float Kp = 0;
float Kd = 0;
float Ki = 0;
float Ko = 0;

int enc1Val = 0;
int enc2Val = 0;
float rightWheel_mPerS = 0;
float leftWheel_mPerS = 0;

double dataPoint_reverse[5];
int avgRPM = 0;

/* Setpoint Info For a Motor */
typedef struct
{
    double TargetSpeed;         // target speed in m/s
    double TargetTicksPerFrame; // target speed in ticks per frame
    long Encoder;               // encoder count
    long PrevEnc;               // last encoder count
    int PrevErr;                // last error
    int Ierror;                 // integrated error
    int output;                 // last motor setting
} SetPointInfo;

// Odometry publishing rate
#define ODOM_RATE 10 // Hz
const float ODOM_INTERVAL = 1000.0 / ODOM_RATE;

/* Rate at which encoders are sampled and PID loop is updated */
#define PID_RATE 10 // Hz
const float PID_INTERVAL = 1000.0 / PID_RATE;

/* Counters to track update rates for PID and Odometry */
unsigned long nextPID = 0;
unsigned long nextOdom = 0;

int moving = 0;
/* Maximum value for a PWM signal */
#define MAXOUTPUT 255
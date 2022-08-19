/*
 * Automatic Addison
 * https://automaticaddison.com/how-to-publish-wheel-odometry-information-over-ros/
 * Date: May 20, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */

// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

#include <ros/console.h>

// Create odometry data publishers
ros::Publisher odom_pub;
ros::Publisher pub_quat;
nav_msgs::Odometry newOdom;
nav_msgs::Odometry oldOdom;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot constraints
// Updated for simpleBot
const double TICKS_PER_REVOLUTION = 468.6; // For reference purposes.
const double WHEEL_RADIUS = 0.03;          // Wheel radius in meters

// Should probably be measured instead theoretical values
const double WHEEL_BASE = 0.20; // Center of left tire to center of right tire
const double TICKS_PER_METER = 2486;

// Distance both wheels have traveled
double leftDistance = 0;
double rightDistance = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;

// Get initial_2d message from either Rviz clicks or a manual pose publisher
void setInitialPose(const geometry_msgs::PoseStamped &rvizClick)
{
    oldOdom.pose.pose.position.x = rvizClick.pose.position.x;
    oldOdom.pose.pose.position.y = rvizClick.pose.position.y;
    oldOdom.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    initialPoseRecieved = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int64 &leftCount)
{

    static int lastCountL = 0;
    if (leftCount.data != 0 && lastCountL != 0)
    {

        int leftTicks = (leftCount.data - lastCountL);
        leftDistance = leftTicks / TICKS_PER_METER;
    }
    lastCountL = leftCount.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int64 &rightCount)
{

    static int lastCountR = 0;
    if (rightCount.data != 0 && lastCountR != 0)
    {

        int rightTicks = rightCount.data - lastCountR;
        rightDistance = rightTicks / TICKS_PER_METER;
    }
    lastCountR = rightCount.data;
}
void publish_quat()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, newOdom.pose.pose.orientation.z);

    nav_msgs::Odometry quatOdom;
    quatOdom.header.stamp = newOdom.header.stamp;
    quatOdom.header.frame_id = "odom";
    quatOdom.child_frame_id = "base_link";
    quatOdom.pose.pose.position.x = newOdom.pose.pose.position.x;
    quatOdom.pose.pose.position.y = newOdom.pose.pose.position.y;
    quatOdom.pose.pose.position.z = newOdom.pose.pose.position.z;
    quatOdom.pose.pose.orientation.x = q.x();
    quatOdom.pose.pose.orientation.y = q.y();
    quatOdom.pose.pose.orientation.z = q.z();
    quatOdom.pose.pose.orientation.w = q.w();
    quatOdom.twist.twist.linear.x = newOdom.twist.twist.linear.x;
    quatOdom.twist.twist.linear.y = newOdom.twist.twist.linear.y;
    quatOdom.twist.twist.linear.z = newOdom.twist.twist.linear.z;
    quatOdom.twist.twist.angular.x = newOdom.twist.twist.angular.x;
    quatOdom.twist.twist.angular.y = newOdom.twist.twist.angular.y;
    quatOdom.twist.twist.angular.z = newOdom.twist.twist.angular.z;

    for (int i = 0; i < 36; i++)
    {
        if (i == 0 || i == 7 || i == 14)
        {
            quatOdom.pose.covariance[i] = .01;
        }
        else if (i == 21 || i == 28 || i == 35)
        {
            quatOdom.pose.covariance[i] += .165;
        }
        else
        {
            quatOdom.pose.covariance[i] = 0;
        }
    }

    pub_quat.publish(quatOdom);
}

void update_odom()
{
    //average distance
    double cycleDistance = (rightDistance + leftDistance) / 2;
    //how many radians robot has turned since last cycle
    double cycleAngle = asin((rightDistance - leftDistance) / WHEEL_BASE);

    //average angle during last cycle
    double avgAngle = cycleAngle / 2 + oldOdom.pose.pose.orientation.z;
    if (avgAngle > PI)
    {
        avgAngle -= 2 * PI;
    }
    else if (avgAngle < -PI)
    {
        avgAngle += 2 * PI;
    }

    //calculate new x, y, and theta
    newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x + cos(avgAngle) * cycleDistance;
    newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y + sin(avgAngle) * cycleDistance;
    newOdom.pose.pose.orientation.z = cycleAngle + oldOdom.pose.pose.orientation.z;

    //prevent lockup from a single erroneous cycle
    if (isnan(newOdom.pose.pose.position.x) || isnan(newOdom.pose.pose.position.y) || isnan(newOdom.pose.pose.position.z))
    {
        newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x;
        newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y;
        newOdom.pose.pose.orientation.z = oldOdom.pose.pose.orientation.z;
    }

    //keep theta in range proper range
    if (newOdom.pose.pose.orientation.z > PI)
    {
        newOdom.pose.pose.orientation.z -= 2 * PI;
    }
    else if (newOdom.pose.pose.orientation.z < -PI)
    {
        newOdom.pose.pose.orientation.z += 2 * PI;
    }

    //calculate velocity
    newOdom.header.stamp = ros::Time::now();
    newOdom.twist.twist.linear.x = cycleDistance / (newOdom.header.stamp.toSec() - oldOdom.header.stamp.toSec());
    newOdom.twist.twist.angular.z = cycleAngle / (newOdom.header.stamp.toSec() - oldOdom.header.stamp.toSec());

    //save odom x, y, and theta for use in next cycle
    oldOdom.pose.pose.position.x = newOdom.pose.pose.position.x;
    oldOdom.pose.pose.position.y = newOdom.pose.pose.position.y;
    oldOdom.pose.pose.orientation.z = newOdom.pose.pose.orientation.z;
    oldOdom.header.stamp = newOdom.header.stamp;

    //publish odom message
    odom_pub.publish(newOdom);
}

int main(int argc, char **argv)
{
    //set fixed data fields
    newOdom.header.frame_id = "odom";
    newOdom.pose.pose.position.z = 0;
    newOdom.pose.pose.orientation.x = 0;
    newOdom.pose.pose.orientation.y = 0;
    newOdom.twist.twist.linear.x = 0;
    newOdom.twist.twist.linear.y = 0;
    newOdom.twist.twist.linear.z = 0;
    newOdom.twist.twist.angular.x = 0;
    newOdom.twist.twist.angular.y = 0;
    newOdom.twist.twist.angular.z = 0;
    oldOdom.pose.pose.position.x = initialX;
    oldOdom.pose.pose.position.y = initialY;
    oldOdom.pose.pose.orientation.z = initialTheta;

    //handshake with ros master and create node object
    ros::init(argc, argv, "encoder_odom_publisher");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, setInitialPose);

    //advertise publisher of simpler odom msg where orientation.z is an euler angle
    odom_pub = node.advertise<nav_msgs::Odometry>("encoder/odom", 100);

    //advertise publisher of full odom msg where orientation is quaternion
    pub_quat = node.advertise<nav_msgs::Odometry>("encoder/odom_quat", 100);

    ros::Rate loop_rate(30);
    while (ros::ok)
    {
        ros::spinOnce();
        if (initialPoseRecieved)
        {
            update_odom();
            publish_quat();
        }
        loop_rate.sleep();
    }

    return 0;
}
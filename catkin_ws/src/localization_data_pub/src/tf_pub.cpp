#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "std_msgs/Float32.h"

using namespace std;
/*
//callback function that broadcasts odom message data as a transform
//this odom message should contain an euler angle in the orientation z data member
void handle_odom(const nav_msgs::Odometry &odom)
{
    //create broadcater object
    static tf2_ros::TransformBroadcaster br;
    //create transform object
    tf2_ros::Transform odom_base_tf;

    //set transform x, y location with data from odom message
    odom_base_tf.setOrigin(tf2_ros::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
    //create quaternion object from euler data received in odom
    //tf2_ros::Quaternion Quaternion;

    //add quaternion data to transform object
    odom_base_tf.setRotation(tf2_ros::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                                 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
    //odom_base_tf.setRotation(Quaternion);

    //broadcast transform
    br.sendTransform(tf2_ros::StampedTransform(odom_base_tf, odom.header.stamp, "odom", "base_link"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle node;

    ros::Subscriber subOdom = node.subscribe("encoder/odom_quat", 10, handle_odom);

    ros::Rate loop_rate(30);
    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
*/

void handle_odom(const nav_msgs::Odometry &odom)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "simpleBot";
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    //q.setRPY(0, 0, msg->theta);
    transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;

    br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle node;

    ros::Subscriber subOdom = node.subscribe("encoder/odom_quat", 10, handle_odom);

    ros::Rate loop_rate(30);
    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};
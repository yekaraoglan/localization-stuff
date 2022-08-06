#pragma once

#include <string>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

class WheelOdom
{
    private:
        double WHEEL_RADIUS;
        std::string PARENT_FRAME_ID;
        std::string CHILD_FRAME_ID;
        std::string OUT_ODOM_TOPIC;
        std::string IN_IMU_TOPIC;
        std::string IN_WHEEL_RPM_TOPIC;

        ros::Subscriber sub_imu;
        ros::Subscriber sub_wheels;
        tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Quaternion q;
        nav_msgs::Odometry odom;
        double yaw;
        geometry_msgs::Pose2D pose;
        geometry_msgs::Twist twist;
        sensor_msgs::Imu imu_start;
        sensor_msgs::Imu imu_current;
        std_msgs::Float64MultiArray wheel_rpms;
        std_msgs::Float64MultiArray wheel_speeds;
        std_msgs::Bool is_mecanum;        

    public:
        ros::NodeHandle nh;
        WheelOdom(ros::NodeHandle&);
        ~WheelOdom();
        
        ros::Publisher pub_odom;
        void initializeSubscribers();
        void initializePublishers();
        void imu_cb (const sensor_msgs::Imu::ConstPtr&);
        void wheel_rpm_cb (const std_msgs::Float64MultiArray::ConstPtr&);
        
        void setRadius(double radius);
        void setParentFrameId(std::string frame_id);
        void setChildFrameId(std::string frame_id);
        void setOutOdomTopic(std::string topic);
        void setInImuTopic(std::string topic);
        void setInWheelRpmTopic(std::string topic);

        double getRadius();
        std::string getParentFrameId();
        std::string getChildFrameId();
        std::string getOutOdomTopic();
        std::string getInImuTopic();
        std::string getInWheelRpmTopic();
        
        void updateWheelSpeeds();
        void updateYaw();
        void updateTwists();
        void updatePoses(double);
        void publishOdom();
        void publishTf();
}; 


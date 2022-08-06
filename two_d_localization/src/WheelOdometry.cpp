#include "two_d_localization/WheelOdometry.h"

WheelOdom::WheelOdom(ros::NodeHandle& nh)
{
    this->nh = nh;
    this->nh.param<double>("/wheel_odometry/wheel_radius", WHEEL_RADIUS, 0.0);
    if (WHEEL_RADIUS <= 0.0)
    {
        ROS_ERROR("wheel_radius is not set");
        ros::shutdown();
    }
    this->nh.param<std::string>("/wheel_odometry/parent_frame_id", PARENT_FRAME_ID, "base_link");
    this->nh.param<std::string>("/wheel_odometry/child_frame_id", CHILD_FRAME_ID, "odom");
    this->nh.param<std::string>("/wheel_odometry/out_odom_topic", OUT_ODOM_TOPIC, "odom");
    this->nh.param<std::string>("/wheel_odometry/in_imu_topic", IN_IMU_TOPIC, "imu");
    this->nh.param<std::string>("/wheel_odometry/in_wheel_rpm_topic", IN_WHEEL_RPM_TOPIC, "wheel_rpm");
    
    ROS_INFO("Wheel Odometry Object is Constructed.");

    initializeSubscribers();
    initializePublishers();
}

WheelOdom::~WheelOdom()
{
    ROS_INFO("Wheel Odometry Object is Destructed.");
}

void WheelOdom::initializeSubscribers()
{
    sub_imu = nh.subscribe(IN_IMU_TOPIC, 1, &WheelOdom::imu_cb, this);
    sub_wheels = nh.subscribe(IN_WHEEL_RPM_TOPIC, 1, &WheelOdom::wheel_rpm_cb, this);
    ROS_INFO("Subscribers are initialized.");
}

void WheelOdom::initializePublishers()
{
    pub_odom = nh.advertise<nav_msgs::Odometry>(OUT_ODOM_TOPIC, 1);
    ROS_INFO("Publishers are initialized.");
}

void WheelOdom::imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    if (ros::Time::now().toSec() - imu_msg->header.stamp.toSec() < 1.0) {imu_start = *imu_msg;}
    else { imu_current = *imu_msg; }
}

void WheelOdom::wheel_rpm_cb(const std_msgs::Float64MultiArray::ConstPtr& wheel_rpm_msg)
{
    wheel_rpms = *wheel_rpm_msg;
}

void WheelOdom::setRadius(double radius)
{
    WHEEL_RADIUS = radius;
    ROS_INFO("Radius is set to %f.", WHEEL_RADIUS);
}

void WheelOdom::setParentFrameId(std::string frame_id)
{
    PARENT_FRAME_ID = frame_id;
    ROS_INFO("Parent Frame Id is set to %s.", PARENT_FRAME_ID.c_str());
}

void WheelOdom::setChildFrameId(std::string frame_id)
{
    CHILD_FRAME_ID = frame_id;
    ROS_INFO("Child Frame Id is set to %s.", CHILD_FRAME_ID.c_str());
}

void WheelOdom::setOutOdomTopic(std::string topic)
{
    OUT_ODOM_TOPIC = topic;
    ROS_INFO("Out Odometry Topic is set to %s.", OUT_ODOM_TOPIC.c_str());
}

void WheelOdom::setInImuTopic(std::string topic)
{
    IN_IMU_TOPIC = topic;
    ROS_INFO("In Imu Topic is set to %s.", IN_IMU_TOPIC.c_str());
}

void WheelOdom::setInWheelRpmTopic(std::string topic)
{
    IN_WHEEL_RPM_TOPIC = topic;
    ROS_INFO("In Wheel RPM Topic is set to %s.", IN_WHEEL_RPM_TOPIC.c_str());
}

double WheelOdom::getRadius()
{
    return WHEEL_RADIUS;
}

std::string WheelOdom::getParentFrameId()
{
    return PARENT_FRAME_ID;
}

std::string WheelOdom::getChildFrameId()
{
    return CHILD_FRAME_ID;
}

std::string WheelOdom::getOutOdomTopic()
{
    return OUT_ODOM_TOPIC;
}

std::string WheelOdom::getInImuTopic()
{
    return IN_IMU_TOPIC;
}

std::string WheelOdom::getInWheelRpmTopic()
{
    return IN_WHEEL_RPM_TOPIC;
}

void WheelOdom::updateWheelSpeeds(){
    for (int i=0; i<4; i++){
        wheel_speeds.data[i] = wheel_rpms.data[i] * 2 * M_PI * WHEEL_RADIUS / 60;
    }
}

void WheelOdom::updateYaw(){
    geometry_msgs::Quaternion qt = imu_current.orientation;
    tf::Quaternion q(qt.x, qt.y, qt.z, qt.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, this->yaw);
}

void WheelOdom::updateTwists(){
    twist.linear.x = (wheel_speeds.data[0] + wheel_speeds.data[1] + wheel_speeds.data[2] + wheel_speeds.data[3])/4;
    twist.angular.z = (wheel_speeds.data[0] + wheel_speeds.data[1] + wheel_speeds.data[2] + wheel_speeds.data[3])/4;
    if (is_mecanum.data) twist.linear.y = (wheel_speeds.data[0] + wheel_speeds.data[1] + wheel_speeds.data[2] + wheel_speeds.data[3])/4;
}

void WheelOdom::updatePoses(double dt){
    pose.x = twist.linear.x * cos(this->yaw) * dt;
    if (is_mecanum.data) pose.x -= twist.linear.y * sin(this->yaw) * dt;
    pose.y = twist.linear.x * sin(this->yaw) * dt;
    if (is_mecanum.data) pose.y += twist.linear.y * cos(this->yaw) * dt;
}

void WheelOdom::publishOdom(){
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = PARENT_FRAME_ID;
    odom.child_frame_id = CHILD_FRAME_ID;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    odom.twist.twist.linear.x = twist.linear.x;
    odom.twist.twist.angular.z = twist.angular.z;
    pub_odom.publish(odom);
}

void WheelOdom::publishTf(){
    tf::Transform transform;
    tf::Quaternion qt;
    transform.setOrigin(tf::Vector3(pose.x, pose.y, 0.0));
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
    quaternionMsgToTF(q , qt);
    transform.setRotation(qt);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), PARENT_FRAME_ID, CHILD_FRAME_ID));
}
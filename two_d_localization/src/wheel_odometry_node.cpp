#include "two_d_localization/WheelOdometry.h"

void SigintHandler(int sig)
{
    ROS_INFO("Shutting down...");
    ros::shutdown();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "wheel_odom", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);
    
    WheelOdom wh(nh);
    ros::spin();
}


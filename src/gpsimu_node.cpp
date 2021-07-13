#include <ros/ros.h> 
#include "jy-gpsimu/gpsimu_driver.h"

int main(int argc, char** argv) { 
    // ------------------------------------------------------
    ros::init(argc, argv, "gpsImuAPI"); 
    ros::Time::init();
    ros::Rate loop_rate(50); // [hz]

    sensor::jyGpsImu gps_IMU_port;
    bool port_status = gps_IMU_port.serialConnect();
    while(ros::ok() && port_status)
    {
        gps_IMU_port.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ------------------------------------------------------
    return 0; 
} 
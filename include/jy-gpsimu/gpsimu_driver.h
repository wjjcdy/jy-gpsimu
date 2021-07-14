#ifndef GPSIMU_DRIVER_H
#define GPSIMU_DRIVER_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <deque>

#include "serial_port.hpp"
#include "conversions.h"
namespace sensor
{
    typedef struct {
        float x;
        float y;
        float z;
    } Acc_t;

    typedef struct {
        float x;
        float y;
        float z;
    } Angle_t;

    typedef struct {
        float x;
        float y;
        float z;
        float w;
    } Quaternions_t;

    typedef struct {
        float x;
        float y;
        float z;
    } AngleSpeed_t;

    typedef struct {
        float longitude;
        float lattitude;
        float height;
        float yaw;
        float velocity;
    } Gps_t;

    typedef struct {
        float x;
        float y;
        float z;
        float temperature;
    } Magnetic_t;

    class jyGpsImu {
        public:
            jyGpsImu(void);
            ~jyGpsImu(void);
            int run(void);
            bool serialConnect(void);

        private:
            void DecodeData(const char chrBuf[]);
            void ParseData(const char chrBuf[], int length); 
            void imuPulish(void);
            void gpsPulish(void);
        private:
            ros::Publisher IMU_pub_;
            ros::Publisher GPS_pub_;
            ros::Publisher odom_UTM_pub_;
            int rsp_ret_;
            robot_ctrl::SerialPort sp_motor_;
            Acc_t acc_xyz_;
            AngleSpeed_t w_xyz_;
            Angle_t angle_;
            geometry_msgs::Quaternion qrientation_;
            
            Quaternions_t orientation_;
            Gps_t pose_gps_;
            Magnetic_t magnetic_xyz_;
            std::string gps_port_;
            std::deque<char> rec_buf_;

            int rx_buffer_leng_;
            
    };
}
#endif
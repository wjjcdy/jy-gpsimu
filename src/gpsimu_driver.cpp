#include <ros/ros.h> 
#include <iostream>
#include "jy-gpsimu/gpsimu_driver.h"
#include "tf/transform_datatypes.h"//转换函数头文件
namespace sensor
{
    // 构建
    jyGpsImu::jyGpsImu(void)
    {
        // ros::NodeHandle param_n("~");

        // parameters init
        // param_n.param<std::string>("serial_port", gps_port_, "/dev/ttyUSB0");
        ros::NodeHandle n; 
        IMU_pub_ = n.advertise<sensor_msgs::Imu>("IMU_data", 60);
        GPS_pub_ = n.advertise<sensor_msgs::NavSatFix>("GPS_data",60);
        odom_UTM_pub_ = n.advertise<nav_msgs::Odometry>("GPS_odom_data",60);
    }
    // 析构
    jyGpsImu::~jyGpsImu(void) {
        sp_motor_.closeSerialport();
    }

    bool jyGpsImu::serialConnect(void) {
        bool status = sp_motor_.openSerialPort("/dev/ttyUSB0",115200);
        if(!status) {
            std::cout<<"serial port open failed"<<std::endl;
            ROS_ERROR("serial port: open failed!\n");
            return false;
        }
        else {
            std::cout<<"serial port open success"<<std::endl;
        }
        return true;
    }

    int jyGpsImu::run(void) {
        char rec_buff[100];
        int rec_num = sp_motor_.readSerialPort(rec_buff,44,0);
        if(rec_num>0) {
            for(int i=0;i<rec_num;++i) {
                ParseData(rec_buff[i]);
            }
        }
    }

    void jyGpsImu::imuPulish(void) {
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        //四元数位姿,所有数据设为固定值，可以自己写代码获取ＩＭＵ的数据，，然后进行传递
        imu_data.orientation = qrientation_;

        //线加速度
        imu_data.linear_acceleration.x = acc_xyz_.x; 
        imu_data.linear_acceleration.y = acc_xyz_.y;
        imu_data.linear_acceleration.z = acc_xyz_.z;
        //角速度
        imu_data.angular_velocity.x = w_xyz_.x; 
        imu_data.angular_velocity.y = w_xyz_.y; 
        imu_data.angular_velocity.z = w_xyz_.z;

        if(IMU_pub_.getNumSubscribers()>0) {
            IMU_pub_.publish(imu_data);
        }
    }

    void jyGpsImu::gpsPulish(void) {
        sensor_msgs::NavSatFix navsatfix;
        navsatfix.header.stamp = ros::Time::now();
        navsatfix.header.frame_id = "base_link";
        navsatfix.longitude = pose_gps_.longitude;
        navsatfix.latitude = pose_gps_.lattitude;
        navsatfix.altitude = pose_gps_.height;
        if(GPS_pub_.getNumSubscribers()>0) {
            GPS_pub_.publish(navsatfix);
        }   

        double northing, easting;
        std::string zone;

        gps_common::LLtoUTM(navsatfix.longitude, navsatfix.longitude, northing, easting, zone);
        nav_msgs::Odometry odom;
        odom.header.stamp = navsatfix.header.stamp;
        odom.header.frame_id = "odom_gps";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = easting;
        odom.pose.pose.position.y = northing;
        if(odom_UTM_pub_.getNumSubscribers()>0) {
            odom_UTM_pub_.publish(odom);
        }
    }

    void jyGpsImu::ParseData(char chr)
    {
        static char chrBuf[100];
        static unsigned char chrCnt=0;
        signed short sData[4];
        unsigned char i;
        
        time_t now;
        chrBuf[chrCnt++]=chr;
        if (chrCnt<11) return;
        
        if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)) 
            { 
                printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);
                std::cout<<""<<std::endl;
                memcpy(&chrBuf[0],&chrBuf[1],10);
                chrCnt--;
                return;
            }
        
            memcpy(&sData[0],&chrBuf[2],8);
            switch(chrBuf[1])
            {
                    case 0x51:
                        acc_xyz_.x = (float)sData[0]/32768.0*4.0;
                        acc_xyz_.y = (float)sData[1]/32768.0*4.0;
                        acc_xyz_.z = (float)sData[2]/32768.0*4.0;
                        // time(&now);
                        printf("\r\nT:%sa:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),acc_xyz_.x,acc_xyz_.y,acc_xyz_.z);
                        // std::cout<<""<<std::endl;
                        break;
                    case 0x52:
                        w_xyz_.x = (float)sData[0]/32768.0*2000.0*M_PI/180.0;
                        w_xyz_.y = (float)sData[1]/32768.0*2000.0*M_PI/180.0;
                        w_xyz_.z = (float)sData[2]/32768.0*2000.0*M_PI/180.0;
                        // printf("w:%7.3f %7.3f %7.3f ",w_xyz_.x,w_xyz_.y,w_xyz_.z);
                        // std::cout<<""<<std::endl;					
                        break;
                    case 0x53:
                        angle_.x = (float)sData[0]/32768.0*M_PI;
                        angle_.y = (float)sData[1]/32768.0*M_PI;
                        angle_.z = (float)sData[2]/32768.0*M_PI;
                        // printf("A:%7.3f %7.3f %7.3f ",angle_.x,angle_.y,angle_.z);
                        // std::cout<<""<<std::endl;
                        qrientation_ = tf::createQuaternionMsgFromRollPitchYaw(angle_.x,angle_.y,angle_.z);
                        imuPulish();
                        break;
                    case 0x54:
                        magnetic_xyz_.x = (float)sData[0];
                        magnetic_xyz_.y = (float)sData[1];
                        magnetic_xyz_.z = (float)sData[2];
                        printf("h:%4.0f %4.0f %4.0f ",magnetic_xyz_.x ,magnetic_xyz_.y,magnetic_xyz_.z);
                        std::cout<<""<<std::endl;
                        break;
                    case 0x57:
                        int32_t Data[2];
                        memcpy(&Data[0],&chrBuf[2],8);
                        pose_gps_.longitude = (float)Data[0]/10000000.0;
                        pose_gps_.lattitude = (float)Data[1]/10000000.0;
                        // printf("gps:%f %f ",pose_gps_.longitude ,pose_gps_.lattitude);
                        // std::cout<<""<<std::endl;
                        gpsPulish(); 
                        break;
                    case 0x58:
                        pose_gps_.height = (float)sData[0]/10.0;
                        pose_gps_.yaw = (float)sData[1]/100.0;
                        pose_gps_.velocity = (float)(sData[2]*65536+sData[3])/1000.0;
                        printf("height:%f, yaw:%f, gps_vel:%f ",pose_gps_.height ,pose_gps_.yaw,pose_gps_.velocity);
                        std::cout<<""<<std::endl;
                        break;
                    case 0x59:
                        tf::Quaternion quat;
                        orientation_.x = (float)sData[0]/32768.0;
                        orientation_.y = (float)sData[1]/32768.0;
                        orientation_.z = (float)sData[2]/32768.0;
                        orientation_.w = (float)sData[3]/32768.0;
                        
                        quat.setValue(orientation_.x,orientation_.y,orientation_.z,orientation_.w);
                        double roll, pitch, yaw;//定义存储r\p\y的容器
                        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
                        printf("roll:%f, pitch:%f, yaw:%f ",roll*180*M_1_PI ,pitch*180*M_1_PI,yaw*180*M_1_PI);
                        std::cout<<""<<std::endl;
                        break;
            }		
            chrCnt=0;		

    }



}
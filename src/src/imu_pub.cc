// 用于在ros中发布IMU数据的节点

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <chrono>
#include "MPU6050.h"
#include <bcm2835.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);

    bcm2835_init();
    bcm2835_i2c_set_baudrate(400000);
    MPU6050_SENSOR imu(0x68);
    imu.initialize(
        100,
        MPU6050_DLPF_BW_42,
        MPU6050_GYRO_FS_1000DPS,
        MPU6050_ACCEL_FS_4G,
        0x01);

    sensor_msgs::Imu data;
    data.header.frame_id = "imu_frame";

    ros::Rate loop_rate(100);
    // ros::Time next_frame_stamp = ros::Time::now();
    while (ros::ok)
    {
        data.header.stamp = ros::Time::now();
        imu.getAccelData(
            &(data.linear_acceleration.x),
            &(data.linear_acceleration.y),
            &(data.linear_acceleration.z));
        imu.getGyroDataRad(
            &(data.angular_velocity.x),
            &(data.angular_velocity.y),
            &(data.angular_velocity.z));
        // data.linear_acceleration_covariance[0] = -1.0;
        // data.angular_velocity_covariance[0] = -1.0;
        pub.publish(data);

        loop_rate.sleep();
        // next_frame_stamp = data.header.stamp + ros::Duration(0.00998);
        // while (ros::Time::now() < next_frame_stamp) {
        //     for (int i = 0; i < 10000; ++i) ;
        // }
    }
    return 0;
}

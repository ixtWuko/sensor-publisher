// 用于在ros中发布IMU数据的节点

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "MPU6050.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);

    MPU6050_SENSOR imu(0x68);
    imu.initialize(
        9, // sample rate = 100Hz
        MPU6050_DLPF_BW_20,
        MPU6050_GYRO_FS_500DPS,
        MPU6050_ACCEL_FS_2G,
        0x01
    );

    double xa, ya, za;
    double xg, yg, zg;
    const double PI_DIVIDE_180 = 0.017453292519943295;
    sensor_msgs::Imu data;

    ros::Rate loop_rate(100);
    while (ros::ok)
    {
        data.header.stamp = ros::Time::now();
        imu.getAccelData(&xa, &ya, &za);
        imu.getGyroData(&xg, &yg, &zg);
        data.header.frame_id = "imu_frame";
        data.linear_acceleration.x = xa;
        data.linear_acceleration.y = ya;
        data.linear_acceleration.z = za;
        // data.linear_acceleration_covariance[0] = -1.0;
        data.angular_velocity.x = xg * PI_DIVIDE_180;
        data.angular_velocity.y = yg * PI_DIVIDE_180;
        data.angular_velocity.z = zg * PI_DIVIDE_180;
        // data.angular_velocity_covariance[0] = -1.0;
        pub.publish(data);
        loop_rate.sleep();
    }
    return 0;
}

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
        MPU6050_GYRO_FS_1000DPS,
        MPU6050_ACCEL_FS_4G,
        0x01
    );

    sensor_msgs::Imu data;
    data.header.frame_id = "imu_frame";

    ros::Rate loop_rate(100);
    while (ros::ok)
    {
        data.header.stamp = ros::Time::now();
        imu.getAccelData(
	    &(data.linear_acceleration.x),
	    &(data.linear_acceleration.y),
	    &(data.linear_acceleration.z)
	);
        imu.getGyroData(
            &(data.angular_velocity.x),
	    &(data.angular_velocity.y),
	    &(data.angular_velocity.z)
	);
        // data.linear_acceleration_covariance[0] = -1.0;
        // data.angular_velocity_covariance[0] = -1.0;
        pub.publish(data);
        loop_rate.sleep();
    }
    return 0;
}

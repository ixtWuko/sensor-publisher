// 用于在ros中发布IMU数据的节点
// 与 imu_pub 的区别在与使用内参数对数据进行修正

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <eigen_conversions/eigen_msg.h>
#include <chrono>
#include "MPU6050.h"
#include <bcm2835.h>

int main(int argc, char **argv)
{
    Eigen::Matrix3d Ta, Ka, Tg, Kg;
    Ta << 1.0, 0.000215674, -0.00439744,
          0.0, 1.0, -0.00750154,
          0.0, 0.0, 1.0;
    Ka << 1.00194, 0.0, 0.0,
          0.0, 0.999356, 0.0,
          0.0, 0.0, 0.966283;
    Tg << 1.0, -0.00592396, 0.000219019,
          -0.0160198, 1.0, 0.0128289,
          0.0212083, 0.0123858, 1.0;
    Kg << 1.00086, 0, 0,
          0, 1.00838, 0,
          0, 0, 1.0094;

    Eigen::Vector3d Ba, Bg;
    Ba << 0.310941, -0.156181, -1.06386;
    Bg << -0.0244814, -0.0828403, 0.0569897;

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

    double ax, ay, az;
    double gx, gy, gz;
    Eigen::Vector3d acc, gyr;

    ros::Rate loop_rate(100);
    while (ros::ok)
    {
        data.header.stamp = ros::Time::now();
        imu.getAccelData(&ax, &ay, &az);
        imu.getGyroDataRad(&gx, &gy, &gz);
        acc << ax, ay, az;
        gyr << gx, gy, gz;
        acc = Ta * Ka * (acc - Ba);
        gyr = Tg * Kg * (gyr - Bg);
	//std::cout << acc << std::endl;
	//std::cout << gyr << std::endl;
        tf::vectorEigenToMsg(acc, data.linear_acceleration);
        tf::vectorEigenToMsg(gyr, data.angular_velocity);

        pub.publish(data);

        loop_rate.sleep();
    }
    return 0;
}

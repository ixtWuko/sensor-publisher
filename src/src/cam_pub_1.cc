// 用于在ros中发布相机数据的节点

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "cam_pub_1");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/cam/1/data_raw", 1);

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    //cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 2);
    //cap.set(cv::CAP_PROP_EXPOSURE, 6)

    ros::Duration(1).sleep();
    if (!cap.isOpened()) return 1;

    cv_bridge::CvImage cvi;
    cvi.encoding = "bgr8";
    cvi.header.frame_id = "cam_1_frame";

    ros::Rate loop_rate(20);
    while(nh.ok()) {
        cvi.header.stamp = ros::Time::now();
        cap.read(cvi.image);
        if (!cvi.image.empty())
        {
            pub.publish(cvi.toImageMsg());
        }
        loop_rate.sleep();
    }
    return 0;
}

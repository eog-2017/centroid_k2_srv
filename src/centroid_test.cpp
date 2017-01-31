#include <iostream>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/highgui/highgui.hpp>

#include <centroid_k2_srv/centroid.h>

/**
cv_bridge::CvImage img_bridge;

bool cld_cap = false;
bool img_cap = false;

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    if (!cld_cap) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::cout<< "Cloud: width = "<< msg->width << " height = " << msg->height << std::endl;

        pcl::fromROSMsg(*msg, *pt_cld);

        pcl::io::savePCDFileASCII ("test_pcd.pcd", *pt_cld);

        cld_cap = true;
    }

}

void image_callback(const sensor_msgs::Image::ConstPtr& msg) {

    if (!img_cap) {
        std::cout<< "Image: width = "<< msg->width << " height = " << msg->height << std::endl;

        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::imwrite("test_img.jpg", img);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
        img_cap = true;
    }

}**/


int main(int argc, char **argv) {

    ros::init(argc, argv, "test_client");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile("/home/isl-server/Workspace/ros/src/centroid_k2_srv/data/test_pcd.pcd", *pt_cld);

    //cv::Mat = cv::imread("/home/isl-server/Workspace/ros/src/centroid_k2_srv/data/test_image.jpg");

    /**pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (pt_cld);
    while (!viewer.wasStopped ())
    {
    }**/

    ROS_DEBUG("Loaded PointCloud");

    /**
     * @brief msg_cld rosmessage format container
     */
    sensor_msgs::PointCloud2::Ptr msg_cld(new sensor_msgs::PointCloud2);

    ROS_DEBUG("Before conversion...");
    pcl::toROSMsg(*pt_cld, *msg_cld);
    ROS_DEBUG("Converted PointCloud");

    /**
     * @brief arr bbox for the cloud bear
     */
    std_msgs::UInt16MultiArray arr;

    arr.data.push_back(351);
    arr.data.push_back(120);
    arr.data.push_back(457);
    arr.data.push_back(241);

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<centroid_k2_srv::centroid>("get_centroid");

    centroid_k2_srv::centroid srv;
    srv.request.pc2 = *msg_cld;
    srv.request.boxes = arr;

    ROS_DEBUG("Before calling client service");

    if(client.call(srv)) {
        ROS_INFO("Successfully called get_centroid");
    } else {
        ROS_ERROR("Failed to call service get_centroid");
        return -1;
    }

    std_msgs::Float32MultiArray oarr = srv.response.centroid;

    ROS_DEBUG("Output : c_x : %f, c_y : %f, c_z : %f, n_x : %f, n_y : %f, n_z : %f",
              oarr.data[0], oarr.data[1], oarr.data[2], oarr.data[3], oarr.data[4], oarr.data[5]);

    /**
    image_transport::ImageTransport it(nh);

    ros::Subscriber cld_sub, img_sub;

    cld_sub = nh.subscribe<sensor_msgs::PointCloud2>("/kinect2/qhd/points", 1, cloud_callback);
    img_sub = nh.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_color", 1, image_callback);
    */

    ros::spinOnce();

    return 0;
}

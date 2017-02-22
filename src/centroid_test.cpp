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

int counter = 0;
cv_bridge::CvImage img_bridge;

int main(int argc, char **argv) {

    ros::init(argc, argv, "test_client");
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile("/home/isl-server/Workspace/ros/src/centroid_k2_srv/data/test_pcd.pcd", *pt_cld);
    cv::Mat img = cv::imread("/home/isl-server/Workspace/ros/src/centroid_k2_srv/data/test_image.jpg");

    ROS_DEBUG("Loaded Data");

    /**
     * @brief msg_cld rosmessage format container
     */
    sensor_msgs::PointCloud2::Ptr msg_cld(new sensor_msgs::PointCloud2);

    ROS_DEBUG("Before conversion...");
    pcl::toROSMsg(*pt_cld, *msg_cld);

    sensor_msgs::Image img_msg;
    std_msgs::Header header;
    header.seq = counter++;
    header.stamp = ros::Time::now();

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    img_bridge.toImageMsg(img_msg);
    ROS_DEBUG("Converted Data");

    ros::ServiceClient client = nh.serviceClient<centroid_k2_srv::centroid>("get_centroid");

    centroid_k2_srv::centroid srv;
    srv.request.pc2 = *msg_cld;
    srv.request.img = img_msg;

    ROS_DEBUG("Before calling client service");

    if(client.call(srv)) {
        ROS_INFO("Successfully called get_centroid");
    } else {
        ROS_ERROR("Failed to call service get_centroid");
        return -1;
    }

    std_msgs::Float32MultiArray oarr = srv.response.centroid;

    ROS_DEBUG("Centroid : c_x : %f, c_y : %f, c_z : %f",
              oarr.data[0], oarr.data[1], oarr.data[2]);
    ROS_DEBUG("Normal : n_x : %f, n_y : %f, n_z : %f",
              oarr.data[3], oarr.data[4], oarr.data[5]);
    ROS_DEBUG("Axis : a_x : %f, a_y : %f, a_z : %f",
              oarr.data[6], oarr.data[7], oarr.data[8]);

    ros::spinOnce();

    return 0;
}

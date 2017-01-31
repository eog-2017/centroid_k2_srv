#include <stdint.h>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <centroid_k2_srv/centroid.h>

#define A_THRESH 400

typedef struct {
    uint16_t xmin;
    uint16_t xmax;
    uint16_t ymin;
    uint16_t ymax;

    uint16_t get_area(){
        return (xmax-xmin) * (ymax-ymin);
    }
    void grow_area() {
        //TODO:
        return;
    }
}bbox;

bool get_centroid(centroid_k2_srv::centroid::Request &req,
                  centroid_k2_srv::centroid::Response &resp) {

    ROS_DEBUG("In callback");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 msg = req.pc2;
    std_msgs::UInt16MultiArray arr = req.boxes;
    ROS_DEBUG("Loaded request");

    pcl::fromROSMsg(msg, *pt_cld);

    ROS_DEBUG("Converted message");

    /**
     * We are going to work with just one box now, change this part later to
     * give multiple centroids later
     */
    bbox roi;
    roi.xmin = arr.data[0];
    roi.ymin = arr.data[1];
    roi.xmax = arr.data[2];
    roi.ymax = arr.data[3];

    ROS_DEBUG("bbox xmin : %d, ymin : %d, xmax : %d, ymax : %d", roi.xmin, roi.ymin, roi.xmax, roi.ymax);

    uint16_t area = roi.get_area();

    ROS_DEBUG("bbox area : %d", area);

    if (area < A_THRESH) {
        roi.grow_area();
        ROS_DEBUG("bbox area : %d", roi.get_area());
        ROS_DEBUG("bbox xmin : %d, ymin : %d, xmax : %d, ymax : %d", roi.xmin, roi.ymin, roi.xmax, roi.ymax);
    }


    uint16_t xmid = uint16_t(round((roi.xmin + roi.xmax)/2));
    uint16_t ymid = uint16_t(round((roi.ymin + roi.ymax)/2));

    ROS_DEBUG("bbox xmid : %d, ymid : %d", xmid, ymid);

    uint16_t new_index;
    uint16_t count = 0;

    /**
     *  Now take only those points that are interesting
     */

    std::cout << pt_cld->height << "  " << pt_cld->width << "\n";

    for(int i = roi.xmin; i <= roi.xmax; i++) {
        for(int j = roi.ymin; j <= roi.ymax; j++) {
            pcl::PointXYZRGB curr_pt = pt_cld->at(i, j);

            if(!isnan(curr_pt.z))
            {
                roi_pt_cld->push_back(curr_pt);

                if (i == xmid && j==ymid) new_index = count;
                count++;
            }
        }
    }
    ROS_DEBUG("downsampled point cloud");

    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("CloudViewer"));

    viewer->addPointCloud(roi_pt_cld);
    pcl::PointXYZRGB test_pt = pt_cld->at(xmid, ymid);
    test_pt.data[2] = 0.f;
    test_pt.data[1] = 0.f;
    test_pt.data[0] = 0.f;
    ROS_DEBUG("roi cloud size : %d", int(roi_pt_cld->size()));
    viewer->addLine(roi_pt_cld->points[new_index], test_pt);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce(100);
    }

    // Get the index of the point here.
    pcl::PointXYZRGB centroid = pt_cld->at(xmid, ymid);
    ROS_DEBUG("centroid : x : %f, y : %f, z : %f", centroid.data[0], centroid.data[1], centroid.data[2]);

    std::vector<int> index;
    ROS_DEBUG("index : %d", new_index);
    //new_index = 2;
    index.push_back(new_index);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(roi_pt_cld);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod(tree);

    //5 cm radius search
    //ne.setRadiusSearch(0.03);
    ne.setKSearch(100);

    float curv;
    float x, y, z;


    pcl::PointCloud<pcl::Normal>::Ptr ptr_normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*ptr_normal_cloud);

//    for(int i=0;i<ptr_normal_cloud->size();i++)
//    {
//        std::cout << "NORMAL = ";
//        for(int j=0;j<3;j++)
//            std::cout << ((pcl::Normal)ptr_normal_cloud->at(i)).normal[j] << " ";
//        std::cout << "\n";

//    }

    /*std::cout << "NORMAL AT CENTROID  = ";
    for(int j=0;j<3;j++)
        std::cout << ((pcl::Normal)ptr_normal_cloud->at(new_index)).normal[j] << " ";
    std::cout << "\n";*/

    pcl::Normal norm = ((pcl::Normal)ptr_normal_cloud->at(new_index));

    //ne.computePointNormal(*roi_pt_cld, index, norm.nx, norm.ny, norm.nz, curv);

    ROS_DEBUG("normal : x : %f, y : %f, z : %f", norm.normal[0], norm.normal[1], norm.normal[2]);

    /**
     * Change this part later.
     */
    resp.centroid.data.push_back(centroid.data[0]);
    resp.centroid.data.push_back(centroid.data[1]);
    resp.centroid.data.push_back(centroid.data[2]);

    resp.centroid.data.push_back(norm.normal[0]);
    resp.centroid.data.push_back(norm.normal[1]);
    resp.centroid.data.push_back(norm.normal[2]);

    ROS_DEBUG("Before returning");

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "centroid_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("get_centroid", get_centroid);
    ROS_INFO("centroid service is ready...");

    ros::spin();
    return 0;
}

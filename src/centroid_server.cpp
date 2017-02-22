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
#include <pcl/common/pca.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <centroid_k2_srv/centroid.h>
#include <faster_rcnn_obj_det_srv/bbox_scores.h>

#define A_THRESH 400
#define CV_DEBUG 0
#define PC_DEBUG 1

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

int counter = 0;
int scale_factor = 2;
cv::Rect crop(255, 120, 375, 240);

cv_bridge::CvImage img_bridge;

ros::ServiceClient bbox_client;

bool get_centroid(centroid_k2_srv::centroid::Request &req,
                  centroid_k2_srv::centroid::Response &resp) {

    ROS_DEBUG("In callback");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi_pt_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 msg = req.pc2;
    sensor_msgs::Image img = req.img;
    ROS_DEBUG("Loaded request");

    pcl::fromROSMsg(msg, *pt_cld);
    cv_bridge::CvImagePtr img_ptr;
    try {
        img_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    cv::Mat tote = img_ptr->image;

    ROS_DEBUG("Converted messages");
    ROS_DEBUG("%d, %d", tote.rows, tote.cols);

    cv::Mat crop_tote = tote(crop);
    cv::Mat int_tote;
    cv::resize(crop_tote, int_tote, cv::Size(crop_tote.cols*scale_factor, crop_tote.rows*scale_factor), 0, 0, cv::INTER_CUBIC);

    sensor_msgs::Image interp_img;
    std_msgs::Header header;
    header.seq = counter++;
    header.stamp = ros::Time::now();

    /*
     * Python expects the image to be in BGR format so change the encoding accordingly
     */
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, int_tote);
    img_bridge.toImageMsg(interp_img);

    faster_rcnn_obj_det_srv::bbox_scores srv;
    srv.request.input_rgb_img = interp_img;

    if (bbox_client.call(srv))
        ROS_INFO("\"faster_rcnn_obj_det_service\" was called successfully...");
    else {
        ROS_ERROR("Trouble calling service, exiting...");
        return false;
    }

    std::vector<int> bbox = srv.response.obj_box_rect.data;
    std::vector<double> score = srv.response.score.data;
    ROS_ASSERT(bbox.size()/5 == score.size());

    /*
     * Choose the bbox with highest prob.
     */
    double max = .0f;
    int j = 0;
    for (int i = 0; i < score.size(); i++){
        if (max < score[i]) {
            max = score[i];
            j = i;
        }
    }
    ROS_INFO("Max. Probability %f", max);
    ROS_INFO("Total number of objects detected %ld", score.size());
    if(CV_DEBUG) {
        std::string classes[] = {"__background__", "barkely_bones", "bunny_book", "cherokee_tshirt",
                  "clorox_brush", "cloud_bear", "command_hooks",
                  "crayola_24_ct", "creativity_stems", "dasani_bottle",
                  "easter_sippy_cup", "elmers_school_glue", "expo_eraser",
                  "fitness_dumbell", "folgers_coffee", "glucose_up_bottle",
                  "jane_dvd", "jumbo_pencil_cup", "kleenex_towels",
                  "kygen_puppies", "laugh_joke_book", "pencils",
                  "platinum_bowl", "rawlings_baseball", "safety_plugs",
                  "scotch_tape", "staples_cards", "viva",
                  "white_lightbulb", "woods_cord"};
        std::vector<cv::Rect> rects;
        for (int i = 0; i < score.size(); i++) {
            cv::Rect rect;
            rect.x = round(bbox[5*i]/scale_factor) + crop.x;
            rect.y = round(bbox[5*i + 1]/scale_factor) + crop.y;
            rect.width = round(bbox[5*i + 2]/scale_factor) - round(bbox[5*i]/scale_factor) ;
            rect.height = round(bbox[5*i + 3]/scale_factor) - round(bbox[5*i + 1]/scale_factor);
            rects.push_back(rect);
            cv::rectangle(tote, rect, cv::Scalar(0, 255, 0), 2);
            cv::putText(tote, classes[bbox[5*i + 4]], cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("debug_win", tote);
        int key = cv::waitKey(0);
        cv::destroyWindow("debug_win");
    }

    /*
     * Scale down the rects so that we are seeing them in the original
     * image.
     */
    cv::Rect orig_rect;
    orig_rect.x = round(bbox[5*j]/scale_factor) + crop.x;
    orig_rect.y = round(bbox[5*j + 1]/scale_factor) + crop.y;
    orig_rect.width = round(bbox[5*j + 2]/scale_factor) - round(bbox[5*j]/scale_factor) ;
    orig_rect.height = round(bbox[5*j + 3]/scale_factor) - round(bbox[5*j + 1]/scale_factor);

    int xmid = round(orig_rect.width/2) + orig_rect.x;
    int ymid = round(orig_rect.height/2) + orig_rect.y;

    ROS_DEBUG("bbox xmid : %d, ymid : %d", xmid, ymid);

    uint16_t new_index;
    uint16_t count = 0;

    /**
     *  Now take only those points that are interesting
     */

    std::cout << pt_cld->height << "  " << pt_cld->width << "\n";

    for(int i = orig_rect.x; i <= orig_rect.x + orig_rect.width; i++) {
        for(int j = orig_rect.y; j <= orig_rect.y + orig_rect.height; j++) {
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

    /**
     * Compute normals using NE
     */
    pcl::PointXYZRGB centroid = roi_pt_cld->at(new_index);
    ROS_DEBUG("centroid : x : %f, y : %f, z : %f", centroid.data[0], centroid.data[1], centroid.data[2]);

    /* This is not even being used
    std::vector<int> index;
    ROS_DEBUG("index : %d", new_index);
    index.push_back(new_index);*/

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(roi_pt_cld);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(2.0);

    pcl::PointCloud<pcl::Normal>::Ptr ptr_normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*ptr_normal_cloud);

    pcl::Normal norm = ((pcl::Normal)ptr_normal_cloud->at(new_index));

    /**
     * Compute major axis using PCA
     */
    pcl::PCA<pcl::PointXYZRGB> pca;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr neighbours(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<int> k_index;
    std::vector<float> k_dist;

    tree->nearestKSearch(roi_pt_cld->at(new_index), 100, k_index, k_dist);
    for (std::vector<int>::iterator it = k_index.begin(); it != k_index.end(); it++) {
        neighbours->push_back(roi_pt_cld->at(*it));
    }

    pca.setInputCloud(neighbours);
    Eigen::Matrix3f E = pca.getEigenVectors();
    pcl::PointXYZRGB axis;
    axis.x = E(0,0);
    axis.y = E(1,0);
    axis.z = E(2,0);

    if(PC_DEBUG) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("CloudViewer"));
        viewer->addPointCloud(roi_pt_cld);
        pcl::PointXYZRGB pt1 = centroid;
        pcl::PointXYZRGB pt2, pt3;
        pt2.x = norm.normal[0] + pt1.x;
        pt2.y = norm.normal[1] + pt1.y;
        pt2.z = norm.normal[2] + pt1.z;

        pt3.x = axis.x + pt1.x;
        pt3.y = axis.y + pt1.y;
        pt3.z = axis.z + pt1.z;

        viewer->addArrow(pt1, pt2, 0, 0, 1.0, true, "norm");
        viewer->addArrow(pt1, pt3, 0, 1.0, 0, true, "axis");

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce(100);
        }
    }

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

    resp.centroid.data.push_back(axis.data[0]);
    resp.centroid.data.push_back(axis.data[1]);
    resp.centroid.data.push_back(axis.data[2]);

    ROS_DEBUG("Before returning");

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "centroid_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("get_centroid", get_centroid);
    bbox_client = nh.serviceClient<faster_rcnn_obj_det_srv::bbox_scores>("/faster_rcnn_obj_det_service");
    ROS_INFO("centroid service is ready...");

    ros::spin();
    return 0;
}

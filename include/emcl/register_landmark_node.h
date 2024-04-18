#ifndef REGISTER_LANDMARK_H_
#define REGISTER_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include "emcl/x_means.h"

namespace emcl {

class register_landmark
{
public:
    register_landmark();
    ~register_landmark();

    struct Pos
    {
        float x, y, z;
    };
    struct Landmark
    {
        std::string class_;
        Pos pos_;
        int clusterID_;
        bool enable_;
        YAML::Node option_;
    };
    std::vector<Landmark> lm_list_;

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cb_yolo(const yolov5_pytorch_ros::BoundingBoxes& msg);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void get_pos(std::string, float, float, Landmark&);
    void read_yaml();
    bool write_yaml();
    void visualize_landmark();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber yolo_sub_;
    ros::ServiceServer save_srv_;
    ros::Publisher sphere_pub_;
    ros::Publisher text_pub_;
    ros::Timer clustering_timer;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    sensor_msgs::PointCloud cloud_;
    // emcl::x_means xm;
    int w_img_ = 1280;
    std::vector<std::string> landmark_list_{};

    //----------parameters----------
    std::vector<std::string> landmark_list{"Door", "Elevator", "Vending machine"};
    std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ver3.yaml";
    // std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ex.yaml";
    //------------------------------
};

}
#endif
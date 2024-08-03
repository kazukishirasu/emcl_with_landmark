#ifndef REGISTER_LANDMARK_H_
#define REGISTER_LANDMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <complex>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include "emcl/struct.h"
// #include "emcl/x_means.h"
#include "emcl/gmm.h"

namespace emcl {

class register_landmark
{
public:
    register_landmark();
    ~register_landmark();
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cb_yolo(const yolov5_pytorch_ros::BoundingBoxes& msg);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void loop();
    void init_list();
    void get_pos(float, float, Landmark&);
    void clustering(const ros::TimerEvent&);
    void read_yaml();
    bool write_yaml(std::vector<std::vector<Landmark>>&);
    void visualize_landmark(std::vector<std::vector<Landmark>>&);
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
    // x_means xm;
    GMM gmm;
    int w_img_ = 1280;
    std::vector<std::vector<Landmark>> data_{}, result_{};
    std::vector<std::string> save_name_{};

    //----------parameters----------
    std::vector<std::string> landmark_name{"Door", "Elevator", "Vending machine"};
    std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ver10.yaml";
    //------------------------------
};

}
#endif
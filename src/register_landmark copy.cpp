#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <math.h>
#include <random>
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
    int w_img_ = 1280;
    struct landmark
    {
        std::string class_;
        float pos_[3];
        int clusterID_;
        bool enable_;
        YAML::Node option_;
    };
    std::vector<struct landmark> lm_list_;
    std::vector<struct landmark> result_list_;
    std::vector<std::string> landmark_list_{};

    //----------parameters----------
    std::vector<std::string> landmark_list{"Door", "Elevator", "Vending machine"};
    std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ver1_default.yaml";
    // std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ver3.yaml";
    // std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ex.yaml";
    //------------------------------
public:
    register_landmark();
    ~register_landmark();
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cb_yolo(const yolov5_pytorch_ros::BoundingBoxes& msg);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void get_pos(std::string, float, float, struct landmark&);
    void x_means(const ros::TimerEvent& e);
    void k_means();
    void calc_centroid(std::vector<std::array<float, 2>>&);
    void allocate_id(std::vector<std::array<float, 2>>&);
    int random(int);
    void read_yaml();
    void write_yaml();
    void visualize_landmark(std::vector<struct landmark>&);
};

register_landmark::register_landmark()
{
    ROS_INFO("Start register_landmark node");
    read_yaml();
    scan_sub_ = nh_.subscribe("/scan", 1, &register_landmark::cb_scan, this);
    yolo_sub_ = nh_.subscribe("/detected_objects_in_image", 1, &register_landmark::cb_yolo, this);
    save_srv_ = nh_.advertiseService("/save_landmark", &register_landmark::cb_save_srv, this);
    sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_sphere", 1);
    text_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_text", 1);
    // clustering_timer =nh_.createTimer(ros::Duration(5), &register_landmark::x_means, this);
    visualize_landmark(lm_list_);
}

register_landmark::~register_landmark()
{
}

void register_landmark::cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!listener_.waitForTransform(msg->header.frame_id, "map", msg->header.stamp + ros::Duration().fromNSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0)))
    {
        return;
    }
    projector_.transformLaserScanToPointCloud("map", *msg, cloud_, listener_);
}

void register_landmark::cb_yolo(const yolov5_pytorch_ros::BoundingBoxes& msg)
{
    struct landmark lm;
    for (const auto &b:msg.bounding_boxes)
    {
        auto itr = std::find(landmark_list.begin(), landmark_list.end(), b.Class);
        if (b.probability > 0.9 && itr != landmark_list.end() && !cloud_.points.empty())
        {
            get_pos(b.Class, b.xmax, b.xmin, lm);
            lm_list_.push_back(lm);
        }
    }
}

bool register_landmark::cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    write_yaml();
    ROS_INFO("Called \"/save_landmark\" service");
    return true;
}

void register_landmark::get_pos(std::string class_, float xmax_, float xmin_, struct landmark &lm)
{
    lm.class_ = class_;
    auto yaw_ = -((((xmax_ + xmin_) / 2) - (w_img_/2)) * M_PI) / (w_img_/2);
    if (yaw_ < 0) {yaw_ += 6.28;}
    int index = (yaw_ * cloud_.points.size()) / 6.28;
    lm.pos_[0] = cloud_.points[index].x;
    lm.pos_[1] = cloud_.points[index].y;
    lm.pos_[2] = 0.0;
    lm.clusterID_ = random(6);
}

void register_landmark::x_means(const ros::TimerEvent& e)
{
    k_means();
}

void register_landmark::k_means()
{
    result_list_.clear();
    struct landmark lm;
    std::vector<std::array<float, 2>> centroid_list;
    for (size_t i = 0; i < 10; i++)
    {
        calc_centroid(centroid_list);
        allocate_id(centroid_list);
        if (i < 9)
        {
            centroid_list.clear();
        }
    }
    for (const auto &c:centroid_list)
    {
        lm.pos_[0] = c[0];
        lm.pos_[1] = c[0];
        lm.pos_[2] = 0.0;
        result_list_.push_back(lm);
    }
    visualize_landmark(result_list_);
}

void register_landmark::calc_centroid(std::vector<std::array<float, 2>> &centroid_list)
{
    std::array<float, 2> centroid;
    for (size_t i = 0; i < 6; i++)
    {
        float sum_x = 0, sum_y = 0;
        int count = 0;
        for (auto &lm:lm_list_)
        {
            if (lm.clusterID_ == i)
            {
                sum_x += lm.pos_[0];
                sum_y += lm.pos_[1];
                count++;
            }
        }
        centroid[0] = sum_x / count;
        centroid[1] = sum_y / count;
        centroid_list.push_back(centroid);
    }
}

void register_landmark::allocate_id(std::vector<std::array<float, 2>> &centroid_list)
{
    for (auto &lm:lm_list_)
    {
        float dist, min_dist;
        bool first = true;
        for (size_t i = 0; i < centroid_list.size(); i++)
        {
            dist = std::hypot(lm.pos_[0] - centroid_list[i][0], lm.pos_[1] - centroid_list[i][1]);
            if (first)
            {
                min_dist = dist;
                first = false;
            }else if (dist < min_dist)
            {
                lm.clusterID_ = i;
                min_dist = dist;
            }
        }
    }
}

int register_landmark::random(int max)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, max - 1);
    return dist(gen);
}

void register_landmark::read_yaml()
{
    try
    {
        struct landmark lm;
        YAML::Node node = YAML::LoadFile(landmark_file_path);
        YAML::Node landmark = node["landmark"];
        for(YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            std::string lm_name = it->first.as<std::string>();
            lm.class_ = lm_name;
            YAML::Node config = landmark[lm_name];
            for(YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
            {
                std::string id = it->first.as<std::string>();
                lm.pos_[0] = it->second["pose"][0].as<float>();
                lm.pos_[1] = it->second["pose"][1].as<float>();
                lm.pos_[2] = it->second["pose"][2].as<float>();
                lm.enable_ = it->second["enable"].as<bool>();
                lm.option_ = it->second["option"].as<YAML::Node>();
                lm_list_.push_back(lm);
            }
            auto itr = std::find(landmark_list_.begin(), landmark_list_.end(), lm_name);
            if (itr == landmark_list_.end())
            {
                landmark_list_.push_back(lm_name);
            }
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

void register_landmark::write_yaml()
{
    try
    {
        std::ofstream fout(landmark_file_path);
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::BeginMap;
        for (const auto &name:landmark_list_)
        {
            out << YAML::Key << name;
            out << YAML::BeginMap;
            int count = 1;
            for (const auto &ll:lm_list_)
            {
                if (ll.class_ == name)
                {
                    std::string id = "id";
                    id += std::to_string(count);
                    out << YAML::Key << id;
                    out << YAML::BeginMap;
                    out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << ll.pos_[0] << ll.pos_[1] << ll.pos_[2] << YAML::EndSeq;
                    out << YAML::Key << "enable" << YAML::Value << true;
                    out << YAML::Key << "option" << YAML::Value << YAML::Null;
                    out << YAML::EndMap;
                    count++;
                }
            }
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;
        fout << out.c_str();
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

void register_landmark::visualize_landmark(std::vector<struct landmark> &lm_list)
{
    visualization_msgs::Marker sphere_, text_;
    geometry_msgs::Point point_;
    std_msgs::ColorRGBA color_;
    sphere_.header.frame_id = "map";
    sphere_.header.stamp = ros::Time::now();
    sphere_.ns = "sphere";
    sphere_.type = visualization_msgs::Marker::SPHERE;
    sphere_.action = visualization_msgs::Marker::ADD;
    sphere_.pose.orientation.x = 0.0;
    sphere_.pose.orientation.y = 0.0;
    sphere_.pose.orientation.z = 0.0;
    sphere_.pose.orientation.w = 1.0;
    sphere_.scale.x = 0.5;
    sphere_.scale.y = 0.5;
    sphere_.scale.z = 0.5;

    text_.header.frame_id = "map";
    text_.header.stamp = ros::Time::now();
    text_.ns = "text";
    text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_.action = visualization_msgs::Marker::ADD;
    text_.pose.orientation.x = 0.0;
    text_.pose.orientation.y = 0.0;
    text_.pose.orientation.z = 0.0;
    text_.pose.orientation.w = 1.0;
    text_.scale.x = 0.5;
    text_.scale.y = 0.5;
    text_.scale.z = 0.5;

    int i = 0;
    for (const auto &ll:lm_list)
    {
        text_.text = ll.class_.c_str();
        sphere_.pose.position.x = ll.pos_[0];
        sphere_.pose.position.y = ll.pos_[1];
        sphere_.pose.position.z = ll.pos_[2];
        text_.pose.position.x = ll.pos_[0];
        text_.pose.position.y = ll.pos_[1] + 0.4;
        text_.pose.position.z = ll.pos_[2];
        sphere_.color.r = 0.0;
        sphere_.color.g = 0.0;
        sphere_.color.b = 1.0;
        sphere_.color.a = 0.8;
        text_.color.r = 1.0;
        text_.color.g = 1.0;
        text_.color.b = 1.0;
        text_.color.a = 1.0;
        sphere_.id = i;
        text_.id = i;
        sphere_pub_.publish(sphere_);
        text_pub_.publish(text_);
        i++;
    }
    
    // visualization_msgs::Marker marker_;
    // geometry_msgs::Point point_;
    // std_msgs::ColorRGBA color_;
    // for (const auto &lm:lm_list)
    // {
    //     point_.x = lm.pos_[0];
    //     point_.y = lm.pos_[1];
    //     point_.z = lm.pos_[2];
    //     marker_.points.push_back(point_);
    //     if (lm.class_ == "Elevator"){
    //         color_.r = 0.0;
    //         color_.g = 0.0;
    //         color_.b = 1.0;
    //     }else if (lm.class_ == "Door"){
    //         color_.r = 0.90;
    //         color_.g = 0.71;
    //         color_.b = 0.13;
    //     }else{
    //         color_.r = 1.0;
    //         color_.g = 0.0;
    //         color_.b = 0.0;
    //     }
    //     color_.a = 1.0;
    //     marker_.colors.push_back(color_);
    // }
    // marker_.header.frame_id = "map";
    // marker_.header.stamp = ros::Time::now();
    // marker_.ns = "sphere";
    // marker_.id = 0;
    // marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    // marker_.action = visualization_msgs::Marker::ADD;
    // marker_.pose.orientation.x = 0.0;
    // marker_.pose.orientation.y = 0.0;
    // marker_.pose.orientation.z = 0.0;
    // marker_.pose.orientation.w = 1.0;
    // marker_.scale.x = 0.5;
    // marker_.scale.y = 0.5;
    // marker_.scale.z = 0.5;
    // marker_pub_.publish(marker_);
}
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Register_landmark");
    emcl::register_landmark rl;
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
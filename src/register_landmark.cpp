#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
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

class register_landmark
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber yolo_sub_;
    ros::ServiceServer save_srv_;
    ros::Publisher marker_pub_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    sensor_msgs::PointCloud cloud_;
    int w_img_ = 1280;
    struct landmark
    {
        std::string class_;
        float pos_[3];
        bool enable_;
        YAML::Node option_;
    };
    std::vector<landmark> lm_list_;

    std::vector<std::string> landmark_list{"Door", "Elevator", "Vending machine"};
    std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark.yaml";
    // std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_copy.yaml";
public:
    register_landmark();
    ~register_landmark();
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void cb_yolo(const yolov5_pytorch_ros::BoundingBoxes& msg);
    bool cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void loop();
    void read_yaml();
    void write_yaml();
    void visualize_landmark(std::vector<landmark>&);
};

register_landmark::register_landmark()
{
    ROS_INFO("Start register_landmark node");
    read_yaml();
    scan_sub_ = nh_.subscribe("/scan", 1, &register_landmark::cb_scan, this);
    yolo_sub_ = nh_.subscribe("/detected_objects_in_image", 1, &register_landmark::cb_yolo, this);
    save_srv_ = nh_.advertiseService("/save_landmark", &register_landmark::cb_save_srv, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
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
        auto it = std::find(landmark_list.begin(), landmark_list.end(), b.Class.c_str());
        if (it != landmark_list.end() && b.probability > 0.9)
        {
            lm.class_ = b.Class.c_str();
            auto yaw_ = -((((b.xmin + b.xmax) / 2) - (w_img_/2)) * M_PI) / (w_img_/2);
            if (yaw_ < 0)
            {
                yaw_ += 6.28;
            }
            int index = (yaw_ * cloud_.points.size()) / 6.28;
            lm.pos_[0] = cloud_.points[index].x;
            lm.pos_[1] = cloud_.points[index].y;
            lm.pos_[2] = 0.0;
            lm_list_.push_back(lm);
        }
    }
    visualize_landmark(lm_list_);
    // lm_list_.clear();
}

bool register_landmark::cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    write_yaml();
    ROS_INFO("called \"/save_landmark\" service");
    return true;
}

void register_landmark::loop()
{
    ROS_INFO("size: %ld", lm_list_.size());
}

void register_landmark::read_yaml()
{
    try
    {
        struct landmark lm;
        YAML::Node node = YAML::LoadFile(landmark_file_path);
        YAML::Node landmark = node["landmark"];
        for(YAML::const_iterator it=landmark.begin();it!=landmark.end();++it)
        {
            std::string lm_name = it->first.as<std::string>();
            lm.class_ = lm_name;
            YAML::Node config = landmark[lm_name];
            for(YAML::const_iterator it=config.begin();it!=config.end();++it)
            {
                std::string id = it->first.as<std::string>();
                lm.pos_[0] = it->second["pose"][0].as<float>();
                lm.pos_[1] = it->second["pose"][1].as<float>();
                lm.pos_[2] = it->second["pose"][2].as<float>();
                lm.enable_ = it->second["enable"].as<bool>();
                lm.option_ = it->second["option"].as<YAML::Node>();
                lm_list_.push_back(lm);
            }
            // auto itr = std::find(landmark_list.begin(), landmark_list.end(), lm_name);
            // if (itr == landmark_list.end())
            // {
            //     landmark_list.push_back(it->first.as<std::string>());
            // }
            // int lm_index = std::distance(landmark_list.begin(), itr);
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

void register_landmark::write_yaml()
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "landmark";
    out << YAML::BeginMap;
    for (const auto &name:landmark_list)
    {
        out << YAML::Key << name;
        out << YAML::BeginMap;
        int count = 1;
        for (const auto &lm:lm_list_)
        {
            if (lm.class_ == name)
            {
                std::string id = "id";
                id += std::to_string(count);
                out << YAML::Key << id;
                out << YAML::BeginMap;
                out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << lm.pos_[0] << lm.pos_[1] << lm.pos_[2] << YAML::EndSeq;
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
    std::ofstream fout(landmark_file_path);
    fout << out.c_str();
}

void register_landmark::visualize_landmark(std::vector<landmark> &lm_list)
{
    visualization_msgs::Marker marker_;
    geometry_msgs::Point point_;
    std_msgs::ColorRGBA color_;
    for (const auto &lm:lm_list)
    {
        point_.x = lm.pos_[0];
        point_.y = lm.pos_[1];
        point_.z = lm.pos_[2];
        marker_.points.push_back(point_);
        if (lm.class_ == "Elevator"){
            color_.r = 0.0;
            color_.g = 0.0;
            color_.b = 1.0;
        }else if (lm.class_ == "Door"){
            color_.r = 0.90;
            color_.g = 0.71;
            color_.b = 0.13;
        }else{
            color_.r = 1.0;
            color_.g = 0.0;
            color_.b = 0.0;
        }
        color_.a = 1.0;
        marker_.colors.push_back(color_);
    }
    marker_.header.frame_id = "map";
    marker_.header.stamp = ros::Time::now();
    marker_.ns = "sphere";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;
    marker_.scale.x = 0.5;
    marker_.scale.y = 0.5;
    marker_.scale.z = 0.5;
    marker_pub_.publish(marker_);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Register_landmark");
    register_landmark rl;
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        rl.loop();
        rate.sleep();
    }
    return 0;
}
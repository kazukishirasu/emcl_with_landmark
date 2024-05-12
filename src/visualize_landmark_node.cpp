#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/Marker.h>
#include "emcl/struct.h"

class visualize_landmark_node
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher sphere_pub_;
    ros::Publisher text_pub_;
    std::vector<std::string> landmark_name_{};
    std::vector<std::vector<Landmark>> lm_list_;

    //----------parameters----------
    std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ver3.yaml";
    // std::string landmark_file_path = ros::package::getPath("emcl") += "/landmark/landmark_ex.yaml";
    //------------------------------
public:
    visualize_landmark_node();
    ~visualize_landmark_node();
    void loop();
    void read_yaml();
    void visualize_landmark(std::vector<std::vector<Landmark>>&);
};

visualize_landmark_node::visualize_landmark_node()
{
    ROS_INFO("Start visualize_landmark_node");
    read_yaml();
    sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_sphere", 1);
    text_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_text", 1);
}

visualize_landmark_node::~visualize_landmark_node()
{
}

void visualize_landmark_node::loop()
{
    visualize_landmark(lm_list_);
}

void visualize_landmark_node::read_yaml()
{
    try
    {
        struct Landmark lm;
        YAML::Node node = YAML::LoadFile(landmark_file_path);
        YAML::Node landmark = node["landmark"];
        int index = 0;
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            std::string lm_name = it->first.as<std::string>();
            landmark_name_.push_back(lm_name);
            YAML::Node config = landmark[lm_name];
            bool first = true;
            for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
            {
                if (first)
                {
                    std::vector<Landmark> class_n;
                    lm_list_.push_back(class_n);
                    first = false;
                }
                std::string id = it->first.as<std::string>();
                lm.pos_.x = it->second["pose"][0].as<float>();
                lm.pos_.y = it->second["pose"][1].as<float>();
                lm.pos_.z = it->second["pose"][2].as<float>();
                lm.enable_ = it->second["enable"].as<bool>();
                lm.option_ = it->second["option"].as<YAML::Node>();
                lm_list_[index].push_back(lm);
            }
            index++;
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

void visualize_landmark_node::visualize_landmark(std::vector<std::vector<Landmark>>& lm_list)
{
    // visualization_msgs::Marker sphere_, text_;
    // geometry_msgs::Point point_;
    // std_msgs::ColorRGBA color_;
    // sphere_.header.frame_id = "map";
    // sphere_.header.stamp = ros::Time::now();
    // sphere_.ns = "sphere";
    // sphere_.type = visualization_msgs::Marker::SPHERE;
    // sphere_.action = visualization_msgs::Marker::ADD;
    // sphere_.pose.orientation.x = 0.0;
    // sphere_.pose.orientation.y = 0.0;
    // sphere_.pose.orientation.z = 0.0;
    // sphere_.pose.orientation.w = 1.0;
    // sphere_.scale.x = 0.5;
    // sphere_.scale.y = 0.5;
    // sphere_.scale.z = 0.5;

    // text_.header.frame_id = "map";
    // text_.header.stamp = ros::Time::now();
    // text_.ns = "text";
    // text_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // text_.action = visualization_msgs::Marker::ADD;
    // text_.pose.orientation.x = 0.0;
    // text_.pose.orientation.y = 0.0;
    // text_.pose.orientation.z = 0.0;
    // text_.pose.orientation.w = 1.0;
    // text_.scale.x = 0.5;
    // text_.scale.y = 0.5;
    // text_.scale.z = 0.5;

    // int index = 0;
    // for (const auto &ll:lm_list)
    // {
    //     text_.text = landmark_name_[index];
    //     int id = 0;
    //     for (const auto &l:ll)
    //     {
    //         sphere_.pose.position.x = l.pos_.x;
    //         sphere_.pose.position.y = l.pos_.y;
    //         sphere_.pose.position.z = l.pos_.z;
    //         text_.pose.position.x = l.pos_.x;
    //         text_.pose.position.y = l.pos_.y + 0.4;
    //         text_.pose.position.z = l.pos_.z;
    //         sphere_.color.r = 0.0;
    //         sphere_.color.g = 0.0;
    //         sphere_.color.b = 1.0;
    //         sphere_.color.a = 0.8;
    //         text_.color.r = 1.0;
    //         text_.color.g = 1.0;
    //         text_.color.b = 1.0;
    //         text_.color.a = 1.0;
    //         sphere_.id = id;
    //         text_.id = id;
    //         sphere_pub_.publish(sphere_);
    //         text_pub_.publish(text_);
    //         id++;
    //     }
    //     index++;
    // }

    visualization_msgs::Marker marker_;
    geometry_msgs::Point point_;
    std_msgs::ColorRGBA color_;
    int index = 0;
    for (const auto &ll:lm_list)
    {
        std::string class_ = landmark_name_[index];
        for (const auto &l:ll)
        {
            point_.x = l.pos_.x;
            point_.y = l.pos_.y;
            point_.z = l.pos_.z;
            marker_.points.push_back(point_);
            if (class_ == "Elevator"){
                color_.r = 0.0;
                color_.g = 0.0;
                color_.b = 1.0;
            }else if (class_ == "Door"){
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
        index++;
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
    sphere_pub_.publish(marker_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "register_landmark_node");
    visualize_landmark_node vl;
    while (ros::ok())
    {
        vl.loop();
    }
    return 0;
}
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <visualization_msgs/Marker.h>

class register_landmark{
    private:
        ros::NodeHandle nh;
        ros::Subscriber scan_sub;
        ros::Subscriber yolo_sub;
        ros::Publisher marker_pub;
        laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud cloud;
        tf::TransformListener listener;
        std::vector<std::pair<std::string, float>> object_yaw;
        int w_img = 1280;
        float scan_angle = 6.28;
        bool success_transform;
        std::string yaml_path = ros::package::getPath("emcl") += "/landmark/landmark.yaml";
        std::vector<std::string> landmark_list{"door", "Elevator", "Vending machine"};
        struct Pose{
            double pose[3];
        };
        struct Id{
            Pose pose;
            bool enable;
            std::string option;
        };
        std::vector<std::vector<Id>> landmark_yaml;

    public:
        register_landmark();
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void yolo_callback(const yolov5_pytorch_ros::BoundingBoxes& msg);
        void loop();
        void get_coordinate();
        void visualize_spot();
        bool read_yaml();
        void write_yaml(bool);
};

register_landmark::register_landmark(){
    ROS_INFO("Start register_landmark node");
    scan_sub = nh.subscribe("/scan", 1, &register_landmark::scan_callback, this);
    yolo_sub = nh.subscribe("/detected_objects_in_image", 1, &register_landmark::yolo_callback, this);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void register_landmark::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    try{
        projector.transformLaserScanToPointCloud("map", *msg, cloud, listener);
        success_transform = true;
    }
    catch (tf::TransformException& e){
        ROS_WARN("%s", e.what());
        success_transform = false;
        return;
    }
}

void register_landmark::yolo_callback(const yolov5_pytorch_ros::BoundingBoxes& msg){
    object_yaw.clear();
    for (size_t i = 0; i < msg.bounding_boxes.size(); i++){
        float yaw = -((((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2) - (w_img/2)) * M_PI) / (w_img/2);
        object_yaw.push_back(std::make_pair(msg.bounding_boxes[i].Class.c_str(), yaw));
    }
}

void register_landmark::loop(){
    if (success_transform){
        get_coordinate();
    }
    write_yaml(read_yaml());
}

void register_landmark::get_coordinate(){
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    visualization_msgs::Marker marker;
    for (size_t i = 0; i < object_yaw.size(); i++){
        if (object_yaw[i].second < 0){
            object_yaw[i].second = object_yaw[i].second + 6.28;
        }
        int index = (object_yaw[i].second * cloud.points.size()) / scan_angle;
        point.x = cloud.points[index].x;
        point.y = cloud.points[index].y;
        point.z = 0.0;
        marker.points.push_back(point);
        if (object_yaw[i].first == "Elevator"){
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 1.0;
            marker.colors.push_back(color);
        }else if (object_yaw[i].first == "Door"){
            color.r = 0.90;
            color.g = 0.71;
            color.b = 0.13;
            color.a = 1.0;
            marker.colors.push_back(color);
        }else{
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
            marker.colors.push_back(color);
        }
    }

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.1;

    marker_pub.publish(marker);
}

void register_landmark::visualize_spot(){
}

bool register_landmark::read_yaml(){
    try{
        YAML::Node config = YAML::LoadFile(yaml_path);
        for (YAML::const_iterator it = config["landmark"].begin(); it != config["landmark"].end(); ++it){
            std::vector<Id> ids;
            for (YAML::const_iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2){
                Id id;
                id.pose.pose[0] = it2->second["pose"][0].as<double>();
                id.pose.pose[1] = it2->second["pose"][1].as<double>();
                id.pose.pose[2] = it2->second["pose"][2].as<double>();
                id.enable = it2->second["enable"].as<bool>();
                if (it2->second["option"]){
                    id.option = it2->second["option"].IsNull() ? "null" : it2->second["option"].as<std::string>();
                }else{
                    id.option = "null";
                }
                ids.push_back(id);
            }
            landmark_yaml.push_back(ids);
        }
        for (size_t i = 0; i < landmark_yaml.size(); ++i) {
            for (size_t j = 0; j < landmark_yaml[i].size(); ++j) {
                std::cout << "pose: [" 
                << landmark_yaml[i][j].pose.pose[0] << ", " 
                << landmark_yaml[i][j].pose.pose[1] << ", " 
                << landmark_yaml[i][j].pose.pose[2] << "], " 
                << "enable: " << landmark_yaml[i][j].enable << ", " 
                << "option: " << landmark_yaml[i][j].option << std::endl;
            }
        }
        return true;
    }catch(const std::exception& e){
        // ROS_ERROR("%s", e.what());
        return false;
    }
}

void register_landmark::write_yaml(bool empty){
    if (!empty){
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::Value << YAML::BeginMap;
        for (const auto &ll : landmark_list){
            out << YAML::Key << ll;
            out << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "id1";
            out << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << 0 << 0 << 0 << YAML::EndSeq;
            out << YAML::Key << "enable" << YAML::Value << false;
            out << YAML::Key << "option" << YAML::Value << YAML::Null;
            out << YAML::EndMap;
            out << YAML::EndMap;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;
        std::ofstream fout(yaml_path);
        fout << out.c_str();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Register_landmark");
    register_landmark rl;
    ros::Rate rate(1);
    while (ros::ok()){
        ros::spinOnce();
        rl.loop();
        rate.sleep();
    }
    return 0;
}
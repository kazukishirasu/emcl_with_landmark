#include <ros/ros.h>
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
        ros::Subscriber scan_sub,
                        yolo_sub;
        ros::Publisher marker_pub;
        int w_img = 1280;
        int scan_angle = 6.28;
        bool success_transform;
        std::vector<std::pair<std::string, float>> object_yaw;
        sensor_msgs::PointCloud cloud;
        laser_geometry::LaserProjection projector;
        tf::TransformListener listener;
    public:
        register_landmark();
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void yolo_callback(const yolov5_pytorch_ros::BoundingBoxes& msg);
        void loop();
        void extraction_scan();
        void read_yaml();
        void write_yaml();
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
        // ROS_INFO("Transform LaserScan to PointCloud");
        success_transform = true;
    }
    catch (tf::TransformException& e){
        // ROS_WARN("%s", e.what());
        success_transform = false;
        return;
    }
}

void register_landmark::yolo_callback(const yolov5_pytorch_ros::BoundingBoxes& msg){
    object_yaw.clear();
    for (size_t i = 0; i < msg.bounding_boxes.size(); i++){
        float yaw = -((((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2) - (w_img/2)) * M_PI) / (w_img/2);
        // ROS_INFO("Class: %s, yaw: %f", msg.bounding_boxes[i].Class.c_str(), yaw);
        object_yaw.push_back(std::make_pair(msg.bounding_boxes[i].Class.c_str(), yaw));
    }
}

void register_landmark::loop(){
    if (success_transform){
        extraction_scan();
    }
}

void register_landmark::extraction_scan(){
    ROS_INFO("------------------------------");
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    visualization_msgs::Marker marker;
    for (size_t i = 0; i < object_yaw.size(); i++){
        if (object_yaw[i].second < 0){
            object_yaw[i].second = object_yaw[i].second + 6.28;
        }
        int index = (object_yaw[i].second * cloud.points.size()) / scan_angle;
        if (object_yaw[i].first == "Vending machine"){
            ROS_INFO("class: %s, x: %f, y: %f", object_yaw[i].first.c_str(), cloud.points[index].x, cloud.points[index].y);
        }
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

void register_landmark::read_yaml(){
}

void register_landmark::write_yaml(){
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Register_landmark");
    register_landmark rl;
    ros::Rate rate(10);
    while (ros::ok()){
        ros::spinOnce();
        rl.loop();
        rate.sleep();
    }
    return 0;
}
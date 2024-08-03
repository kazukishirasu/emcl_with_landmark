#include "emcl/register_landmark_node.h"

namespace emcl
{

register_landmark::register_landmark()
{
    ROS_INFO("Start register_landmark_node");
    init_list();
    read_yaml();
    scan_sub_ = nh_.subscribe("/scan", 30, &register_landmark::cb_scan, this);
    yolo_sub_ = nh_.subscribe("/detected_objects_in_image", 30, &register_landmark::cb_yolo, this);
    save_srv_ = nh_.advertiseService("/save_landmark", &register_landmark::cb_save_srv, this);
    sphere_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_sphere", 1);
    text_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_text", 1);
    // clustering_timer = nh_.createTimer(ros::Duration(10), &register_landmark::clustering, this);
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
    //時間同期あり
    if (std::abs(msg.header.stamp.toSec() - cloud_.header.stamp.toSec()) < 0.01)
    {
        struct Landmark lm;
        for (const auto &b:msg.bounding_boxes)
        {
            auto itr = std::find(landmark_name.begin(), landmark_name.end(), b.Class);
            if (b.probability > 0.8 && b.xmax - b.xmin > 30 && itr != landmark_name.end() && !cloud_.points.empty())
            // if (b.probability > 0.8 && itr != landmark_name.end() && !cloud_.points.empty())
            {
                const int index = std::distance(landmark_name.begin(), itr);
                get_pos(b.xmax, b.xmin, lm);
                data_[index].push_back(lm);
            }
        }
    }

    //時間同期なし
    // struct Landmark lm;
    // for (const auto &b:msg.bounding_boxes)
    // {
    //     auto itr = std::find(landmark_name.begin(), landmark_name.end(), b.Class);
    //     if (b.probability > 0.9 && itr != landmark_name.end() && !cloud_.points.empty())
    //     {
    //         const int index = std::distance(landmark_name.begin(), itr);
    //         get_pos(b.xmax, b.xmin, lm);
    //         data_[index].push_back(lm);
    //     }
    // }
}

bool register_landmark::cb_save_srv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (write_yaml(data_))
    {
        ROS_INFO("Landmark saved successfully");
        return true;
    }else{
        return false;
    }
    // if (write_yaml(result_))
    // {
    //     ROS_INFO("Landmark saved successfully");
    //     return true;
    // }else{
    //     return false;
    // }
}

void register_landmark::loop()
{
    // visualize_landmark(result_);
    visualize_landmark(data_);
}

void register_landmark::init_list()
{
    //ランドマークの数だけ要素を追加
    std::vector<Landmark> list;
    for (size_t i = 0; i < landmark_name.size(); i++)
    {
        data_.push_back(list);
        result_.push_back(list);
    }
}

void register_landmark::get_pos(float xmax, float xmin, struct Landmark& lm)
{
    auto yaw_ = -((((xmax + xmin) / 2) - (w_img_/2)) * M_PI) / (w_img_/2);
    if (yaw_ < 0) {yaw_ += 6.28;}
    int index = (yaw_ * cloud_.points.size()) / 6.28;
    lm.pos_.x = cloud_.points[index].x;
    lm.pos_.y = cloud_.points[index].y;
    lm.pos_.z = 0.0;
}

void register_landmark::clustering(const ros::TimerEvent& e)
{
    static bool empty = true;
    if (empty)
    {
        for (const auto &d:data_)
        {
            if (!d.empty())
            {
                empty = false;
            }
        }
    }
    if (!empty)
    {
        // xm.main(data_, result_);
        gmm.main(data_, result_);
    }
}

void register_landmark::read_yaml()
{
    try
    {
        for (const auto &ln:landmark_name){save_name_.push_back(ln);}
        struct Landmark lm;
        YAML::Node node = YAML::LoadFile(landmark_file_path);
        YAML::Node landmark = node["landmark"];
        for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
        {
            std::string lm_name = it->first.as<std::string>();
            auto itr = std::find(save_name_.begin(), save_name_.end(), lm_name);
            if (itr == save_name_.end())
            {
                std::vector<Landmark> list;
                save_name_.push_back(lm_name);
                data_.push_back(list);
            }
            YAML::Node config = landmark[lm_name];
            int index = std::distance(save_name_.begin(), itr);
            for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
            {
                std::string id = it->first.as<std::string>();
                lm.pos_.x = it->second["pose"][0].as<float>();
                lm.pos_.y = it->second["pose"][1].as<float>();
                lm.pos_.z = it->second["pose"][2].as<float>();
                lm.enable_ = it->second["enable"].as<bool>();
                lm.option_ = it->second["option"].as<YAML::Node>();
                data_[index].push_back(lm);
            }
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
    }
}

bool register_landmark::write_yaml(std::vector<std::vector<Landmark>>& lm_list)
{
    try
    {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "landmark";
        out << YAML::BeginMap;
        for (size_t index = 0; const auto &ll:lm_list)
        {
            std::string name = save_name_[index];
            out << YAML::Key << name;
            out << YAML::BeginMap;
            for (size_t id_n = 0; const auto &l:ll)
            {
                std::string id = "id";
                id += std::to_string(id_n);
                out << YAML::Key << id;
                out << YAML::BeginMap;
                out << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq << l.pos_.x << l.pos_.y << l.pos_.z << YAML::EndSeq;
                out << YAML::Key << "enable" << YAML::Value << true;
                out << YAML::Key << "option" << YAML::Value << YAML::Null;
                out << YAML::EndMap;
                id_n++;
            }
            out << YAML::EndMap;
            index++;
        }
        out << YAML::EndMap;
        out << YAML::EndMap;
        std::ofstream fout(landmark_file_path);
        fout << out.c_str();
        return true;
    }
    catch(const std::exception& e)
    {
        ROS_WARN("%s", e.what());
        return false;
    }
}

void register_landmark::visualize_landmark(std::vector<std::vector<Landmark>>& lm_list)
{
    //----------new----------
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

    for (size_t index = 0, id = 0; const auto &ll:lm_list)
    {
        if (index >= landmark_name.size())
        {
            return;
        }
        text_.text = landmark_name[index];
        for (const auto &l:ll)
        {
            sphere_.pose.position.x = l.pos_.x;
            sphere_.pose.position.y = l.pos_.y;
            sphere_.pose.position.z = l.pos_.z;
            text_.pose.position.x = l.pos_.x;
            text_.pose.position.y = l.pos_.y + 0.4;
            text_.pose.position.z = l.pos_.z;
            sphere_.color.r = 0.0;
            sphere_.color.g = 0.0;
            sphere_.color.b = 1.0;
            sphere_.color.a = 0.8;
            text_.color.r = 1.0;
            text_.color.g = 1.0;
            text_.color.b = 1.0;
            text_.color.a = 1.0;
            sphere_.id = id;
            text_.id = id;
            sphere_pub_.publish(sphere_);
            text_pub_.publish(text_);
            id++;
        }
        index++;
    }

    //----------old----------
    // visualization_msgs::Marker marker_;
    // geometry_msgs::Point point_;
    // std_msgs::ColorRGBA color_;
    // for (size_t index = 0; const auto &ll:lm_list)
    // {
    //     if (index >= landmark_name.size())
    //     {
    //         return;
    //     }
    //     std::string class_ = landmark_name[index];
    //     for (const auto &l:ll)
    //     {
    //         point_.x = l.pos_.x;
    //         point_.y = l.pos_.y;
    //         point_.z = l.pos_.z;
    //         marker_.points.push_back(point_);
    //         if (class_ == "Elevator"){
    //             color_.r = 0.0;
    //             color_.g = 0.0;
    //             color_.b = 1.0;
    //         }else if (class_ == "Door"){
    //             color_.r = 0.90;
    //             color_.g = 0.71;
    //             color_.b = 0.13;
    //         }else{
    //             color_.r = 1.0;
    //             color_.g = 0.0;
    //             color_.b = 0.0;
    //         }
    //         color_.a = 1.0;
    //         marker_.colors.push_back(color_);
    //     }
    //     index++;
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
    // sphere_pub_.publish(marker_);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "register_landmark_node");
    emcl::register_landmark rl;
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rl.loop();
        rate.sleep();
    }
    return 0;
}
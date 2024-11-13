//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/ExpResetMcl.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>

namespace emcl {

ExpResetMcl::ExpResetMcl(const Pose &p, int num, const Scan &scan,
				const std::shared_ptr<OdomModel> &odom_model,
				const std::shared_ptr<LikelihoodFieldMap> &map,
				double alpha_th, double open_space_th,
				double expansion_radius_position,
				double expansion_radius_orientation,
				const YAML::Node& landmark_config)
	: alpha_threshold_(alpha_th), open_space_threshold_(open_space_th),
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation), Mcl::Mcl(p, num, scan, odom_model, map)
{
	build_kd_tree(landmark_config);
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::build_kd_tree(const YAML::Node& landmark_config)
{
	YAML::Node landmark = landmark_config["landmark"];
	for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it){
		std::string name = it->first.as<std::string>();
		YAML::Node config = landmark[name];
		std::vector<kd_tree::Point> points;
		for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
			kd_tree::Point point;
			point.x = it->second["pose"][0].as<double>();
			point.y = it->second["pose"][1].as<double>();
			points.push_back(point);
		}
		std::shared_ptr<kd_tree::KdNode> node = kdt.build_kd_tree(points, 0);
		tree_list_.push_back(std::make_pair(name, node));
	}
	return;
}

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, double t, bool inv, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double ratio, const double phi_th, const double R_th, const int B)
{
	if(processed_seq_ == scan_.seq_)
		return;

	Scan scan;
	int seq = -1;
	while(seq != scan_.seq_){//trying to copy the latest scan before next 
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if (!inv) {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	} else {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	for(auto &p : particles_){
		p.w_ *= p.likelihood(map_.get(), scan);
	}

	alpha_ = normalizeBelief(particles_)/valid_beams;
	// alpha_ = nonPenetrationRate( particles_.size() / 20, map_.get(), scan); //new version

	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		vision_sensorReset(scan, bbox, landmark_config, w_img, R_th, B, t);
		expansionReset();
		std::vector<Particle> vision_particles = particles_;
		for (auto &p : particles_){
			p.w_ *= p.likelihood(map_.get(), scan);
		}
		normalizeBelief(particles_);
		for (auto &p : vision_particles){
			p.w_ *= p.vision_weight(bbox, landmark_config, phi_th, R_th, w_img);
		}
		normalizeBelief(vision_particles);
		for (size_t i = 0; i < particles_.size(); i++){
			particles_[i].w_ = (particles_[i].w_ * (1.0 - ratio)) + (vision_particles[i].w_ * ratio);
		}
	}
	// vision_sensorReset(scan, bbox, landmark_config, w_img, R_th, B, t);

	if(normalizeBelief(particles_) > 0.000001)
		resampling();
	else
		resetWeight();

	processed_seq_ = scan_.seq_;
}

void ExpResetMcl::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}

void ExpResetMcl::vision_sensorReset(const Scan& scan, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double R_th, const int B, double t)
{
	srand((unsigned)time(NULL));
	auto reset1 = [](std::vector<Particle>& particles,
					 const yolov5_pytorch_ros::BoundingBoxes& bbox,
					 const YAML::Node& landmark_config,
					 const double R_th, const int B){
		if (bbox.bounding_boxes.size() != 0){
        for(auto observed_landmark : bbox.bounding_boxes){
            for(YAML::const_iterator l_ = landmark_config["landmark"][observed_landmark.Class].begin(); l_!= landmark_config["landmark"][observed_landmark.Class].end(); ++l_){
                for (int i = 0; i <= B; i++){
                    Pose p_;
                    p_.x_ = l_->second["pose"][0].as<double>() + (double) rand() / RAND_MAX * R_th;
                    p_.y_ = l_->second["pose"][1].as<double>() + (double) rand() / RAND_MAX * R_th;
                    p_.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
                    Particle P(p_.x_, p_.y_, p_.t_, 0);
                    particles.push_back(P);
                    particles.erase(particles.begin());
                }
            }
        }
    }
	};
	auto reset2 = [](){
	};

	if (bbox.bounding_boxes.empty()){
		return;
	}else if (bbox.bounding_boxes.size() == 1){
		reset1(particles_, bbox, landmark_config, R_th, B);
	}else{
		reset2();
	}
}

}

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
	calc_inv_det(landmark_config);
	calc_distance(landmark_config);
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::calc_inv_det(const YAML::Node& landmark_config)
{
	YAML::Node landmark = landmark_config["landmark"];
	for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
	{
		Particle::Data data;
		std::string name = it->first.as<std::string>();
		data.name = name;
		YAML::Node config = landmark[name];
		for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it)
		{
			Eigen::Vector2d mean;
			mean(0) = it->second["pose"][0].as<double>();
			mean(1) = it->second["pose"][1].as<double>();

			Eigen::Matrix2d cov;
			cov(0, 0) = it->second["cov"][0][0].as<double>();
			cov(0, 1) = it->second["cov"][0][1].as<double>();
			cov(1, 0) = it->second["cov"][1][0].as<double>();
			cov(1, 1) = it->second["cov"][1][1].as<double>();

			Eigen::Matrix2d cov_inv = cov.inverse();
			double cov_det = cov.determinant();
			data.mean.push_back(mean);
			data.inv.push_back(cov_inv);
			data.det.push_back(cov_det);
		}
		data_.push_back(data);
	}
}

void ExpResetMcl::calc_distance(const YAML::Node& landmark_config){
	std::vector<std::pair<geometry_msgs::Point, std::string>> landmark_list;
	YAML::Node landmark = landmark_config["landmark"];
	for (YAML::const_iterator itr=landmark.begin(); itr!=landmark.end(); ++itr){
		std::string name = itr->first.as<std::string>();
		YAML::Node config = landmark[name];
		for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
			geometry_msgs::Point point;
			point.x = it->second["pose"][0].as<double>();
			point.y = it->second["pose"][1].as<double>();
			point.z = it->second["pose"][2].as<double>();
			std::string name_id = name + "-" + it->first.as<std::string>();
			landmark_list.push_back(std::make_pair(point, name_id));
		}
	}
	for (const auto& base:landmark_list){
		for (const auto& target:landmark_list){
			if (base.second != target.second){
				std::array<std::string, 2> name_pair = {base.second, target.second};
				std::sort(name_pair.begin(), name_pair.end());
				bool find = false;
				for (const auto& distance:landmark_distance_){
					if (distance.second == name_pair){
						find = true;
						break;
					}
				}
				if (!find){
					double dx = base.first.x - target.first.x;
					double dy = base.first.y - target.first.y;
					double dist = std::sqrt((dx * dx) + (dy * dy));
					landmark_distance_.push_back(std::make_pair(dist, name_pair));
				}
			}
		}
	}
}

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double ratio, const double R_th, const int B)
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
		vision_sensorReset(scan, bbox, landmark_config, w_img, R_th, B);
		expansionReset();
		std::vector<Particle> vision_particles = particles_;
		for (auto &p : particles_){
			p.w_ *= p.likelihood(map_.get(), scan);
		}
		normalizeBelief(particles_);
		for (auto &p : vision_particles){
			p.w_ *= p.vision_weight(map_.get(), scan, bbox, data_, w_img);
		}
		normalizeBelief(vision_particles);
		for (size_t i = 0; i < particles_.size(); i++){
			particles_[i].w_ = (particles_[i].w_ * (1.0 - ratio)) + (vision_particles[i].w_ * ratio);
		}
	}

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

void ExpResetMcl::vision_sensorReset(const Scan& scan, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double R_th, const int B)
{
	srand((unsigned)time(NULL));
	auto reset1 = [](std::vector<Particle>& particles,
					 const yolov5_pytorch_ros::BoundingBoxes& bbox,
					 const YAML::Node& landmark_config,
					 const double R_th, const int B){
		size_t index = 0;
		for (const auto& b:bbox.bounding_boxes){
			for (YAML::const_iterator it=landmark_config["landmark"][b.Class].begin(); it!=landmark_config["landmark"][b.Class].end(); ++it)
			{
				for (size_t i = 0; i < B; i++)
				{
					particles[index].p_.x_ = it->second["pose"][0].as<double>() + (double) rand() / RAND_MAX * R_th;
					particles[index].p_.y_ = it->second["pose"][1].as<double>() + (double) rand() / RAND_MAX * R_th;
					particles[index].p_.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
					index++;
				}
			}
		}
	};
	auto reset2 = [](std::vector<Particle>& particles,
					 const Scan& scan,
					 const yolov5_pytorch_ros::BoundingBoxes& bbox,
					 std::vector<DistWithName>& landmark_distance){
	};

	if (bbox.bounding_boxes.empty()){
		return;
	}else if (bbox.bounding_boxes.size() == 1){
		reset1(particles_, bbox, landmark_config, R_th, B);
	}else{
		reset1(particles_, bbox, landmark_config, R_th, B);
		reset2(particles_, scan, bbox, landmark_distance_);
	}
	
}

}

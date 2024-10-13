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
				const yolov5_pytorch_ros::BoundingBoxes& bbox,
				const YAML::Node& landmark_config,
				const int w_img)
	: alpha_threshold_(alpha_th), open_space_threshold_(open_space_th),
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation), Mcl::Mcl(p, num, scan, odom_model, map)
{
	calc_inv_det(landmark_config);
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

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img)
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

	alpha_ = normalizeBelief()/valid_beams;
	//alpha_ = nonPenetrationRate( particles_.size() / 20, map_.get(), scan); //new version

	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);

	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		expansionReset();
		for(auto &p : particles_){
			p.w_ *= p.likelihood(map_.get(), scan);
		}
	}

	// if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
	// 	ROS_INFO("RESET");
	// 	vision_sensorReset(bbox, landmark_config, w_img);
	// 	expansionReset();
	// 	for(auto &p : particles_){
	// 		p.w_ *= p.likelihood(map_.get(), scan);
	// 		if (!bbox.bounding_boxes.empty())
	// 		{
	// 			auto w_v = p.vision_weight(map_.get(), scan, bbox, data_, w_img);
	// 			p.w_ *= w_v;
	// 		}
	// 	}
	// }

	std::cout << "----------" << std::endl;
	for (auto& p:particles_)
	{
		if (!bbox.bounding_boxes.empty()){
			p.vision_weight(map_.get(), scan, bbox, data_, w_img);
		}
	}

	if(normalizeBelief() > 0.000001)
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

void ExpResetMcl::vision_sensorReset(const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img)
{
	int B = 1;
	double R_th = 10.0;
	srand((unsigned)time(NULL));
    if (bbox.bounding_boxes.size() != 0) {
        for(auto observed_landmark : bbox.bounding_boxes){
            for(YAML::const_iterator l_ = landmark_config["landmark"][observed_landmark.Class].begin(); l_!= landmark_config["landmark"][observed_landmark.Class].end(); ++l_){
                for (int i = 0; i <= B; i++) {
                    Pose p_;
                    p_.x_ = l_->second["pose"][0].as<double>() + (double) rand() / RAND_MAX * R_th;
                    p_.y_ = l_->second["pose"][1].as<double>() + (double) rand() / RAND_MAX * R_th;
                    p_.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
                    Particle P(p_.x_, p_.y_, p_.t_, 0);
					particles_.push_back(P);
					particles_.erase(particles_.begin());
                }
            }
        }
    }
}

}

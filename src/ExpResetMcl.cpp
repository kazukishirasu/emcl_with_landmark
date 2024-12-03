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
	calc_inv_det(landmark_config);
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::build_kd_tree(const YAML::Node& landmark_config)
{
	ICP_Matching::Tree tree;
	YAML::Node landmark = landmark_config["landmark"];
	for (YAML::const_iterator itr=landmark.begin(); itr!=landmark.end(); ++itr){
		std::string name = itr->first.as<std::string>();
		tree.name = name;
		YAML::Node config = landmark[name];
		std::vector<KD_Tree::Point> points;
		for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
			KD_Tree::Point point;
			point.x = it->second["pose"][0].as<double>();
			point.y = it->second["pose"][1].as<double>();
			points.push_back(point);
		}
		tree.node = kdt.build_kd_tree(points, 0);
		tree_list_.push_back(tree);
	}
	return;
}

void ExpResetMcl::calc_inv_det(const YAML::Node& landmark_config)
{
	YAML::Node landmark = landmark_config["landmark"];
	for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it)
	{
		Particle::Inv_Det data;
		std::string Class = it->first.as<std::string>();
		data.Class = Class;
		YAML::Node config = landmark[Class];
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
		inv_det_.push_back(data);
	}
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
		// p.w_ *= p.likelihood(map_.get(), scan, valid_beams);
		// p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, bbox, landmark_config, phi_th, R_th, w_img, ratio);
		p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, bbox, inv_det_, w_img, ratio);
	}

	alpha_ = normalizeBelief(particles_);

	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		vision_sensorReset(scan, bbox, landmark_config, w_img, R_th, B, t, lidar_t);
		expansionReset();
		for (auto &p : particles_){
			// p.w_ *= p.likelihood(map_.get(), scan, valid_beams);
			// p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, bbox, landmark_config, phi_th, R_th, w_img, ratio);
			p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, bbox, inv_det_, w_img, ratio);
		}
	}

	if(normalizeBelief(particles_) > 0.000001)
		resampling();
	else
		resetWeight(particles_);

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

void ExpResetMcl::vision_sensorReset(const Scan& scan, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double R_th, const int B, const double robot_t, const double lidar_t)
{
	srand((unsigned)time(NULL));
	auto reset1 = [&bbox, &landmark_config, &R_th, &B](std::vector<Particle>& particles){
		std::vector<std::string> Class_list;
		int number = 0;
		for (const auto& b : bbox.bounding_boxes){
			auto itr = std::find(Class_list.begin(), Class_list.end(), b.Class);
			if (itr == Class_list.end()){
				Class_list.push_back(b.Class);
				number += landmark_config["landmark"][b.Class].size();
			}
		}
		for(const auto& c : Class_list){
            for(YAML::const_iterator l = landmark_config["landmark"][c].begin(); l!= landmark_config["landmark"][c].end(); ++l){
                for (int i = 0; i <= (particles.size() * 0.8) / number; i++){
                    Pose p;
                    p.x_ = l->second["pose"][0].as<double>() + ((double) rand() / RAND_MAX * R_th);
                    p.y_ = l->second["pose"][1].as<double>() + ((double) rand() / RAND_MAX * R_th);
                    p.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
                    Particle P(p.x_, p.y_, p.t_, 0);
                    particles.push_back(P);
                    particles.erase(particles.begin());
                }
            }
    	}
	};
	auto reset2 = [&scan, &bbox, &landmark_config, &w_img, &R_th, &B, &robot_t, &lidar_t](std::vector<Particle>& particles, KD_Tree kdt, std::vector<ICP_Matching::Tree> tree_list){
		// ロボットと観測したランドマークとの相対座標を計算
		std::vector<ICP_Matching::Landmark> observed_list;
		for(const auto& b : bbox.bounding_boxes){
			KD_Tree::Point point;
			float yaw = -((((b.xmin + b.xmax) / 2) - (w_img / 2)) * M_PI) / (w_img / 2);
			if (yaw < 0)
				yaw += (M_PI * 2);
			yaw -= scan.angle_min_;
			if (yaw > M_PI * 2)
				yaw -= (M_PI * 2);
			yaw -= lidar_t;
			if (yaw > M_PI * 2){
				yaw -= M_PI * 2;
			}else if (yaw < 0){
				yaw += M_PI * 2;
			}
			int i = (yaw * scan.ranges_.size()) / (M_PI * 2);
			double a = (scan.angle_increment_ * i) + std::abs(scan.angle_min_) + scan.lidar_pose_yaw_;
			point.x = scan.ranges_[i] * std::cos(a);
			point.y = scan.ranges_[i] * std::sin(a);
			bool find = false;
			for (auto& observed : observed_list){
				if (b.Class == observed.name){
					observed.points.push_back(point);
					find = true;
				}
			}
			if (!find){
				ICP_Matching::Landmark landmark;
				landmark.name = b.Class;
				landmark.points.push_back(point);
				observed_list.push_back(landmark);
			}
    	}
		// ランドマーク周辺のランダムな位置を初期位置として設定
		std::vector<ICP_Matching::Data> data_list;
		YAML::Node landmark = landmark_config["landmark"];
		for (YAML::const_iterator itr=landmark.begin(); itr!=landmark.end(); ++itr){
			std::string name = itr->first.as<std::string>();
			YAML::Node config = landmark[name];
			for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
				for (size_t i = 0; i < B; i++){
					ICP_Matching::Data data;
					data.robot_pose.x = it->second["pose"][0].as<double>() + ((double) rand() / RAND_MAX * R_th);
					data.robot_pose.y = it->second["pose"][1].as<double>() + ((double) rand() / RAND_MAX * R_th);
					data.robot_pose.t = 2 * M_PI * rand() / RAND_MAX - M_PI;
					Eigen::Matrix2d rotation_matrix;
					rotation_matrix(0, 0) = std::cos(data.robot_pose.t);
					rotation_matrix(0, 1) = -std::sin(data.robot_pose.t);
					rotation_matrix(1, 0) = std::sin(data.robot_pose.t);
					rotation_matrix(1, 1) = std::cos(data.robot_pose.t);
					for (const auto& observed : observed_list){
						ICP_Matching::Landmark landmark;
						landmark.name = observed.name;
						for (const auto& observed_point : observed.points){
							Eigen::Vector2d point;
							point(0) = observed_point.x;
							point(1) = observed_point.y;
							point = rotation_matrix * point;
							KD_Tree::Point transformed_point;
							transformed_point.x = point(0) + data.robot_pose.x;
							transformed_point.y = point(1) + data.robot_pose.y;
							landmark.points.push_back(transformed_point);
						}
						data.landmarks.push_back(landmark);
					}
					data_list.push_back(data);
				}
			}
		}
		ICP_Matching icp;
		int ratio = particles.size() / data_list.size();
		for (auto& data : data_list){
			bool convergence = icp.matching(tree_list, data, bbox.bounding_boxes.size());
			if (convergence){
				for (size_t i = 0; i < ratio; i++){
					Pose p;
					p.x_ = data.robot_pose.x;
					p.y_ = data.robot_pose.y;
					p.t_ = data.robot_pose.t;
					Particle P(p.x_, p.y_, p.t_, 0);
					particles.push_back(P);
					particles.erase(particles.begin());
				}
			}else{
				Pose p;
				p.x_ = data.robot_pose.x;
				p.y_ = data.robot_pose.y;
				p.t_ = data.robot_pose.t;
				Particle P(p.x_, p.y_, p.t_, 0);
				particles.push_back(P);
				particles.erase(particles.begin());
			}
		}
	};

	if (bbox.bounding_boxes.empty()){
		return;
	}else if (bbox.bounding_boxes.size() == 1){
		reset1(particles_);
	}else{
		reset2(particles_, kdt, tree_list_);
	}
}

}

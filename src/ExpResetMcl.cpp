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

void ExpResetMcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, double t, bool inv, const bool use_vision, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double vision_ratio, const double sr_vision_ratio, const double phi_th, const double radius_th, const double particle_ratio)
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

	for(auto &p : particles_)
		// p.w_ *= p.likelihood(map_.get(), scan, valid_beams);
		p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, use_vision, bbox, landmark_config, phi_th, radius_th, w_img, vision_ratio);
		// p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, use_vision, bbox, inv_det_, w_img, vision_ratio);

	alpha_ = normalizeBelief(particles_);

	ROS_INFO("ALPHA: %f / %f", alpha_, alpha_threshold_);
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		if (use_vision){
			vision_sensorReset(scan, bbox, landmark_config, w_img, radius_th, particle_ratio, lidar_t);
		}
		expansionReset();
		for (auto &p : particles_)
			// p.w_ *= p.likelihood(map_.get(), scan, valid_beams);
			p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, use_vision, bbox, landmark_config, phi_th, radius_th, w_img, sr_vision_ratio);
			// p.w_ *= p.vision_weight(map_.get(), scan, valid_beams, use_vision, bbox, inv_det_, w_img, sr_vision_ratio);
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

void ExpResetMcl::vision_sensorReset(const Scan& scan, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, const int w_img, const double radius_th, double particle_ratio, const double lidar_t)
{
	// センサリセット1のラムダ式
	auto reset1 = [&bbox, &landmark_config, &radius_th, &particle_ratio](std::vector<Particle>& particles, LikelihoodFieldMap *map){
		srand((unsigned)time(NULL));
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
                for (int i = 0; i <= (particles.size() * particle_ratio) / number; i++){
                    Pose p;
                    p.x_ = l->second["pose"][0].as<double>() + ((double) rand() / RAND_MAX * radius_th);
                    p.y_ = l->second["pose"][1].as<double>() + ((double) rand() / RAND_MAX * radius_th);
                    p.t_ = 2 * M_PI * rand() / RAND_MAX - M_PI;
					if (map->inMapJudgment(p.x_, p.y_)){
						Particle P(p.x_, p.y_, p.t_, 0);
						particles.push_back(P);
						particles.erase(particles.begin());
					}
                }
            }
    	}
	};

	// センサリセット2のラムダ式
	auto reset2 = [&scan, &bbox, &landmark_config, &w_img, &radius_th, &particle_ratio, &lidar_t](std::vector<Particle>& particles, LikelihoodFieldMap *map, KD_Tree kdt, std::vector<ICP_Matching::Tree> tree_list){
		srand((unsigned)time(NULL));
		// ヨー角計算のラムダ式
		auto get_yaw = [&scan, &lidar_t](double& yaw){
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
		};
		// ロボットと観測したランドマークとの相対座標を計算
		std::vector<ICP_Matching::Landmark> observed_list;
		for(const auto& b : bbox.bounding_boxes){
			KD_Tree::Point min_point, max_point;
			double min_yaw = -((b.xmin - (w_img / 2)) * M_PI) / (w_img / 2);
			double max_yaw = -((b.xmax - (w_img / 2)) * M_PI) / (w_img / 2);
			get_yaw(min_yaw);
			get_yaw(max_yaw);
			int min_i = (min_yaw * scan.ranges_.size()) / (M_PI * 2);
			int max_i = (max_yaw * scan.ranges_.size()) / (M_PI * 2);
			double min_a = (scan.angle_increment_ * min_i) - std::abs(scan.angle_min_) - scan.lidar_pose_yaw_;
			double max_a = (scan.angle_increment_ * max_i) - std::abs(scan.angle_min_) - scan.lidar_pose_yaw_;
			min_point.x = scan.ranges_[min_i] * std::cos(min_a);
			min_point.y = scan.ranges_[min_i] * std::sin(min_a);
			max_point.x = scan.ranges_[max_i] * std::cos(max_a);
			max_point.y = scan.ranges_[max_i] * std::sin(max_a);
			KD_Tree::Point point;
			point.x = (min_point.x + max_point.x) / 2;
			point.y = (min_point.y + max_point.y) / 2;
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
		std::vector<std::string> Class_list;
		int number = 0;
		for (const auto& b : bbox.bounding_boxes){
			auto itr = std::find(Class_list.begin(), Class_list.end(), b.Class);
			if (itr == Class_list.end()){
				Class_list.push_back(b.Class);
				number += landmark_config["landmark"][b.Class].size();
			}
		}
		std::vector<ICP_Matching::Data> data_list;
		for(const auto& c : Class_list){
            for(YAML::const_iterator it = landmark_config["landmark"][c].begin(); it!= landmark_config["landmark"][c].end(); ++it){
                for (int i = 0; i < (particles.size() * particle_ratio) / number; i++){
					ICP_Matching::Data data;
					data.robot_pose.x = it->second["pose"][0].as<double>() + ((double) rand() / RAND_MAX * radius_th);
					data.robot_pose.y = it->second["pose"][1].as<double>() + ((double) rand() / RAND_MAX * radius_th);
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
		for (auto& data : data_list){
			bool convergence = icp.matching(tree_list, data, bbox.bounding_boxes.size());
			if (convergence && map->inMapJudgment(data.robot_pose.x, data.robot_pose.y)){
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
		reset1(particles_, map_.get());
	}else{
		reset2(particles_, map_.get(), kdt, tree_list_);
	}
}

}

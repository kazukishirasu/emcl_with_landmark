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
	// calc_inv_det(landmark_config);
	calc_distance(landmark_config);
}

ExpResetMcl::~ExpResetMcl()
{
}

void ExpResetMcl::calc_inv_det(const YAML::Node& landmark_config)
{
	YAML::Node landmark = landmark_config["landmark"];
	for (YAML::const_iterator it=landmark.begin(); it!=landmark.end(); ++it){
		Particle::InvDet invdet;
		std::string name = it->first.as<std::string>();
		invdet.name = name;
		YAML::Node config = landmark[name];
		for (YAML::const_iterator it=config.begin(); it!=config.end(); ++it){
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
			invdet.mean.push_back(mean);
			invdet.inv.push_back(cov_inv);
			invdet.det.push_back(cov_det);
		}
		invdet_list_.push_back(invdet);
	}
}

void ExpResetMcl::calc_distance(const YAML::Node& landmark_config)
{
	YAML::Node base = landmark_config["landmark"];
	for (YAML::const_iterator b_itr=base.begin(); b_itr!=base.end(); ++b_itr){
		std::string base_name = b_itr->first.as<std::string>();
		unsigned int base_id = 0;
		YAML::Node base_config = base[base_name];
		for (YAML::const_iterator bc_itr=base_config.begin(); bc_itr!=base_config.end(); ++bc_itr){
			float bx = bc_itr->second["pose"][0].as<float>();
			float by = bc_itr->second["pose"][1].as<float>();
			LandmarkInfo base_info;
			base_info.name = base_name;
			base_info.id = base_id;
			base_info.point.x = bx;
			base_info.point.y = by;
			base_info.dist = 0.0;
			DistanceList dist_list;
			dist_list.base = base_info;
			YAML::Node target = landmark_config["landmark"];
			for (YAML::const_iterator t_itr=target.begin(); t_itr!=target.end(); ++t_itr){
				std::string target_name = t_itr->first.as<std::string>();
				unsigned int target_id = 0;
				YAML::Node target_config = target[target_name];
				for (YAML::const_iterator tc_itr=target_config.begin(); tc_itr!=target_config.end(); ++tc_itr){
					if (target_name != base_name || target_id != base_id){
						float tx = tc_itr->second["pose"][0].as<float>();
						float ty = tc_itr->second["pose"][1].as<float>();
						float dx = bx - tx;
						float dy = by - ty;
						float dist = std::sqrt((dx * dx) + (dy * dy));
						LandmarkInfo target_info;
						target_info.name = target_name;
						target_info.id = target_id;
						target_info.point.x = tx;
						target_info.point.y = ty;
						target_info.dist = dist;
						dist_list.target.push_back(target_info);
					}
					target_id++;
				}
			}
			map_list_.push_back(dist_list);
			base_id++;
		}
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
			// p.w_ *= p.vision_weight(scan, bbox, invdet_list_, w_img);
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
	auto reset2 = [](std::vector<Particle>& particles,
					 const Scan& scan,
					 const yolov5_pytorch_ros::BoundingBoxes& bbox,
					 const int w_img,
					 std::vector<DistanceList>& map_list,
					 const YAML::Node& landmark_config,
					 double t){
		// ロボットから観測したランドマークまでの距離を計算
		std::vector<LandmarkInfo> observed_landmark;
		unsigned int id = 0;
		for (const auto& b:bbox.bounding_boxes){
			LandmarkInfo observed_info;
			observed_info.name = b.Class;
			observed_info.id = id;
			float yaw = -((((b.xmin + b.xmax) / 2) - (w_img / 2)) * M_PI) / (w_img / 2);
			if (yaw < 0)
				yaw += (M_PI * 2);
			observed_info.yaw = yaw;
			yaw -= scan.angle_min_;
			if (yaw > M_PI * 2)
				yaw -= (M_PI * 2);
			int i = (yaw * scan.ranges_.size()) / (M_PI * 2);
			observed_info.point.x = scan.ranges_[i] * std::cos((scan.angle_increment_ * i) + std::abs(scan.angle_min_));
			observed_info.point.y = scan.ranges_[i] * std::sin((scan.angle_increment_ * i) + std::abs(scan.angle_min_));
			observed_info.dist = scan.ranges_[i];
			observed_landmark.push_back(observed_info);
			id++;
		}
		// 各ランドマーク間の距離を計算
		std::vector<DistanceList> observed_list;
		for (auto& base:observed_landmark){
			DistanceList dist_list;
			dist_list.base = base;
			for (auto target:observed_landmark){
				if (target.name != base.name || target.id != base.id){
					float dx = base.point.x - target.point.x;
					float dy = base.point.y - target.point.y;
					target.dist = std::sqrt((dx * dx) + (dy * dy));
					dist_list.target.push_back(target);
				}
			}
			observed_list.push_back(dist_list);
		}
		// 各ランドマーク間の距離情報からどのランドマークを観測しているか予測
		std::vector<DistanceList> prediction_list;
		for (const auto& observed:observed_list){
			DistanceList dist_list;
			dist_list.base = observed.base;
			unsigned int sum = 0;
			for (auto& map:map_list){
				unsigned int count = 0;
				if (observed.base.name == map.base.name){
					for (const auto& o_target:observed.target){
						for (const auto& m_target:map.target){
							if (o_target.name == m_target.name){
								float diff = std::abs(o_target.dist - m_target.dist);
								if (diff < 0.5){
									count++;
									break;
								}
							}
						}
					}
				}
				// if (count >= (observed_landmark.size() - 1) * 0.6){
				if (count >= 1){
					map.base.probability = count;
					dist_list.target.push_back(map.base);
					sum += count;
				}
			}
			// 観測したランドマークがマップ上のどのランドマークか確率を計算
			for (auto& dl:dist_list.target){
				dl.probability /= (float)sum;
			}
			prediction_list.push_back(dist_list);
		}
		// for (const auto& a:prediction_list){
		// 	std::cout << "observed_landmark " << a.base.name << a.base.id << std::endl;
		// 	for (const auto& b:a.target){
		// 		std::cout << " " << b.name << b.id << " probability:" << b.probability << std::endl;
		// 	}
		// }
		// 予測結果からパーティクルを配置
		int raito = (particles.size() * 0.6) / observed_landmark.size();
		for (const auto& predicted:prediction_list){
			float x = predicted.base.dist * std::cos(predicted.base.yaw - M_PI + t);
			float y = predicted.base.dist * std::sin(predicted.base.yaw - M_PI + t);
			for (const auto& landmark:predicted.target){
				for (size_t i = 0; i < raito * landmark.probability; i++){
					Pose p_;
					p_.x_ = landmark.point.x + x;
					p_.y_ = landmark.point.y + y;
					p_.t_ = t;
					Particle P(p_.x_, p_.y_, p_.t_, 0);
					particles.push_back(P);
                    particles.erase(particles.begin());
				}
			}
		}
	};

	if (bbox.bounding_boxes.empty()){
		return;
	}else if (bbox.bounding_boxes.size() == 1){
		reset1(particles_, bbox, landmark_config, R_th, B);
	}else{
		reset2(particles_, scan, bbox, w_img, map_list_, landmark_config, t);
	}
}

}

//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef PARTICLE_H__
#define PARTICLE_H__

#include "emcl/Pose.h"
#include "emcl/LikelihoodFieldMap.h"
#include "yaml-cpp/yaml.h"
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <Eigen/Dense>

namespace emcl {

class Particle
{
public:
	Particle(double x, double y, double t, double w);

	struct Inv_Det{
		std::string Class;
		std::vector<Eigen::Vector2d> mean;
		std::vector<Eigen::Matrix2d> inv;
		std::vector<double> det;
	};

	double likelihood(LikelihoodFieldMap *map, Scan &scan, const int &valid_beams);
	// double vision_weight(const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node &landmark_config, double phi_th, double R_th, double w_img);
	double vision_weight(LikelihoodFieldMap *map, Scan &scan, const int &valid_beams, const bool use_vision, const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config, double phi_th, double R_th, double w_img, double ratio);
	double vision_weight(LikelihoodFieldMap *map, Scan &scan, const int &valid_beams, const bool use_vision, const yolov5_pytorch_ros::BoundingBoxes& bbox, const std::vector<Inv_Det>& inv_det, double w_img, double ratio);
	bool wallConflict(LikelihoodFieldMap *map, Scan &scan, double threshold, bool replace);
	Pose p_;
	double w_;

	Particle operator =(const Particle &p);
private:
	bool isPenetrating(double ox, double oy, double range, uint16_t direction, 
			LikelihoodFieldMap *map, double &hit_lx, double &hit_ly);

	bool checkWallConflict(LikelihoodFieldMap *map, double ox, double oy, 
			double range, uint16_t direction, double threshold, bool replace);

	void sensorReset(double ox, double oy,
		double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
		double range2, uint16_t direction2, double hit_lx2, double hit_ly2);
};

}

#endif

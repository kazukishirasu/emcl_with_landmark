//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EXP_PF_H__
#define EXP_PF_H__

#include "emcl/Mcl.h"

namespace emcl {

class ExpResetMcl : public Mcl
{
public: 
	ExpResetMcl(const Pose &p, int num, const Scan &scan,
			const std::shared_ptr<OdomModel> &odom_model,
			const std::shared_ptr<LikelihoodFieldMap> &map,
			double alpha_th, double open_space_th,
			double expansion_radius_position,
			double expansion_radius_orientation,
			const YAML::Node& landmark_config);
	~ExpResetMcl();

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t,
					  double t, bool inv, const bool use_vision,
					  const yolov5_pytorch_ros::BoundingBoxes& bbox,
					  const YAML::Node& landmark_config, const int w_img,
					  const double vision_ratio, const double sr_vision_ratio,
					  const double phi_th, const double radius_th,
					  const double particle_ratio);
	void build_kd_tree(const YAML::Node& landmark_config);
	void calc_inv_det(const YAML::Node& landmark_config);
private:
	double alpha_threshold_;
	double open_space_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;
	KD_Tree kdt;
	std::vector<ICP_Matching::Tree> tree_list_;
	std::vector<Particle::Inv_Det> inv_det_;

	void expansionReset(void);
	void vision_sensorReset(const Scan& scan,
							const yolov5_pytorch_ros::BoundingBoxes& bbox,
							const YAML::Node& landmark_config,
							const int w_img, const double radius_th,
							double particle_ratio, const double lidar_t);
};

}

#endif

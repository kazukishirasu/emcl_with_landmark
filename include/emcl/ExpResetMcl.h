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
					  double t, bool inv,
					  const yolov5_pytorch_ros::BoundingBoxes& bbox,
					  const YAML::Node& landmark_config,
					  const int w_img, const double ratio,
					  const double phi_th, const double R_th, const int B);
	void build_kd_tree(const YAML::Node& landmark_config);
private:
	double alpha_threshold_;
	double open_space_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;
	KD_Tree kdt;
	std::vector<ICP_Matching::Tree> tree_list_;

	void expansionReset(void);
	void vision_sensorReset(const Scan& scan,
							const yolov5_pytorch_ros::BoundingBoxes& bbox,
							const YAML::Node& landmark_config,
							const int w_img, const double R_th, const int B,
							const double t, const double lidar_t);
};

}

#endif

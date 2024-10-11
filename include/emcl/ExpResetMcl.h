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
			const yolov5_pytorch_ros::BoundingBoxes& bbox,
			const YAML::Node& landmark_config,
			const int w_img);
	~ExpResetMcl();

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);
private:
	double alpha_threshold_;
	double open_space_threshold_;
	double expansion_radius_position_;
	double expansion_radius_orientation_;
	const yolov5_pytorch_ros::BoundingBoxes& bbox_;
	const YAML::Node& landmark_config_;
	const int w_img_;

	void expansionReset(void);
	void vision_sensorReset(const yolov5_pytorch_ros::BoundingBoxes& bbox, const YAML::Node& landmark_config);
};

}

#endif

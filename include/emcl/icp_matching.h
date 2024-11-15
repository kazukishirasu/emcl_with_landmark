#ifndef ICP_MATCHING_H__
#define ICP_MATCHING_H__

#include <Eigen/Dense>
#include "emcl/kd_tree.h"

namespace emcl{

class ICP_Matching
{
public:
    struct Tree{
		std::string name;
		std::shared_ptr<KD_Tree::KdNode> node;
	};
	struct Landmark{
		std::string name;
		std::vector<KD_Tree::Point> points;
	};
	struct Data{
		KD_Tree::Point robot_pose;
		std::vector<Landmark> landmarks;
    };
    struct Pair{
        Eigen::MatrixX2d fix;
        Eigen::MatrixX2d move;
    };

    ICP_Matching();
    ~ICP_Matching();
    void matching(const std::vector<Tree>& tree_list, Data& data, size_t size);
private:
    KD_Tree kdt;
};

}

#endif
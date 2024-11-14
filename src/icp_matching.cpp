#include "emcl/icp_matching.h"

namespace emcl{

ICP_Matching::ICP_Matching()
{
}

ICP_Matching::~ICP_Matching()
{
}

void ICP_Matching::matching(const std::vector<Tree>& tree_list, Data& data, size_t size)
{
    Pair pair;
    pair.target.resize(size, 2);
    pair.source.resize(size, 2);
    int index = 0;
    for (const auto& landmark : data.landmarks){
        for (const auto& tree : tree_list){
            if (landmark.name == tree.name){
                for (const auto& point : landmark.points){
                    KD_Tree::Point best_point;
                    double best_dist = std::numeric_limits<double>::max();
                    kdt.search(tree.node, point, best_point, best_dist, 0);
                    pair.target(index, 0) = point.x;
                    pair.target(index, 1) = point.y;
                    pair.source(index, 0) = best_point.x;
                    pair.source(index, 1) = best_point.y;
                    index++;
                }
            }
        }
    }
    pair.target = pair.target.rowwise() - pair.target.colwise().mean();
    pair.source = pair.source.rowwise() - pair.source.colwise().mean();
    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < pair.target.rows(); i++){
        W += pair.target.row(i).transpose() * pair.source.row(i);
    }
}

}
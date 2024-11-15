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
    pair.fix.resize(size, 2);
    pair.move.resize(size, 2);
    Eigen::Matrix2d old_R = Eigen::Matrix2d::Zero();
    Eigen::Vector2d old_t = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < 10; i++){
        int index = 0;
        for (const auto& landmark : data.landmarks){
            for (const auto& tree : tree_list){
                if (landmark.name == tree.name){
                    for (const auto& point : landmark.points){
                        KD_Tree::Point best_point;
                        double best_dist = std::numeric_limits<double>::max();
                        kdt.search(tree.node, point, best_point, best_dist, 0);
                        pair.move(index, 0) = point.x;
                        pair.move(index, 1) = point.y;
                        pair.fix(index, 0) = best_point.x;
                        pair.fix(index, 1) = best_point.y;
                        index++;
                    }
                }
            }
        }
        Eigen::RowVector2d fix_mean = pair.fix.colwise().mean();
        Eigen::RowVector2d move_mean = pair.move.colwise().mean();
        pair.fix = pair.fix.rowwise() - fix_mean;
        pair.move = pair.move.rowwise() - move_mean;
        Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < pair.move.rows(); i++){
            W +=  pair.move.row(i).transpose() * pair.fix.row(i);
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // 回転行列 R
        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV();
        // 並進ベクトル t
        Eigen::Vector2d t = fix_mean.transpose() - R * move_mean.transpose();
        Eigen::Vector2d current_pose, update_pose;
        current_pose(0) = data.robot_pose.x;
        current_pose(1) = data.robot_pose.y;
        update_pose = R * current_pose + t;
        data.robot_pose.x = update_pose(0);
        data.robot_pose.y = update_pose(1);
        for (auto& lm : data.landmarks){
            for (auto& point : lm.points){
                current_pose(0) = point.x;
                current_pose(1) = point.y;
                update_pose = R * current_pose + t;
                point.x = update_pose(0);
                point.y = update_pose(1);
            }
        }
        double delta_R = (R - old_R).norm();
        double delta_t = (t - old_t).norm();
        if (delta_R < 0.000001 || delta_t < 0.2)
            return;
        old_R = R;
        old_t = t;
    }
    return;
}

}
#ifndef ARM_PLANNING_NODE_H
#define ARM_PLANNING_NODE_H

#include "arm_planning/common_include.h"


namespace arm_planning{

class Node{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Node> Ptr;

    Node() : cost_(0) {}


public:
    Eigen::VectorXd joint_pos_;
    std::vector<std::string> joint_names_;
    Eigen::Isometry3d tcp_pose_;
    VecEigenVec3 links_pos_;

    Ptr parent_ = nullptr;
    std::string note_ = ""; 
    // for rrtstar
    double cost_ = 0;
};


} // namespace

#endif
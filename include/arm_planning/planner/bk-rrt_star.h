#ifndef ARM_PLANNING_RRTSTAR_H
#define ARM_PLANNING_RRTSTAR_H

#include "arm_planning/common_include.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include "arm_planning/scene.h"
#include "arm_planning/robot.h"
#include "arm_planning/visualize.h"
#include "arm_planning/node.h"
#include "arm_planning/planner/planner_base.h"


namespace arm_planning{

class RRTStar: public PlannerBase
{
public:
    typedef std::shared_ptr<RRTStar> Ptr;

    RRTStar(
        const ros::NodeHandle& nh, 
        Scene::Ptr &scene
    );

    void loadParams();

    bool checkTwoNodeTcpPoseCanConnect(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    Node::Ptr extendToTcpPose(
        std::vector<Node::Ptr> &node_list,
        const Node::Ptr &from_node,
        const Node::Ptr &to_node,
        bool greedy = false
    );

    Node::Ptr oneStepSteerToTcpPose(
        const Node::Ptr &from_node,
        const Node::Ptr &to_node
    );

    bool solve(
        const PlanningRequest &req,
        PlanningResponse &res
    );


public:
    const std::string ALGORITHM_NAME = "RRTStar";

    // parameters
    double goal_bias_probability_;
    double pos_steer_step_size_;
    double angle_steer_step_size_;
    std::string jacobian_steer_type_;
    double extend_step_size_;
    double collision_check_step_size_;
    double neighborhood_radius_;
    double optimal_cost_to_come_;
    double cost_threshold_;
    std::string cost_function_;
    std::string sampling_strategy_;
    double steering_k_;
    double steering_d_;
    double steering_max_angle_;
    double steering_max_distance_;
    double steering_randomness_;


}; // end class


} // end namespace

#endif
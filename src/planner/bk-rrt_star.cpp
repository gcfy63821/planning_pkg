#include "arm_planning/planner/rrt_star.h"

namespace arm_planning {

// -------------------------------------------------------
RRTStar::RRTStar(
    const ros::NodeHandle& nh,
    Scene::Ptr &scene
): PlannerBase(nh, scene)
{
    b_node_tcp_pose_ = true; // in RRT*, b_node_tcp_pose_ must be true;

    loadParams();
}

// -------------------------------------------------------
void RRTStar::loadParams()
{
    std::string param_name;

    param_name = "planner_configs/" + ALGORITHM_NAME + "/goal_bias_probability";
    nh_.getParam(param_name, goal_bias_probability_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/max_iterations";
    nh_.getParam(param_name, rrt_max_iter_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/extend_step_size";
    nh_.getParam(param_name, extend_max_steps_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/path_collision_check_step_size";
    nh_.getParam(param_name, path_collision_check_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/joint_steer_step_size";
    nh_.getParam(param_name, joint_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/jacobian_steer_type";
    nh_.getParam(param_name, jacobian_steer_type_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/pos_steer_step_size";
    nh_.getParam(param_name, pos_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/angle_steer_step_size";
    nh_.getParam(param_name, angle_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/neighborhood_radius";
    nh_.getParam(param_name, neighborhood_radius_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/optimal_cost_to_come";
    nh_.getParam(param_name, optimal_cost_to_come_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/cost_threshold";
    nh_.getParam(param_name, cost_threshold_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/cost_function";
    nh_.getParam(param_name, cost_function_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/sampling_strategy";
    nh_.getParam(param_name, sampling_strategy_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/steering_k";
    nh_.getParam(param_name, steering_k_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/steering_d";
    nh_.getParam(param_name, steering_d_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/steering_max_angle";
    nh_.getParam(param_name, steering_max_angle_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/steering_max_distance";
    nh_.getParam(param_name, steering_max_distance_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/steering_randomness";
    nh_.getParam(param_name, steering_randomness_);
}

// -------------------------------------------------------
Node::Ptr RRTStar::extend(
    std::vector<Node::Ptr> &node_list,
    const Node::Ptr &nearest_node,
    const Node::Ptr &rand_node
){
    Node::Ptr new_node = oneStepSteerToTcpPose(nearest_node, rand_node);
    if (!new_node)
        return nullptr;

    // Check for collision
    if (!checkTwoNodeTcpPoseCanConnect(nearest_node, new_node))
        return nullptr;

    // Calculate the cost to the new node
    double new_cost = nearest_node->getCostToCome() + cost_function_(nearest_node, new_node);

    // Find nodes within the neighborhood radius for rewiring
    std::vector<Node::Ptr> nearby_nodes;
    for (const auto& node : node_list)
    {
        if (distanceFunction(node, new_node) < neighborhood_radius_)
            nearby_nodes.push_back(node);
    }

    // Check if the new node can improve the path cost for nearby nodes
    for (const auto& nearby_node : nearby_nodes)
    {
        double nearby_node_cost = nearby_node->getCostToCome();
        if (new_cost + costFunction(new_node, nearby_node) < nearby_node_cost)
        {
            nearby_node->setParent(new_node);
            nearby_node->setCostToCome(new_cost + costFunction(new_node, nearby_node));
        }
    }

    new_node->setCostToCome(new_cost);
    node_list.push_back(new_node);
    return new_node;
}

// -------------------------------------------------------
bool RRTStar::solve(
    const PlanningRequest &req,
    PlanningResponse &res
){
    // Initialize the RRT* tree
    std::vector<Node::Ptr> node_list;
    Node::Ptr start_node = std::make_shared<Node>(req.start);
    Node::Ptr goal_node = std::make_shared<Node>(req.goal);
    node_list.push_back(start_node);

    int iter = 0;
    Node::Ptr reached_node;

    // rrt main loop
    while (ros::ok() && iter < rrt_max_iter_)
    {
        Node::Ptr rand_node;
        if (rand() < goal_bias_probability_)
        {
            rand_node = goal_node;
        }
        else
        {
            rand_node = sampleFreeNode(req.workspace);
        }

        // Find the nearest node in the tree
        Node::Ptr nearest_node = getNearestNode(node_list, rand_node, "tcp_pose");

        // Extend the tree towards the random node
        reached_node = extend(node_list, nearest_node, rand_node);

        // Check if we reached the goal
        if (checkTwoNodeTcpPoseCanConnect(reached_node, goal_node))
        {
            // Smooth the path and return
            pathSmooth(req.res_);
            res.success_ = true;
            return true;
        }

        ROS_DEBUG_STREAM("solve(): rrt iter " << iter << " done.");
        iter++;
    }

    // If the loop ends without finding a path, return failure
    res.success_ = false;
    return false;
}

// -------------------------------------------------------
void RRTStar::rewireTree(
    std::vector<Node::Ptr> &node_list,
    const Node::Ptr &new_node
){
    // Find nodes within the neighborhood radius for rewiring
    std::vector<Node::Ptr> nearby_nodes;
    for (const auto& node : node_list)
    {
        if (twoNodeDistance(node, new_node, "tcp_pose") < neighborhood_radius_ && node != new_node)
            nearby_nodes.push_back(node);
    }

    // Check if the new node can improve the path cost for nearby nodes
    for (const auto& nearby_node : nearby_nodes)
    {
        double new_cost = new_node->getCostToCome() + costFunction(new_node, nearby_node);
        if (new_cost < nearby_node->getCostToCome())
        {
            nearby_node->setParent(new_node);
            nearby_node->setCostToCome(new_cost);
        }
    }
}

// ... (additional helper functions as needed)

} // end namespace
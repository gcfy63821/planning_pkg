#include "arm_planning/planner/rrt_star.h"


namespace arm_planning{



// -------------------------------------------------------
RRTStar::RRTStar(
    const ros::NodeHandle& nh,
    Scene::Ptr &scene
): PlannerBase(nh, scene)
{
    loadParams();
}


// -------------------------------------------------------
void RRTStar::loadParams()
{
    std::string param_name;

    param_name = "planner_configs/" + ALGORITHM_NAME + "/goal_bias_probability";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, goal_bias_probability_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/rrt_max_iter";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, rrt_max_iter_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/extend_max_steps";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, extend_max_steps_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/path_smooth_max_iter";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, path_smooth_max_iter_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/path_collision_check_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, path_collision_check_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/joint_steer_step_size";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, joint_steer_step_size_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/dist_metric";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, dist_metric_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/connect_joint_max_dist";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, connect_joint_max_dist_);

    if(dist_metric_ == "links_pos"){
        b_node_links_pos_ = true;
    }
    // new
    param_name = "planner_configs/" + ALGORITHM_NAME + "/rewire_radius";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, rewire_radius_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/max_neighbors";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, max_neighbors_);

    param_name = "planner_configs/" + ALGORITHM_NAME + "/optimality_tolerance";
    ROS_ERROR_STREAM_COND(!nh_.hasParam(param_name), "ROS param " << param_name << " doesn't exist.");
    nh_.getParam(param_name, optimality_tolerance_);
}


// -------------------------------------------------------
bool RRTStar::solve(
    const PlanningRequest &req,
    PlanningResponse &res
){
    // check input
    ROS_ERROR_STREAM_COND(req.goal_type_ != "joint_pos", ALGORITHM_NAME << "can only deal with the goal type of 'joint_pos'.");
    ROS_ERROR_COND(req.start_joint_pos_.size() != robot_->joint_num_, "The number of the joints of start configuration is wrong.");
    ROS_ERROR_COND(req.goal_joint_pos_.size() != robot_->joint_num_, "The number of the joints of goal configuration is wrong.");
    
    Node::Ptr start_node = std::make_shared<Node>();
    start_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.start_joint_pos_);
    start_node->note_ = "start";
    
    Node::Ptr goal_node = std::make_shared<Node>();
    goal_node->joint_pos_ = Utils::stdVector2EigenVectorXd(req.goal_joint_pos_);
    goal_node->note_ = "goal";

    ROS_ERROR_COND(checkNodeCollision(start_node), "The start node is in collision.");
    ROS_ERROR_COND(checkNodeCollision(goal_node), "The goal node is in collision.");

    if(b_node_links_pos_){
        updateNodeLinksPos(start_node);
        updateNodeLinksPos(goal_node);
    }

    if(req.b_visualize_start_goal_){
        visualizer_->publishText("start node");
        visualizer_->publishNode(start_node);
        ros::Duration(1).sleep();

        visualizer_->publishText("goal node");
        visualizer_->publishNode(goal_node);
        ros::Duration(1).sleep();
        visualizer_->publishText("planning ...");
    }
    
    std::vector<Node::Ptr> node_list{start_node};
    std::vector<Node::Ptr> res_path_list;

    std::chrono::steady_clock::time_point t_begin, t_end;
    std::chrono::duration<double> time_used;

    // rrt main loop
    t_begin = std::chrono::steady_clock::now();
    ros::Rate rate(1);
    int iter = 0;
    while(ros::ok() && iter < rrt_max_iter_){

        Node::Ptr rand_node = (Utils::getRandomDouble() < goal_bias_probability_) ? goal_node : randomSampleNode();

        Node::Ptr nearest_node = getNearestNode(node_list, rand_node, dist_metric_);
        Node::Ptr new_node = extend(node_list, nearest_node, rand_node);

        if (new_node) {
            new_node->cost_ = nearest_node->cost_ + twoNodeDistance(nearest_node, new_node, dist_metric_); // Update the cost

            node_list.push_back(new_node);

            // Rewire nearby nodes
            rewire(node_list, new_node);

            if (checkTwoNodeCanConnect(new_node, goal_node)){
                res_path_list = pathExtract(new_node, goal_node);
                
                t_end = std::chrono::steady_clock::now();
                time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin);
                ROS_INFO_STREAM(ALGORITHM_NAME + " find feasible path, path length: " << res_path_list.size() 
                    << ", time cost: " << time_used.count() << "s" << ", iter: " << iter);

                for (int i = 0; i < path_smooth_max_iter_; ++i) {
                    pathSmooth(res_path_list);
                }

                t_end = std::chrono::steady_clock::now();
                time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin);
                ROS_INFO_STREAM("Smoothed path length: " << res_path_list.size() 
                    << ", total time cost: " << time_used.count() << "s");

                res.total_time_cost_ = time_used.count();
                res.total_iter_ = iter;
                pathNode2Vector(res_path_list, res.path_);
                res.success_ = true;
                return true;
            }
        }

        iter++;
    }

    res.success_ = false;
    ROS_WARN_STREAM("Failed to find feasible path.");
    return false;
}

// -------------------------------------------------------
void RRTStar::rewire(std::vector<Node::Ptr>& node_list, Node::Ptr new_node) {
    for (auto& node : node_list) {
        if (node != new_node && twoNodeDistance(new_node, node, dist_metric_) < rewire_radius_) {
            if (checkTwoNodeCanConnect(new_node, node)) {
                // If the path through new_node is better, update node's parent
                double new_cost = new_node->cost_ + twoNodeDistance(new_node, node, dist_metric_);
                if (new_cost < node->cost_) {
                    node->parent_ = new_node;
                    node->cost_ = new_cost; // Update the cost
                }
            }
        }
    }
}


} // end namespace

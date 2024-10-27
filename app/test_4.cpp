/*
date:2024.1.25
rqc
测试能否获取当前的位姿

*/
#include "arm_planning/common_include.h"
#include "arm_planning/planning_interface.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"

using namespace arm_planning;


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

   

    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    PlanningRequest req;
    PlanningResponse res;
    req.group_name_ = "panda_manipulator";


    
    //initial joint position
    req.start_joint_pos_ = moveit::planning_interface::MoveGroup::getCurrentJointValues();//test?
    // req.start_joint_pos_ = std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 1.61, 0.86};// ��ʼ�ؽ�λ��
    req.goal_type_ = "tcp_pose";
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = -0.25337;
    goal_pose.position.y = 0.414;
    goal_pose.position.z = -0.110;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 1.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;
    req.goal_pose_ = goal_pose;

    req.b_visualize_start_goal_ = true;// �Ƿ���ӻ���ʼ��Ŀ��λ��
    req.b_visualize_planning_process_ = false;// �Ƿ���ӻ��滮����
    req.b_visualize_res_path_ = true;// �Ƿ���ӻ��滮���·��

    std::string algorithm_name = "JTRRT";
    const int num_test = 3;
    double time_cost = 0.0;
    int num_success = 0;
    int iter = 0;

    for (size_t i = 0; ros::ok() && i < num_test; i++) {
        std::cout << "test " << i << ": " << std::endl;
        pi->solve(algorithm_name, req, res);
        if (res.success_) {
            num_success++;
            time_cost += res.total_time_cost_;
            iter += res.total_iter_;
        }
    }

    std::cout << "Results for JTRRT: success rate: " << double(num_success) / double(num_test) << std::endl;
    std::cout << "Results for JTRRT: average time cost: " << time_cost / double(num_success) << std::endl;
    std::cout << "Results for JTRRT: average iteration: " << iter / double(num_success) << std::endl;

    ros::Duration(0.2).sleep();
    return 0;
}


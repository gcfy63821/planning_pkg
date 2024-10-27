#include "ros/ros.h"
#include <iostream>
#include "actionlib/client/simple_action_client.h"
#include "demo_actions/CountNumberAction.h"
 
using namespace std;
 
// 结果回调
void done_cb(const actionlib::SimpleClientGoalState &state,
             const demo_actions::CountNumberResultConstPtr &result,
             const actionlib::SimpleActionClient<demo_actions::CountNumberAction> *client) {
    if (state == state.ABORTED) {
        ROS_INFO("server working error, don't finish my job.");
    } else if (state == state.PREEMPTED) {
        ROS_INFO("client cancel job.");
    } else if (state == state.SUCCEEDED) {
        ROS_INFO("server finish job.");
        ROS_INFO_STREAM("result: " << result->count);
    }
}
 
// 激活回调
void active_cb() {
    ROS_INFO_STREAM("active callback");
}
 
//过程反馈
void feedback_cb(const demo_actions::CountNumberFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM("percent: " << feedback->percent);
}
 
int main(int argc, char **argv) {
    string nodeName = "CountNumberClient";
    string actionName = "/count_number";
 
    // 创建节点
    ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
    ros::NodeHandle node;
 
    // 创建client
    actionlib::SimpleActionClient<demo_actions::CountNumberAction> &&client = actionlib::SimpleActionClient<demo_actions::CountNumberAction>(
            node, actionName);
    client.waitForServer();
 
    // 发送
    demo_actions::CountNumberGoal goal;
    goal.max = 100;
    goal.duration = 0.5;
    client.sendGoal(goal,
                    boost::bind(&done_cb, _1, _2, &client),
                    boost::bind(&active_cb),
                    boost::bind(&feedback_cb, _1));
    // 阻塞线程
    ros::spin();
 
    return 0;
}
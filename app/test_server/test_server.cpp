#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include <iostream>
#include "demo_actions/CountNumberAction.h"
 
using namespace std;
 
void execute_callback(const demo_actions::CountNumberGoalConstPtr &goal,
                       actionlib::SimpleActionServer<demo_actions::CountNumberAction> *server) {
    ROS_INFO_STREAM("max: " << goal->max);
    ROS_INFO_STREAM("duration: " << goal->duration);
    //进度操作
    ros::Rate rate(1.0 / goal->duration);
    int count = 0;
    while (count < goal->max) {
        // 处理进度，反馈进度
        demo_actions::CountNumberFeedback feedback;
        feedback.percent = count * 100.0 / goal->max;
        server->publishFeedback(feedback);
        rate.sleep();
        count++;
    }
 
    // 响应结果
    demo_actions::CountNumberResult result;
    result.count = count;
    server->setSucceeded(result);
}
 
int main(int argc, char **argv) {
    string nodeName = "CountNumberServer";
    string actionName = "/count_number";
    //创建节点
    ros::init(argc, argv, nodeName, ros::init_options::AnonymousName);
    ros::NodeHandle node;
 
    //创建server
    actionlib::SimpleActionServer<demo_actions::CountNumberAction> server(node, actionName,boost::bind(&execute_callback, _1,&server),false);
    server.start();
 
    //阻塞
    ros::spin();
    return 0;
}
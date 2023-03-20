#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "rs_planner/reeds_shepp.h"

class RSPlanner
{
public:
    RSPlanner(ros::NodeHandle &nh)
    {
        goal_pose_sub = nh.subscribe("move_base_simple/goal", 1, &RSPlanner::goalPoseCallback, this);
        forward_path_pub = nh.advertise<nav_msgs::Path>("/forwrad_path", 1);
        backward_path_pub = nh.advertise<nav_msgs::Path>("/backward_path", 1);
    }
    
private:
    ros::Subscriber goal_pose_sub;
    ros::Publisher backward_path_pub;
    ros::Publisher forward_path_pub;
    void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

void RSPlanner::goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double turningRadius = 1.0;
    ReedsSheppStateSpace rs(turningRadius);
    double q0[3] = {0.0, 0.0, 0.0};
    double q1[3] = {msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation)};
    double step_size = 0.1;
    std::vector<std::vector<double> > points;
    rs.sample(q0, q1, step_size, points);
    nav_msgs::Path for_path_msg;
    for_path_msg.header.frame_id = "/map";
    for_path_msg.header.stamp = ros::Time::now();
    nav_msgs::Path back_path_msg;
    back_path_msg.header.frame_id = "/map";
    back_path_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < points.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = points[i][0];
        pose.pose.position.y = points[i][1];
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(points[i][2]);
        if (points[i][4] > 0)
        {
            for_path_msg.poses.push_back(pose);
            ROS_INFO("Forward, x: %f, y: %f, yaw: %f", points[i][0], points[i][1], points[i][2]);
        }
        else
        {
            back_path_msg.poses.push_back(pose);
            ROS_INFO("Backward, x: %f, y: %f, yaw: %f", points[i][0], points[i][1], points[i][2]);
        }
    }
    forward_path_pub.publish(for_path_msg);
    backward_path_pub.publish(back_path_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_planner_node");
    ros::NodeHandle nh;
    RSPlanner rs_planner(nh);
    ros::spin();
    return 0;
}
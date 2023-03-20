#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include "rs_planner/reeds_shepp.h"

class RSPlanner
{
public:
    RSPlanner(ros::NodeHandle &nh)
    {
        goal_pose_sub = nh.subscribe("move_base_simple/goal", 1, &RSPlanner::goalPoseCallback, this);
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis_path", 1);
    }
    
private:
    ros::Subscriber goal_pose_sub;
    ros::Publisher vis_pub;
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
    visualization_msgs::MarkerArray path;
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.ns = "path";
    marker.action = visualization_msgs::Marker::DELETEALL;
    path.markers.push_back(marker);
    vis_pub.publish(path);
    path.markers.resize(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = i;
        marker.pose.position.x = points[i][0];
        marker.pose.position.y = points[i][1];
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(points[i][2]);
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        if (points[i][4] > 0)
        {
            marker.ns = "forward";
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            ROS_INFO("forward, x: %f, y: %f, yaw: %f", points[i][0], points[i][1], points[i][2]);
        }
        else
        {
            marker.ns = "backward";
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            ROS_INFO("backward, x: %f, y: %f, yaw: %f", points[i][0], points[i][1], points[i][2]);
        }
        path.markers[i] = marker;
    }
    vis_pub.publish(path);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rs_planner_node");
    ros::NodeHandle nh;
    RSPlanner rs_planner(nh);
    ros::spin();
    return 0;
}
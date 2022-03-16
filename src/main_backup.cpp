#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <list>
#include <iostream>
#include <fstream>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

std::list<geometry_msgs::PoseStamped> readFile(std::string filename) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream input_file(filename);
    std::list<geometry_msgs::PoseStamped> waypoint_list;
    geometry_msgs::PoseStamped waypoint;
    int seq = 0;
    if (!input_file.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return waypoint_list;
    }
    while (getline(input_file, line)) {
        lines.push_back(line);
    }
    input_file.close();

    for (auto l : lines) {
        int i = 0;
        int j = l.find(',');
        auto x = l.substr(i,j);
        i = j;
        j = l.find(',',j+1);
        auto y = l.substr(i+1,j-i-1);
        i = j+1;
        j = l.find(',',j+1);
        auto z = l.substr(i,j-i);
        i = j+1;
        j = l.find(',',j+1);
        auto roll = l.substr(i,j-i);
        i = j+1;
        j = l.find(',',j+1);
        auto pitch = l.substr(i,j-i);
        i = j+1;
        j = l.find(',',j+1);
        auto yaw = l.substr(i,j-i);
        waypoint.header.seq = seq;
        waypoint.header.frame_id = "odom";
        //std::cout << x << " and " << y << " and " << z << " and " << roll <<
        // " and " << pitch << " and " << yaw << std::endl;
        waypoint.pose.position.x = std::stoi(y);
        waypoint.pose.position.y = std::stoi(x);
        waypoint.pose.position.z = std::stoi(z);
        tf2::Quaternion rot;
        rot.setEuler(std::stoi(roll),std::stoi(pitch),std::stoi(yaw));
        waypoint.pose.orientation.w = rot.getW();
        waypoint.pose.orientation.x = rot.getX();
        waypoint.pose.orientation.y = rot.getY();
        waypoint.pose.orientation.z = rot.getZ();
        waypoint_list.push_back(waypoint);
        seq++;
    }
    return waypoint_list;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_list");

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
  move_base_msgs::MoveBaseGoal goal;

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  auto waypoints = readFile("waypoints.txt");

  while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("move_base action server is up");

  std::list<geometry_msgs::PoseStamped>::iterator iter = waypoints.begin();
  geometry_msgs::PoseArray array;
  array.header.frame_id = "/odom";
  array.header.stamp = ros::Time::now();
  array.header.seq = 1;
  while(iter != waypoints.end()) {
    array.poses.push_back(iter->pose);
    iter++;
  }
  std::cout << array << std::endl;
  pub.publish(array);
  ros::spinOnce();

  iter = waypoints.begin();

  while(iter != waypoints.end()) {
    iter->header.stamp = ros::Time::now();
    goal.target_pose = *iter;
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Waypoint goal has SUCCEEDED");
      }
    if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        ROS_INFO("Waypoint goal was ABORTED");
        break;
      }
    ROS_INFO("Moving onto next");
    iter++;
  }

  ROS_INFO("Successfully published waypoints");
  return 0;
}

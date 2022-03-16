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
#include <waypoint_list_server/ReadWaypointList.h>

bool readFile(waypoint_list_server::ReadWaypointList::Request &req,
              waypoint_list_server::ReadWaypointList::Response &res){
    std::vector<std::string> lines;
    std::string line;
    std::cout << req.filename << std::endl;
    std::ifstream input_file(req.filename);
    std::list<geometry_msgs::PoseStamped> waypoint_list;
    geometry_msgs::PoseStamped waypoint;
    int seq = 0;
    if (!input_file.is_open()) {
        std::cerr << "Failed to open file" << std::endl;
        return false;
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
        waypoint.header.frame_id = "map";
        //std::cout << x << " and " << y << " and " << z << " and " << roll <<
        // " and " << pitch << " and " << yaw << std::endl << std::endl;
        waypoint.pose.position.x = std::stof(x);
        waypoint.pose.position.y = std::stof(y);
        waypoint.pose.position.z = std::stof(z);
        tf2::Quaternion rot;
        rot.setEuler(std::stof(roll),std::stof(pitch),std::stof(yaw));
        waypoint.pose.orientation.w = rot.getW();
        waypoint.pose.orientation.x = rot.getX();
        waypoint.pose.orientation.y = rot.getY();
        waypoint.pose.orientation.z = rot.getZ();
        waypoint_list.push_back(waypoint);
        seq++;
        //std::cout << waypoint << std::endl;
    }

    //Convert from "std::list<geometry_msgs::PoseStamped>"" to geometry_msgs::PoseArray
    std::list<geometry_msgs::PoseStamped>::iterator iter = waypoint_list.begin();
    geometry_msgs::PoseArray array;
    array.header.frame_id = "";
    array.header.stamp = ros::Time::now();
    array.header.seq = 1;
    while(iter != waypoint_list.end()) {
      array.poses.push_back(iter->pose);
      iter++;
    }
    res.list = array;
    //std::cout << res.list << std::endl;
    ROS_INFO("Successfully sent waypoints to client");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_list");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("waypoint_list_server", readFile);
  ROS_INFO("Ready to retrieve waypoint list.");
  ros::spin();

  return 0;
}

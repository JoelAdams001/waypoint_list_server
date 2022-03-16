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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_viz");
  ros::NodeHandle n;
  ros::Publisher viz_pub = n.advertise<geometry_msgs::PoseArray>("waypoints", 10);
  ros::ServiceClient client = n.serviceClient<waypoint_list_server::ReadWaypointList>("waypoint_list_server");
  waypoint_list_server::ReadWaypointList srv;
  srv.request.filename = "waypoints.txt";
  geometry_msgs::PoseArray list;
  if (client.call(srv))
    {
        list = srv.response.list;
    }
    else
    {
        ROS_ERROR("Failed to call service waypoint_list_server");
        return 1;
    }
  viz_pub.publish(list);
  ros::spin();
  return 0;
}

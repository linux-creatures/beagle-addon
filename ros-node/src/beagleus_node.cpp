#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

using namespace std;

namespace hc_sr04_node {

// Maximum distance reported. Values over this distance
// report MAX_DISTANCE. TODO make this a property.
const static float MAX_DISTANCE = 400;
const static float DIST_SCALE = 58.0;
const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;


int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "hc_sr04s");
  ros::NodeHandle node;
  ros::Rate rate(800);  // 800 hz for 4 sensors: 200Hz

  // TODO: config these
  vector<hc_sr04_node::Sonar> sonars;
  sonars.push_back(hc_sr04_node::Sonar(24, 25));
  sonars.push_back(hc_sr04_node::Sonar(22, 23));

  // Build a publisher for each sonar.
  vector<ros::Publisher> sonar_pubs;
  for (int i = 0; i < sonars.size(); ++i) {
    stringstream ss;
    ss << "sonar_" << i;
    sonar_pubs.push_back(node.advertise<sensor_msgs::Range>(ss.str(), 10));
  }
  
  // Build base range message that will be used for
  // each time a msg is published.
  sensor_msgs::Range range;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.min_range = 0.0;
  range.max_range = hc_sr04_node::MAX_DISTANCE;
 
  float distance;
  bool error;
  while(ros::ok()) {    
    for (int i = 0; i < sonars.size(); ++i) {
      range.header.stamp = ros::Time::now();
      range.range = sonars[i].distance(&error);
      if (error)
	       ROS_WARN("Error on sonar %d", i);

      else{
	       sonar_pubs[i].publish(range);
    }
  }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

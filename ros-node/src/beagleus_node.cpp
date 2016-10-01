#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

using namespace std;


// Maximum distance reported. Values over this distance
// report MAX_DISTANCE. TODO make this a property.
const static float MAX_DISTANCE = 400;
const static float DIST_SCALE = 58.0;
const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;


int main(int argc, char **argv) {

  // Start ROS node.
  string 
  ROS_INFO("Starting node");
  ros::init(argc, argv, "sonar rosnode");
  ros::NodeHandle node;
  ros::Rate rate(200);  // 200 Hz


  // Build a publisher for each sonar.
  vector<ros::Publisher> sonar_x;
  vector<ros::Publisher> sonar_y;

  ros::Publisher sonarx_pub = node.advertise<sensor_msgs::Range>("sonar_x", 10);
  ros::Publisher sonary_pub = node.advertise<sensor_msgs::Range>("sonar_y", 10);

  
  // Build base range message that will be used for
  // each time a msg is published.
  sensor_msgs::Range range;
 
  float distance;
  bool error;
  while(ros::ok()) { 

      ifstream sonar_left ("/dev//dev/rpmsg_pru30");
      sonar_left >> string_left;
      range.header.stamp = ros::Time::now();
      range.range = distance;
      if (error)
	       ROS_WARN("Error on /us/sonar_x for the left sensor");

      else{
	       sonarx_pub.publish(range);
    }


      range.header.stamp = ros::Time::now();
      range.range = distance+200
      if (error)
         ROS_WARN("Error on /us/sonar_x for the right sensor");

      else{
         sonarx_pub.publish(range);
    }


      ifstream sonar_right ("/dev//dev/rpmsg_pru31");
      sonar_right >> string_right;
      range.header.stamp = ros::Time::now();
      range.range = distance+400
      if (error)
         ROS_WARN("Error on /us/sonar_y for the front sensor");

      else{
         sonary_pub.publish(range);
    }

      range.header.stamp = ros::Time::now();
      range.range = distance+600
      if (error)
         ROS_WARN("Error on /us/sonar_y for the back sensor");

      else{
         sonary_pub.publish(range);
    }


    ros::spinOnce();
    rate.sleep();    
  }
  
  return 0;
}
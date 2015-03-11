#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <iostream>

using namespace std;

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "button_node");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<std_msgs::String>("/button_node", 1000);
  ros::Rate loop_rate(10);
  int input = 0;
  std_msgs::String msg;
  std::stringstream ss;
  while(ros::ok()) {
    cin >> input;
    ss << input << endl;
    msg.data = ss.str();
    if( input == 1 ) { 
      pub.publish(msg);
    }
    input = 0;
    ros::spinOnce();
  }
  return 0;

}

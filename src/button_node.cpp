#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "button_node");
  ros::NodeHandle node;
  int input = 0;
  ros::Publisher pub = node.advertise<std_msgs::String>("/button_node", 1000);
  ros::Rate loop_rate(10);
  loop_rate.sleep();
  loop_rate.sleep();
  string line;
  std_msgs::String msg;
  std::stringstream ss;
  /*while(ros::ok() && cin >> input) {
    ss << input << endl;
    msg.data = ss.str();
    if( input == 1 ) { 
      pub.publish(msg);
      ros::spinOnce();
    } else  {
      cout << "Sign in failed please try again." << endl;
    }
    input = 0;
  } */

  while(ros::ok()) {
    ifstream myfile("/sys/class/gpio/gpio7/value");
    if(myfile.is_open()) {
      getline(myfile, line);
      msg.data = line.c_str();
      if( line == "1" )
        pub.publish(msg);
    }
    else {
      cout << "File not found" << endl;
    }
    ros::spinOnce();
  }
  return 0;

}

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
#include <iostream>

using namespace std;

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "button_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(10);

  ros::Publisher pub = node.advertise<std_msgs::String>("/button_node", 1000);
  int input = 0;
  loop_rate.sleep();
  loop_rate.sleep();
  std_msgs::String msg;
  std::stringstream ss;
  while(ros::ok() && cin >> input) {
    cout << "Press button to sign in user: ";
    ss << input << endl;
    msg.data = ss.str();
    if( input == 1 ) { 
      pub.publish(msg);
      ros::spinOnce();
    } else  {
      cout << "Sign in failed please try again." << endl;
    }
    input = 0;
  }
  return 0;

}

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Int64.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

#define THRESHOLD 4

//image_transport::Publisher image_pub;
Mat prev, current, next;
ros::Publisher pub;
int previous;
int baseHeight, base_x, face_radius;
bool toggleImg;
bool faceDetected;

void faceDetectAction( const std_msgs::String::ConstPtr & msg  ) {
  stringstream ss;
  faceDetected = true;
  ss << msg->data.c_str();
  ss >> baseHeight >> base_x >> face_radius;
}

void drawMap( const Mat& current, Mat& result, int step, int threshold) {
  Mat blue(result.size(), result.type()), green(result.size(), result.type()), 
      red(result.size(), result.type());
  int blueC = 0, greenC = 0, redC = 0;
  int maxCols = base_x + face_radius * 1.5;
  int minCols = base_x - face_radius * 1.5;
  if( baseHeight > result.rows )
    baseHeight = 0;
  if( base_x + face_radius*1.5 > result.cols)
    maxCols = result.cols;
  if( base_x - face_radius*1.5 < 0)
    minCols = 0;
  for(int y = baseHeight; y < result.rows; y += step)
    for(int x = minCols; x < maxCols; x += step)
    {
      Vec3b pixel = current.at<Vec3b>(y, x);
      // if( pixel[2] > 110 && pixel[1] < 160 && pixel[0] < 50 ){
      if( pixel[2] > pixel[1] + pixel[0] - (pixel[1] + pixel[0])/threshold ){
        blue.at<uchar>(Point(x,y)) = 255;
        blueC++;
      }
      else {
        blue.at<uchar>(Point(x,y)) = 0;
      }
      // if( pixel[2] < 135 && pixel[1] > 100 && pixel[0] < 110 ) {
      if( pixel[2] + pixel[0] - (pixel[2] + pixel[0])/threshold < pixel[1] ) {
        green.at<uchar>(Point(x,y)) = 255;
        greenC++;
      }
      else {
        green.at<uchar>(Point(x,y)) = 0;
      }
      // if( pixel[2] < 50 && pixel[1] < 85 && pixel[0] > 120 ){
      if( pixel[2] + pixel[1] - (pixel[2] + pixel[1])/threshold < pixel[0] ){
        red.at<uchar>(Point(x,y)) = 255;
        redC++;
      }
      else {
        red.at<uchar>(Point(x,y)) = 0;
      }

    }
  cout << "Please wait...." << endl;
  int currentColor = 0;
  std_msgs::String msg;
  if( blueC > greenC ) {
    if( blueC > redC ) {
      result = blue;
      currentColor = 0;
      msg.data = "blue";
    }
    else {
      result = red;
      currentColor = 1;
      msg.data = "red";
    }
  }
  else {
    if( greenC > redC ) {
      result = green;
      currentColor = 2;
      msg.data = "green";
    } else {
      result = red;
      currentColor = 1;
      msg.data = "red";
    }
  }

  if( currentColor != previous ) {
    if(previous < 3 ) {
      cout << "\033[2J\033[1;1H";
      if(toggleImg) {
        cout << "\n\n"
          << "\tBBBBBBBBBB  \tEEEEEEEEEE \tEEEEEEEEEE \tRRRRRRRRR \t      .ssssssssssss.   \n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t     |@@ssssssssss@@|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t  ___|s@@sss@@sss@@s|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t / __ .ss@@@sss@@ss.| \n"
          << "\tBB     BB   \tEE         \tEE         \tRR     RR \t| /  |...ssss@sss...|\n"
          << "\tBBBBBBB     \tEEEEEEEEEE \tEEEEEEEEEE \tRRRRRRR   \t| |  |.....sss@.....|\n"
          << "\tBB     BB   \tEE         \tEE         \tRR     RR \t| \\  |.....s@.......|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t \\ --.....s@........|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t  \\__...............|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t     |..............|\n"
          << "\tBBBBBBBBBB  \tEEEEEEEEEE \tEEEEEEEEEE \tRR      RR\t     |..............|\n"
          << "\n\n " << endl;
        toggleImg = !toggleImg;
      } else {
        cout << "\n\n"
          << "\tBBBBBBBBBB  \tEEEEEEEEEE \tEEEEEEEEEE \tRRRRRRRRR \t        .ssssssssss.   \n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t     |@@ssssss@sss@@|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t  ___|ss@@sssssss@@s|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t / __ ..s@@ssss@@ss.| \n"
          << "\tBB     BB   \tEE         \tEE         \tRR     RR \t| /  |....ss@@sss...|\n"
          << "\tBBBBBBB     \tEEEEEEEEEE \tEEEEEEEEEE \tRRRRRRR   \t| |  |.....s@ss.....|\n"
          << "\tBB     BB   \tEE         \tEE         \tRR     RR \t| \\  |.....ss.......|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t \\ --......ss.......|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t  \\__...............|\n"
          << "\tBB       BB \tEE         \tEE         \tRR      RR\t     |..............|\n"
          << "\tBBBBBBBBBB  \tEEEEEEEEEE \tEEEEEEEEEE \tRR      RR\t     |..............|\n"
          << "\n\n" << endl;
        toggleImg = !toggleImg;
      }
      cout << "Press button to change user. " << endl;
    }
    cout << "You are now signed in as ";
    if(currentColor == 0)
      cout << "blue";
    else if(currentColor == 1)
      cout << "red";
    else
      cout << "Jesus";
    cout << "." << endl;
    previous = currentColor;
    pub.publish(msg);
    cout.flush();
  } else {
    cout << "You are already signed in." << endl;
  }
  faceDetected = false;
}

void faceDetect( const sensor_msgs::Image::ConstPtr & msg ) {
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
  }
  catch (cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  current = cv_ptr->image;
  
  if(prev.data) {
    if(faceDetected) {
      cout << "Retrieving user's information. ";
      cout.flush();
      drawMap(current, cv_ptr->image, 1, THRESHOLD);
    }
  }
  prev = current;
  // image_pub.publish(cv_ptr->toImageMsg());

}

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "facerec");
  baseHeight = 0;
  previous = 5;
  faceDetected = false;
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  pub = node.advertise<std_msgs::String>("/chatter", 1000);
  image_transport::Subscriber mySub = it.subscribe("/camera/visible/image", 
                                                    1, faceDetect);
  ros::Subscriber detect_sub = node.subscribe("/face_detect",1000, faceDetectAction);
  ros::Rate loop_rate(10);
  loop_rate.sleep();
  cout << "\033[2J\033[1;1H";
  cout << "\n\n"
    << "\tBBBBBBBBBB  \tEEEEEEEEEE \tEEEEEEEEEE \tRRRRRRRRR \t      .ssssssssssss.   \n"
    << "\tBB       BB \tEE         \tEE         \tRR      RR\t     |@@ssssssssss@@|\n"
    << "\tBB       BB \tEE         \tEE         \tRR      RR\t  ___|s@@ssssssss@@s|\n"
    << "\tBB       BB \tEE         \tEE         \tRR      RR\t / __ .ss@@ssss@@ss.| \n"
    << "\tBB     BB   \tEE         \tEE         \tRR     RR \t| /  |...sss@@sss...|\n"
    << "\tBBBBBBB     \tEEEEEEEEEE \tEEEEEEEEEE \tRRRRRRR   \t| |  |.....s@ss.....|\n"
    << "\tBB     BB   \tEE         \tEE         \tRR     RR \t| \\  |.....s@.......|\n"
    << "\tBB       BB \tEE         \tEE         \tRR      RR\t \\ --.....ss........|\n"
    << "\tBB       BB \tEE         \tEE         \tRR      RR\t  \\__...............|\n"
    << "\tBB       BB \tEE         \tEE         \tRR      RR\t     |..............|\n"
    << "\tBBBBBBBBBB  \tEEEEEEEEEE \tEEEEEEEEEE \tRR      RR\t     |..............|\n"
    << endl;
  cout << "Press button to sign in user: ";
  cout.flush();
  ros::spin();

  return 0;
}

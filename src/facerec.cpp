#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

#define THRESHOLD 4

image_transport::Publisher image_pub;
Mat prev, current, next;
int previous;

void drawMap( const Mat& current, Mat& result, int step, int threshold) {
  Mat blue(result.size(), result.type()), green(result.size(), result.type()), 
      red(result.size(), result.type());
  int blueC = 0, greenC = 0, redC = 0;
  for(int y = 0; y < result.rows; y += step)
    for(int x = 0; x < result.cols; x += step)
    {
      //if(x == 100 && y == 100)
      //cout << current.at<Vec3b>(y, x) << endl;
      //const Point2f& fxy = current.at<Point2f>(y, x);
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
  //red.at<uchar>(Point(100,100)) = 255;
  int currentColor = 0;
  if( blueC > greenC ) {
    if( blueC > redC ) {
      result = blue;
      currentColor = 0;
    }
    else {
      result = red;
      currentColor = 1;
    }
  }
  else {
    if( greenC > redC ) {
      result = green;
      currentColor = 2;
    } else {
      result = red;
      currentColor = 1;
    }
  }

  if( currentColor != previous ) {
    cout << "The prominent color is ";
    if(currentColor == 0)
      cout << "blue";
    else if(currentColor == 1)
      cout << "red";
    else
      cout << "green";
    cout << endl;
    previous = currentColor;
  }
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
    drawMap(current, cv_ptr->image, 1, THRESHOLD);
  }
  prev = current;
  // image_pub.publish(cv_ptr->toImageMsg());

}

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "facerec");
  //prev = 0;
  previous = 0;
  //-- 1. Load the cascades
  // if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
  // if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_transport::Subscriber mySub = it.subscribe("/camera/visible/image", 
                                                    1, faceDetect);
  image_pub = it.advertise("/raw_image", 1);
  ros::spin();

  return 0;
}

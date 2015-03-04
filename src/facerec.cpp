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

image_transport::Publisher image_pub;
Mat prev, current, next;

/** Function Headers */
void detectAndDisplay( Mat frame );

/** Global variables */
string face_cascade_name = "haarcascade_frontalface_alt.xml";
string eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";
RNG rng(12345);

void drawMap( const Mat& current, Mat& result, int step, double threshold) {
  Mat blue, green, red;
  int blueC = 0, greenC = 0, redC = 0;
  for(int y = 0; y < result.rows; y += step)
    for(int x = 0; x < result.cols; x += step)
    {
      //const Point2f& fxy = current.at<Point2f>(y, x);
      Vec3b pixel = current.at<Vec3b>(y, x);
      if( pixel[2] > 110 && pixel[1] < 160 && pixel[0] < 50 ){
        blue.at<uchar>(Point(x,y)) = 255;
        blueC++;
      }
      else {
        blue.at<uchar>(Point(x,y)) = 0;
      }

    }
  if( blueC > greenC ) {
    if( blueC > redC )
      result = blue;
    else
      result = red;
  }
  else {
    if( greenC > redC )
      result = green;
    else 
      result = red;
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
    drawMap(current, cv_ptr->image, 1, 255);
  }
  prev = current;
  image_pub.publish(cv_ptr->toImageMsg());

}

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "facerec");
  prev = 0;
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

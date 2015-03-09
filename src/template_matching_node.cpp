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
#include <sys/stat.h>

using namespace std;
using namespace cv;

image_transport::Publisher image_pub;
Mat prev, current, next;
Mat myTemplate;


Mat templateMatch( Mat &img, Mat &mytemplate )
{
  Mat result;

  /// Create the result matrix
  int result_cols =  img.cols - myTemplate.cols + 1;
  int result_rows = img.rows - myTemplate.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  return result;
}

void match( const sensor_msgs::Image::ConstPtr & msg ) {
  cv_bridge::CvImagePtr cv_ptr;
  Point match;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
  }
  catch (cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  current = cv_ptr->image;
  Rect ROI;

  if(prev.data) {
    current = templateMatch( cv_ptr->image, myTemplate);
   
    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( current, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
   // if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
   //   {
//   matchLoc = minLoc;
    //}
   // else
     // {
    matchLoc = maxLoc;
    //}

    /// Show me what you got
  //  rectangle( img_display, matchLoc, Point( matchLoc.x + myTemplate.cols , matchLoc.y + myTemplate.rows ),
   //            Scalar::all(0), 2, 8, 0 );
    rectangle( cv_ptr->image, matchLoc, Point( matchLoc.x + myTemplate.cols , matchLoc.y + myTemplate.rows ),
               Scalar::all(0), 2, 8, 0 );

    ROI = cv::Rect( match.x, match.y, myTemplate.cols, myTemplate.rows );
    cv_ptr->image(ROI);
  }
  prev = current;
  image_pub.publish(cv_ptr->toImageMsg());

}

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "template_matching_node");
  ros::NodeHandle node;
  string file = "/home/mcieplak/catkin_ws/src/BeerBot/template/water_bottle.png";
  struct stat buf;
  int statResult = stat(file.c_str(),&buf);
  if (statResult || buf.st_ino < 0) {
    cout << "File not found: " << file << endl;
    exit(-2);
  }
  myTemplate = imread(file, CV_LOAD_IMAGE_COLOR);
  if( !myTemplate.data)
  {
    cout << "Could not open or find the image" << endl;
    return -1;
  }

  //prev = 0;
  image_transport::ImageTransport it(node);
  image_transport::Subscriber mySub = it.subscribe("/camera/visible/image",
                                                    1, match);
  image_pub = it.advertise("/raw_image", 1);
  ros::spin();

  return 0;
}

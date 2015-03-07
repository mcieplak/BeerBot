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


Point minmax( Mat &result )
{
  double minVal, maxVal;
  Point  minLoc, maxLoc, matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  matchLoc = minLoc;

  return matchLoc;
}

Mat templateMatch( Mat &img, Mat &mytemplate )
{
  Mat result;

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
		match = minmax( current );
		rectangle( cv_ptr->image, match, Point( match.x + myTemplate.cols, 
								match.y + myTemplate.rows ), CV_RGB(255, 255, 255), 0.5 );
		ROI = cv::Rect( match.x, match.y, myTemplate.cols, myTemplate.rows );
		cv_ptr->image(ROI);
  }
  prev = current;
  image_pub.publish(cv_ptr->toImageMsg());

}

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "template_matching_node");
  ros::NodeHandle node;
  string file = "/home/viki/catkin_ws/src/BeerBot/template/ruination_ipa.jpg";
  struct stat buf;
  int statResult = stat(file.c_str(),&buf);
	cout << "Fuck me!" << endl;
  if (statResult || buf.st_ino < 0) {
    cout << "File not found: " << file << endl;
    exit(-2);
  }
	cout << "Fuck me!" << endl;
  myTemplate = imread(file, CV_LOAD_IMAGE_COLOR);
  if( !myTemplate.data)
  {
    cout << "Could not open or find the image" << endl;
    return -1;
  }
	cout << "Fuck me!" << endl;

  //prev = 0;
  image_transport::ImageTransport it(node);
  image_transport::Subscriber mySub = it.subscribe("/camera/visible/image",
                                                    1, match);
  image_pub = it.advertise("/raw_image", 1);
  ros::spin();

  return 0;
}

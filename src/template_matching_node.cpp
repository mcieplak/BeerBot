#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <stdio.h>
#include <sys/stat.h>

using namespace std;
using namespace cv;

image_transport::Publisher image_pub;
Mat prev, current, next;
Mat myTemplate;
Mat img_previous;
int option;
string file;

void draw_square( const Mat& current, Mat& result) {
//-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 1500;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_1, keypoints_2;

  detector.detect( current, keypoints_1 );
  detector.detect( result, keypoints_2 );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_1, descriptors_2;

  extractor.compute( current, keypoints_1, descriptors_1 );
  extractor.compute( result, keypoints_2, descriptors_2 );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_1, descriptors_2, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
  //-- small)
  //-- PS.- radiusMatch can also be used here.
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_1.rows; i++ )
  { if( matches[i].distance <= max(2*min_dist, 0.02) )
    { good_matches.push_back( matches[i]); }
  }

  //-- Draw only "good" matches
 cout << good_matches.size() << endl;

  if( good_matches.size() < 20 ) {
	  Mat img_matches;
	  drawMatches( current, keypoints_1, result, keypoints_2,
			  good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
			  vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	  img_previous = img_matches;
	  //-- Show detected matches
	  imshow( "Good Matches", img_matches );

	  for( int i = 0; i < (int)good_matches.size(); i++ )
	  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
  } else if( img_previous.data ) {
	  imshow("Good Matches", img_previous);
  }
  else
	  imshow("Good Matches", current );
  cv::waitKey(30);
  
}

void template_match( const sensor_msgs::Image::ConstPtr & msg ) {
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
    draw_square(myTemplate, cv_ptr->image);
  }
  prev = current;
  image_pub.publish(cv_ptr->toImageMsg());

}

/*
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
*/

void respondToRequest( const std_msgs::String::ConstPtr & msg ) {
  if( msg->data.c_str()[0] == '0' ) {
    option = 0;
    //file = "/home/jesus/src/beerbot/template/ruination_ipa.jpg";
    file = "/home/jesus/catkin_ws/src/beerbot/template/monster.jpg";
  }
  if( msg->data.c_str()[0] == '1' ) {
    option = 1;
    file = "/home/jesus/catkin_ws/src/beerbot/template/water_bottle.png";
    //file = "/home/jesus/catkin_ws/src/beerbot/template/monster.jpg";
  }
  if( msg->data.c_str()[0] == '2' ) {
    option = 2;
   // file = "/home/jesus/catkin_ws/src/beerbot/template/stone_delicious2.jpg";
  file = "/home/jesus/catkin_ws/src/beerbot/template/coke.jpg";
  }
  myTemplate = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
  if( !myTemplate.data)
  {
    cout << "Could not open or find the image" << endl;
  }
}

int main( int argc, char ** argv ) {
  ros::init(argc, argv, "template_matching_node");
  ros::NodeHandle node;

  option = 0;
  ros::Subscriber sub = node.subscribe("/chatter",1000, respondToRequest);
  file = "/home/jesus/catkin_ws/src/beerbot/template/coke.jpg";
  struct stat buf;
  int statResult = stat(file.c_str(),&buf);
  if (statResult || buf.st_ino < 0) {
    cout << "File not found: " << file << endl;
    exit(-2);
  }
  myTemplate = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
  if( !myTemplate.data)
  {
    cout << "Could not open or find the image" << endl;
    return -1;
  }

  image_transport::ImageTransport it(node);
  image_transport::Subscriber mySub = it.subscribe("/camera/visible/image1",
                                                    1, template_match);
  image_pub = it.advertise("/raw_image", 1);
  ros::spin();

  return 0;
}

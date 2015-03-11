#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int64.h>

using namespace std;
using namespace cv;

ros::Publisher y;
CascadeClassifier face_cascade;
bool buttonClicked;

void buttonAction( const std_msgs::String::ConstPtr & msg ) {
  if( msg->data.c_str()[0] == '1' ) {
    buttonClicked = true;
    cout << "Searching for user...";
    cout.flush();
  }
}

void getY (int64 num) {
    std_msgs::String msg;
    std::stringstream ss;
    long int y_coord = (long int) num;
    // ROS_INFO("y coord is: %d", (int) y_coord);  
    ss << y_coord << endl;
    msg.data = ss.str();
    y.publish(msg);
    buttonClicked = false;
    cout << " user found!" << endl;
}

void detectAndDisplay( Mat &frame )
{
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    if(!faces.empty())
      getY(faces[0].y + faces[0].height);
    for( size_t i = 0; i < faces.size(); i++ )
    {
      Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
      ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& source)
{
    if( !buttonClicked )
      return;
    cv_bridge::CvImagePtr aa = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::MONO8);

    static cv_bridge::CvImagePtr prev = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::MONO8);
    cv_bridge::CvImagePtr out = cv_bridge::toCvCopy(source, sensor_msgs::image_encodings::RGB8);

    detectAndDisplay(out->image);
}

int main( int argc, char** argv )
{
    buttonClicked = false;

    String face_cascade_name = "/home/jesus/catkin_ws/src/facedetect/src/haarcascade_frontalface_alt.xml";
    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
    ros::init(argc, argv, "facedetect");
    ros::NodeHandle node;
    y = node.advertise<std_msgs::String>("/face_detect", 1);
    ros::Subscriber button_sub = node.subscribe("/button_node",1000, buttonAction);

    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/camera/visible/image", 1, imageCallback);
    ros::spin();
    return 0;
}


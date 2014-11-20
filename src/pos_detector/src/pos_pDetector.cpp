
#include "ros/ros.h"


#include <math.h>
#include <sstream>

#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>	// Sychronized Message Handling
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <compressed_depth_image_transport/compression_common.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"


//using namespace cv;
//using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> ApproximateTimePolicy;

ros::NodeHandle* hRosNode;
message_filters::Subscriber<sensor_msgs::CompressedImage> 	*hRosSubRGBVid1, *hRosSubRGBVid2, *hRosSubDepthVid;
message_filters::Synchronizer<ApproximateTimePolicy> *rosVideoSync;

image_transport::Publisher image_pub1, image_pub2;


	void syncVideoCallback(const sensor_msgs::CompressedImageConstPtr& rgbImg1, const sensor_msgs::CompressedImageConstPtr& rgbImg2) {

	cv::Mat cv_rgb1, cv_rgb2;

	// load the images:
	cv::Mat tmp_rgb1 = cv::imdecode(cv::Mat(rgbImg1->data), CV_LOAD_IMAGE_UNCHANGED);
	cv::Mat tmp_rgb2 = cv::imdecode(cv::Mat(rgbImg2->data), CV_LOAD_IMAGE_UNCHANGED);

	tmp_rgb1.convertTo(cv_rgb1, CV_8UC3);
	tmp_rgb2.convertTo(cv_rgb2, CV_8UC3);
	
	// process images, by rearranging the color values
	cv::cvtColor(cv_rgb1,cv_rgb1, CV_BGR2RGB);
	cv::cvtColor(cv_rgb2,cv_rgb2, CV_BGR2RGB);
	
	// GRAY-SCALE
	cv::Mat cv_gray1, cv_gray2;
	cvtColor(cv_rgb1, cv_gray1, CV_RGB2GRAY);
	cvtColor(cv_rgb2, cv_gray2, CV_RGB2GRAY);
	



	//--- FIND CHESSBOARD CORNERS -- http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findchessboardcorners

	cv::Size patternsize(8,6); //interior number of corners
	cv::Mat left = cv_gray1.clone(); //source image
	cv::Mat right = cv_gray2.clone();
	cv::vector<cv::Point2f> corners_l, corners_r; //this will be filled by the detected corners

	//CALIB_CB_FAST_CHECK saves a lot of time on images
	//that do not contain any chessboard corners
	bool patternfound_l = cv::findChessboardCorners(left, patternsize, corners_l,
        			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
        			+ cv::CALIB_CB_FAST_CHECK);
	bool patternfound_r = cv::findChessboardCorners(right, patternsize, corners_r,
        			cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
        			+ cv::CALIB_CB_FAST_CHECK);

	if(patternfound_l)
  		cornerSubPix(left, corners_l, cv::Size(11, 11), cv::Size(-1, -1),
    				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	if(patternfound_r)
  		cornerSubPix(right, corners_r, cv::Size(11, 11), cv::Size(-1, -1),
    				cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	cv::drawChessboardCorners(left, patternsize, cv::Mat(corners_l), patternfound_l);
	cv::drawChessboardCorners(right, patternsize, cv::Mat(corners_r), patternfound_r);
	// ------------------------------
	
	// Mean points in img 1 and 2 where the chessboard is found 
	cv::Point2f meanP_l, meanP_r;
	if ( patternfound_l && patternfound_r) {
		
		cv::Mat points_l, points_r;
		cv::reduce( corners_l, points_l, CV_REDUCE_AVG, 1);
		cv::reduce( corners_r, points_r, CV_REDUCE_AVG, 1);

		cv::Point2f mean_l(points_l.at<float>(0,0), points_l.at<float>(0,1)); 
		cv::Point2f mean_r(points_r.at<float>(0,0), points_r.at<float>(0,1)); 
		
		meanP_l = mean_l;
		meanP_r = mean_r;
		//std::cout << "mean_l: " << mean_l.x << ", " << mean_l.y << std::endl;
		//std::cout << "mean_r: " << mean_r.x << ", " << mean_r.y << std::endl;
	}


	
	
	// Converts cv::Mat into CvImage
	cv_bridge::CvImage out_msg1, out_msg2;
	out_msg1.header   = rgbImg1->header; 
	out_msg1.encoding = sensor_msgs::image_encodings::MONO8; 
	out_msg1.image    = left; 

	out_msg2.header   = rgbImg2->header; 
	out_msg2.encoding = sensor_msgs::image_encodings::MONO8; 
	out_msg2.image    = right;

	// Publish images
	image_pub1.publish( out_msg1.toImageMsg());	
	image_pub2.publish( out_msg2.toImageMsg());
} 



/**
 * Main method for initialice the ROS node
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ubotcontrol");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

	hRosNode = new ros::NodeHandle();

	image_transport::ImageTransport it(n);

  /**
   * 	Subscribes to the rgb and depth compressed channel of the camera
   */

  hRosSubRGBVid1 = 
			new message_filters::Subscriber<sensor_msgs::CompressedImage>
				(*hRosNode, "/camera1/rgb/image/compressed", 1);
  hRosSubRGBVid2 = 
			new message_filters::Subscriber<sensor_msgs::CompressedImage>
				(*hRosNode, "/camera2/rgb/image/compressed", 1);

  rosVideoSync = 
			new message_filters::Synchronizer<ApproximateTimePolicy>
				(ApproximateTimePolicy(15), *hRosSubRGBVid1, *hRosSubRGBVid2);

  rosVideoSync->registerCallback(boost::bind(&syncVideoCallback, _1, _2));

  
  image_pub1 = it.advertise("/left", 1);
  image_pub2 = it.advertise("/right", 1);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}




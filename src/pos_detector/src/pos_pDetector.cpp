
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
	



	// -------- FIND CHESSBOARD CORNERS --------
	// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findchessboardcorners

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
	// ----------------------------------------
	

	// Camera matrix parameters from the stereo calibration
	cv::Mat cameraMatrix1 	= (cv::Mat_<double>(3,3) << 	533.2588540406424, 0.0, 305.4269987380971, 0.0, 533.5267980704926, 234.49660553682335, 0.0, 0.0, 1.0);
	cv::Mat cameraMatrix2 	= (cv::Mat_<double>(3,3) << 	530.6046944518044, 0.0, 301.1964094473342, 0.0, 530.9279419363547, 261.0150759049696, 0.0, 0.0, 1.0);
	cv::Mat distCoeffs1 	= (cv::Mat_<double>(1,5) << -0.05549635710799639, -0.04510391780805645, 0.0003586192451753303, -0.0048501238838964044, 0.0);
	cv::Mat distCoeffs2 	= (cv::Mat_<double>(1,5) << -0.09846369782733233, 0.014397165901325767, 0.006069571494486711, 0.0023531755572130646, 0.0);
	cv::Mat T 		= (cv::Mat_<double>(3,1) << 0.01667109250238205, 7.877564019959262e-05, -0.0004156408615162623);
	cv::Mat R 		= (cv::Mat_<double>(3,3) << 0.9999794072254488, 0.006414151426626488, 0.0002092522795325808, -0.006417520130024028, 0.9993206169607065, 0.036292147258789126, 2.3673211032360872e-05, -0.036292742783498796, 0.9993412011224344);
	
	// Parameters for stereoRectify & triangulatePoints 
	// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
	cv::Mat projMatr1, projMatr2, R1, R2, Q, points4D, point4D;
	
	// Projection matrix of both cameras
	cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, 
					cv_rgb1.size(), R, T, R1, R2, projMatr1, projMatr2, Q, 
					cv::CALIB_ZERO_DISPARITY, -1, cv::Size(), 0, 0 );
	
	// Transform chessboard corners to Matrix
	cv::Mat points_l = cv::Mat(2, corners_l.size(), CV_32F);
	cv::Mat points_r = cv::Mat(2, corners_r.size(), CV_32F);
  	for( int i = 0; i< corners_l.size(); i++ ){
    		points_l.at<float>(0,i) = corners_l[i].x;
		points_l.at<float>(1,i) = corners_l[i].y;	}
	for( int i = 0; i< corners_r.size(); i++ ){
    		points_r.at<float>(0,i) = corners_r[i].x;
		points_r.at<float>(1,i) = corners_r[i].y;	}
	
	/*// Get the mean  -----    GETTING THE MEAN BEFORE TRIANGULATION -----
	cv::Mat mean_cL,mean_cR;
	cv::reduce(corners_l, mean_cL, CV_REDUCE_AVG, 1);
	cv::reduce(corners_r, mean_cR, CV_REDUCE_AVG, 1); */// ------------------


	///	DEBUG
	#if 0	
		std::cout << "points_l: " 	<< std::endl << points_l 	<< std::endl 
			  << "Size points_l: "	<< std::endl << points_l.size()	<< std::endl;

		std::cout << "points_r: " 	<< std::endl << points_r 	<< std::endl 
			  << "Size points_r: "	<< std::endl << points_r.size()	<< std::endl;
	#endif

	// In case we have found the marker in both cameras
	if ( patternfound_l && patternfound_r) {
		
		// We get the array of points in homogeneus coordinates
		cv::triangulatePoints(projMatr1, projMatr2, 
					points_l, points_r, points4D);

		/*   -----    GETTING THE MEAN BEFORE TRIANGULATION -----------------
			cv::triangulatePoints(projMatr1, projMatr2, 
					mean_cL, mean_cR, point4D);*/// ------------------

		// We get the mean of every point in order to get the center of the mark
		cv::Mat mean_Hom;
		cv::reduce(points4D, mean_Hom, CV_REDUCE_AVG, 1);
	
		///	DEBUG
		#if 1

		std::cout << "Points4D: " 	<< std::endl << points4D 	<< std::endl 
			  << "Size Points: "	<< std::endl << points4D.size()	<< std::endl;

		std::cout << "Mean Position: " 	<< std::endl << mean_Hom  	<< std::endl
			  << "Size meanP: "	<< std::endl << mean_Hom.size()	<< std::endl;

		#endif
	}

	/*
	// Mean points in img 1 and 2 where the chessboard is found 
		cv::Point2f mean_l(points_l.at<float>(0,0), points_l.at<float>(0,1)); 
		cv::Point2f mean_r(points_r.at<float>(0,0), points_r.at<float>(0,1)); 
	*/


	
	
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




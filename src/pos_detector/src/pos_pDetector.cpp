
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

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

// RGB Subscribers of both cameras
message_filters::Subscriber<sensor_msgs::CompressedImage> 	*hRosSubRGBVid1, *hRosSubRGBVid2, *hRosSubDepthVid;

// Synchronizer
message_filters::Synchronizer<ApproximateTimePolicy> *rosVideoSync;

// RGB Publishers of both cameras after modifications
image_transport::Publisher image_pub1, image_pub2;

// STEREO CAMERA PARAMS
// Camera matrix parameters from the stereo calibration
cv::Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, T, R;
// Parameters for stereoRectify & triangulatePoints 
// http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
cv::Mat projMatr1, projMatr2, R1, R2, Q, points4D, point4D;

// For publishing the position of both cameras and the marker
tf::Transform transformCam, transformMark;
bool markerDet(false);

// NUMBER COL and NUM ROW of the MARKER
//int N_COL_M(8), N_ROW_M(6);
int N_COL_M(4), N_ROW_M(3);

	/** 
	  * Set up the camera parameters
	  * 
	  * Get the projection matrix of both cameras from
	  * the matrix parameters of the stereo calibration
	  * 
	  */
	void getCameraParameters() {

	// Camera matrix parameters from the stereo calibration
	cameraMatrix1 	= (cv::Mat_<double>(3,3) << 	537.8281251735632, 0.0, 323.0828891918958, 0.0, 537.8757320685007, 237.48090952888322, 0.0, 0.0, 1.0);
	cameraMatrix2 	= (cv::Mat_<double>(3,3) << 	536.9427015766673, 0.0, 311.3358857550221, 0.0, 537.1131837370293, 238.4096524118303, 0.0, 0.0, 1.0);
	distCoeffs1 	= (cv::Mat_<double>(1,5) << 	0.03625205913192116, -0.11229919070279479, 0.0004558456165320926, 0.0032326063446587974, 0.0);
	distCoeffs2 	= (cv::Mat_<double>(1,5) << 	0.03392840932173053, -0.10046007445798867, 0.0009116648814868233, -0.00041073496623280194, 0.0);
	T 		= (cv::Mat_<double>(3,1) << 	0.5425115119982284, 0.003691868475910774, 0.00805608905575923);
	R 		= (cv::Mat_<double>(3,3) << 	0.9802375726717746, 0.04092563235839786, -0.19354429399652393, -0.02865853640253347, 0.9974236508014525, 0.06576282470489558, 0.19573704149652485, -0.058916495465764246, 0.9788850071117753);

	// CAMERA RESOLUTION
	cv::Size camera_resolution = cv::Size(640, 480);

	// Projection matrix of both cameras
	cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, 
					camera_resolution, R, T, R1, R2, projMatr1, projMatr2, Q, 
					cv::CALIB_ZERO_DISPARITY, -1, cv::Size(), 0, 0 );

	// Creates tf transform for the camera
	tf::Vector3 origin;
  	origin.setValue(T.at<double>(0,0),T.at<double>(1,0),T.at<double>(2,0));

  	tf::Matrix3x3 tf3d;
  	tf3d.setValue(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), 
        		R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), 
        		R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

  	tf::Quaternion tfqt;
  	tf3d.getRotation(tfqt);

  	transformCam.setOrigin(origin);
  	transformCam.setRotation(tfqt);


	}

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

	cv::Size patternsize(N_COL_M,N_ROW_M); //interior number of corners
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

		///	DEBUG
		#if 1
		std::cout << "Points4D: " 	<< std::endl << points4D 	<< std::endl 
			  << "Size Points: "	<< std::endl << points4D.size()	<< std::endl;
		#endif

		// We divide the first 3 components by the 4th to get the actual coordinates
		//cv::convertPointsFromHomogeneous(points4D, points4D);
		cv::Mat lastRow = points4D.row( 3 );
		cv::Mat tmp;
		repeat(lastRow, 4, 1, tmp );  // create a tmp matrix of 4xN whose elements are repeated elements of the last row of points4D
		points4D = points4D / tmp;
		
		#if 0
		std::cout << "Points4D: " 	<< std::endl << points4D 	<< std::endl 
			  << "Size Points: "	<< std::endl << points4D.size()	<< std::endl;
		std::cout << "tmp: " 		<< std::endl << tmp 		<< std::endl;
		#endif

		/*   -----    GETTING THE MEAN BEFORE TRIANGULATION -----------------
			cv::triangulatePoints(projMatr1, projMatr2, 
					mean_cL, mean_cR, point4D);*/// ------------------

		// We get the mean of every point in order to get the center of the mark
		cv::Mat mean_Hom;
		cv::reduce(points4D, mean_Hom, CV_REDUCE_AVG, 1);
	
		///	DEBUG
		#if 1
		//std::cout << "Points4D: " 	<< std::endl << points4D 	<< std::endl 
		//	  << "Size Points: "	<< std::endl << points4D.size()	<< std::endl;
		std::cout << "Mean Position: " 	<< std::endl << mean_Hom  	<< std::endl
			  << "Size meanP: "	<< std::endl << mean_Hom.size()	<< std::endl;
		//std::cout << "Point 1: " 	<< std::endl << points4D.at<float>(0,1) << " " 
		//				<< points4D.at<float>(1,1) << " " << points4D.at<float>(2,1) << std::endl;
		#endif

		// We get the angle of the marker by getting the mean vector between a point and the next point
		// in the same row.
		tf::Vector3 markDirection;

		// We create a matrix to store every (horizontal) direction between two points in the marker
		cv::Mat markDir = cv::Mat(3, (N_COL_M-1)*N_ROW_M , CV_32F);
		// index of the new Matrix
		int index = 0;
		for(int n_row=0; n_row < N_ROW_M; n_row++) {
			for(int n_col=0; n_col < N_COL_M-1; n_col++) {
				// position in the 4xN matrix of the origin point of the difference vector
				int index_origin = n_col + n_row*N_COL_M;
				// x coordinate diference
				markDir.at<float>(0,index) = points4D.at<float>(0,index_origin+1) - points4D.at<float>(0,index_origin);
				// y coordinate diference
				markDir.at<float>(1,index) = points4D.at<float>(1,index_origin+1) - points4D.at<float>(1,index_origin);
				// z coordinate diference
				markDir.at<float>(2,index) = points4D.at<float>(2,index_origin+1) - points4D.at<float>(2,index_origin);
				index++;
			}
		}
		// We compute the mean of the matrix to get the mean direction vector
		cv::Mat mark_Dir_mean;
		cv::reduce(markDir, mark_Dir_mean, CV_REDUCE_AVG, 1);
		markDirection.setValue(mark_Dir_mean.at<float>(0,0),mark_Dir_mean.at<float>(0,1),mark_Dir_mean.at<float>(0,2));
		
		/// DEBUG
		#if 0
		std::cout << "Points4D: z - coordinate" 	<< std::endl << points4D.row(2) 	<< std::endl;
		std::cout << "markDir: " 	<< std::endl << markDir 	<< std::endl;
		std::cout << "mark_Dir_Vector: " 	<< mark_Dir_mean 	<< std::endl;
		std::cout << "markDirection: " 		<< mark_Dir_mean.at<float>(0,0) 	<< " " 
						<< mark_Dir_mean.at<float>(0,1) 		<< " " 
						<< mark_Dir_mean.at<float>(0,2) 		<< std::endl;
		#endif

		// Creates tf transform for the marker
		// POSITION
		tf::Vector3 origin;
  		origin.setValue(mean_Hom.at<float>(0,0),mean_Hom.at<float>(0,1),mean_Hom.at<float>(0,2));
		transformMark.setOrigin(origin);
		
		// ROTATION
		tf::Vector3 cameraDirection;
		cameraDirection.setValue(0,0,1);

		tf::Vector3 right_markD_vector = markDirection.cross(cameraDirection);
		right_markD_vector.normalized();
		double theta = markDirection.dot(cameraDirection);
		double angle_rotation = -1.0*acos(theta);

		/// DEBUG
		#if 0
		std::cout << "theta" 		<< theta 	<< std::endl;
		std::cout << "angle_rotation: " 	<< angle_rotation 	<< std::endl;
		std::cout << "mark_Dir_Vector: " 	<< mark_Dir_mean 	<< std::endl;
		std::cout << "right_markD_vector: " 		<< right_markD_vector.x() 	<< " " 
						<< right_markD_vector.y() 		<< " " 
						<< right_markD_vector.z() 		<< std::endl;
		#endif

		// Create quaternion
		tf::Quaternion tfqt(right_markD_vector, angle_rotation);

  		transformMark.setRotation(tfqt);
		
		// MARKER DETECTED
		markerDet = true;

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

	static tf::TransformBroadcaster br;

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

  // GET CAMERA PARAMETERS
  getCameraParameters();

  // PUBLISHERS
  image_pub1 = it.advertise("/left", 1);
  image_pub2 = it.advertise("/right", 1);

  ros::Rate loop_rate(25); // Number in hz

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    
    ros::spinOnce();

    // Send position of cameras
    br.sendTransform(tf::StampedTransform(transformCam, ros::Time::now(), "camera_left", "camera_right"));
    if(markerDet) {
		markerDet = false;
		//br.sendTransform(tf::StampedTransform(transformMark, ros::Time::now(), "camera_left", "marker"));
    }

    loop_rate.sleep();
    ++count;
  }


  return 0;
}




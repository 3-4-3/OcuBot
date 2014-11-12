
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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> ApproximateTimePolicy;

ros::NodeHandle* hRosNode;
message_filters::Subscriber<sensor_msgs::CompressedImage> 	*hRosSubRGBVid, *hRosSubDepthVid;
message_filters::Synchronizer<ApproximateTimePolicy> *rosVideoSync;

image_transport::Publisher image_pub;

void syncVideoCallback(const sensor_msgs::CompressedImageConstPtr& depthImg, const sensor_msgs::CompressedImageConstPtr& rgbImg) {

	//------ std::cout << "GETTING VIDEO SIGNAL" << std::endl;  -----// checked
	cv::Mat cv_rgb, cv_depth;

	// We have to cut away the compression header to load the depth image into openCV
	compressed_depth_image_transport::ConfigHeader compressionConfig;
	memcpy(&compressionConfig, &depthImg->data[0], sizeof(compressionConfig));
	const std::vector<uint8_t> depthData(depthImg->data.begin() + sizeof(compressionConfig), depthImg->data.end());
	
	// load the images:
	cv::Mat tmp_depth = cv::imdecode(cv::Mat(depthData), CV_LOAD_IMAGE_UNCHANGED);
	cv::Mat tmp_rgb = cv::imdecode(cv::Mat(rgbImg->data), CV_LOAD_IMAGE_UNCHANGED);
	tmp_depth.convertTo(cv_depth, CV_16U);
	tmp_rgb.convertTo(cv_rgb, CV_8UC3);
	
	// process images, by bluring the depth and rearranging the color values
	cv::GaussianBlur(cv_depth, cv_depth, cv::Size(11,11), 0, 0);
	cv::cvtColor(cv_rgb,cv_rgb, CV_BGR2RGB);


	// adding circle

	cv::circle(cv_rgb, cv::Point(50, 50), 10, CV_RGB(255,255,255));

	// Converts cv::Mat into CvImage
	cv_bridge::CvImage out_msg;
	out_msg.header   = rgbImg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
	out_msg.image    = cv_rgb; // Your cv::Mat


	// Publish image
	image_pub.publish( out_msg.toImageMsg());
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

  hRosSubRGBVid = 
			new message_filters::Subscriber<sensor_msgs::CompressedImage>
				(*hRosNode, "/camera1/rgb/image/compressed", 1);
  hRosSubDepthVid = 
			new message_filters::Subscriber<sensor_msgs::CompressedImage>
				(*hRosNode, "/camera1/depth/image_raw/compressed", 1);

  rosVideoSync = 
			new message_filters::Synchronizer<ApproximateTimePolicy>
				(ApproximateTimePolicy(15), *hRosSubDepthVid, *hRosSubRGBVid);

  rosVideoSync->registerCallback(boost::bind(&syncVideoCallback, _1, _2));

  
  image_pub = it.advertise("/image_converter/output_video", 1);

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




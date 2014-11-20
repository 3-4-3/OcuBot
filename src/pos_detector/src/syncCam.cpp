
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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximateTimePolicy;

ros::NodeHandle* hRosNode;
message_filters::Subscriber<sensor_msgs::Image> 	*hRosSubRGBVid1, *hRosSubRGBVid2;
message_filters::Synchronizer<ApproximateTimePolicy> *rosVideoSync;

image_transport::Publisher image_pub1, image_pub2;



void syncVideoCallback(const sensor_msgs::ImageConstPtr& rgbImg1, const sensor_msgs::ImageConstPtr& rgbImg2) {
	sensor_msgs::Image newImg1;
	
	
	newImg1.header 		= rgbImg1->header;
	newImg1.header.stamp 	= rgbImg2->header.stamp;
	newImg1.height 		= rgbImg1->height;
	newImg1.width 		= rgbImg1->width;
	newImg1.encoding 	= rgbImg1->encoding;
	newImg1.is_bigendian 	= rgbImg1->is_bigendian;
	newImg1.step 		= rgbImg1->step;
	newImg1.data  		= rgbImg1->data;


	image_pub1.publish( newImg1);
	image_pub2.publish( rgbImg2);
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
			new message_filters::Subscriber<sensor_msgs::Image>
				(*hRosNode, "/camera1/rgb/image_raw/image", 1);
  hRosSubRGBVid2 = 
			new message_filters::Subscriber<sensor_msgs::Image>
				(*hRosNode, "/camera2/rgb/image_raw/image", 1);

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



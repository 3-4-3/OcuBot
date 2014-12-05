/** 
 * 	
 *	Code used for publising a map frame and get the mean position of the global one
 *	
 *	The map frame has a different angle of the global frame (big marker)
 *	and same position to make it easier to create the ogre scene.
 */
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>	

#include <sstream>

# define M_PI           3.14159265358979323846  /* pi */
# define NUMBER_TF       20 /* Number of times that we get a tf to get the mean value */  

/**
 * Main method for initialice the ROS node
 */
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "ubotcontrol");

  	ros::NodeHandle n;

	static tf::TransformBroadcaster br;
	
	// For publishing the position of the map frame
        tf::Transform transformMap;
	// For publishing the position of the mean frame
        tf::Transform meanGlobal;

  	ros::Rate loop_rate(25); // Number in hz

	// MAP FRAME
	tf::Quaternion tfqt;
	double yaw(0.0), pitch(-M_PI / 2.0) ,roll(0.0);
	tfqt.setEuler 	( yaw, pitch, roll ) ;	
	transformMap.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transformMap.setRotation(tfqt);


	// MEAN_GLOBAL FRAME
	tf::TransformListener tfListener;
	static tf::StampedTransform vdTransform;
	tf::Vector3 meanPosition(0.0, 0.0, 0.0);

  int count = 0;
  std::cout << "Getting the position of the 1st camera to the global marker" << std::endl;
  while (ros::ok())
  {
    
    ros::spinOnce();


    if(count < NUMBER_TF) {


	try {
		if(count == 0) std::cout << "-- Waiting for the tf data --" << std::endl;

		ros::Time now = ros::Time::now();
    		tfListener.waitForTransform("global", "camera_left", now, ros::Duration(1.0) );
    		tfListener.lookupTransform("global", "camera_left", now, vdTransform);
		if(count == 0) std::cout << "-- Receiving tf data --" << std::endl;
		count++;


		meanPosition = meanPosition + vdTransform.getOrigin();

		if(!(count < NUMBER_TF)) {

			meanPosition = meanPosition / NUMBER_TF;



			meanGlobal.setOrigin( meanPosition );
			meanGlobal.setRotation( vdTransform.getRotation() ); // Get quaternion of last message
 			std::cout << "-- Got global position --" << std::endl;

		}
	} catch (tf::TransformException ex) {
   	 	ROS_ERROR("%s",ex.what());
	}



    } else {

	 br.sendTransform(tf::StampedTransform(meanGlobal, ros::Time::now(), "mean_global", "camera_left"));

    }



    // Send position of cameras
    br.sendTransform(tf::StampedTransform(transformMap, ros::Time::now(), "mean_global", "map"));
    
    loop_rate.sleep();
  }


  return 0;
}




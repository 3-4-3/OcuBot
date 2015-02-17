/** 
 * 	
 *	Code used for publishing a proper tf map by getting the information of the
 *	marker pose published by ar_pose. 	
 * 	
 * 	The global marker will be the parent of every other frame included 	
 *	both cameras. While both cameras will be the parents of the rest of the markers.
 * 	 	
 * 	The objective is to get the most updated information of the position of the robot 	
 *	by using any of the markers / cameras that are avaiable. 	
 *
 *	The global marker position will be saved as its position and the camera positions 	
 * 	will be static. To reduce the error a mean of the position will be performed after 	
 * 	measuring it several times. 	(Frame Name: "mean_global")
 *
 *	Another map frame (Frame Name: "map") is created with the same position and
 *	a different angle to the previous "mean_global" frame. This will 
 *	help to create the ogre scene in a easier way.
 *
 *
 *	Author: Carlos Pérez
 *
 */
#include "marker_system.h"


/**
 * Main method for initialice the ROS node
 */
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "ubotcontrol");

  	ros::NodeHandle n;

	
	br = new tf::TransformBroadcaster();

  	ros::Rate loop_rate(25); // Number in hz

	// MAP FRAME	
	create_map_frame();

	// Subscribe to markers pose
	global_cam1	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
					("ar_pose/cam1/global", 1, boost::bind(&global_Callback, _1, 1)));
	global_cam2	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
					("ar_pose/cam2/global", 1, boost::bind(&global_Callback, _1, 2)));
	marker1_cam1	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
					("ar_pose/cam1/marker1", 1, boost::bind(&marker_Callback, _1, 1, 1)));
	marker1_cam2	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
					("ar_pose/cam2/marker1", 1, boost::bind(&marker_Callback, _1, 2, 1)));
	odom_sub	=	new ros::Subscriber(n.subscribe<nav_msgs::Odometry>
					("odom", 5, boost::bind(&odom_Callback, _1)));


	std::cout << "-- Waiting for the tf data --" << std::endl;

	// MEAN_GLOBAL FRAME  - erase -
	tf::TransformListener tfListener;
	static tf::StampedTransform vdTransform;
	

  std::cout << "Getting the position of the 1st camera to the global marker" << std::endl;
  while (ros::ok())
  {
    
    ros::spinOnce();

	// Publish the camera frames
    if(!(count_c1 < NUMBER_TF)) {
	 br->sendTransform(tf::StampedTransform(global_to_cam1.inverse(), ros::Time::now(), "mean_global", "camera_left"));
    }
    if(!(count_c2 < NUMBER_TF)) {
	 br->sendTransform(tf::StampedTransform(global_to_cam2.inverse(), ros::Time::now(), "mean_global", "camera_right"));
    }

	// Republish marker frame
	if(n_camera == 1 ) {
		br->sendTransform(tf::StampedTransform(cam_to_robot, ros::Time::now(), "camera_left", "marker"));
	} else if (n_camera == 2 ) {
		br->sendTransform(tf::StampedTransform(cam_to_robot, ros::Time::now(), "camera_right", "marker"));
	}



    // Send position map frame
    br->sendTransform(tf::StampedTransform(transformMap, ros::Time::now(), "mean_global", "map"));
    
    loop_rate.sleep();
  }


  return 0;
}


void create_map_frame(void) {
	tf::Quaternion tfqt,tfqt2; // QUITAR TFQT2
	//double yaw(0.0), pitch(-M_PI / 2.0) ,roll(0.0);
	double yaw(M_PI), pitch(M_PI / 2.0) ,roll(0.0);
	tfqt.setEuler 	( 0.0, pitch, 0.0 ) ;	
	tfqt2.setEuler 	( yaw, 0.0, 0.0 ) ;	
	transformMap.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transformMap.setRotation(tfqt*tfqt2);

}


void marker_Callback(const ar_pose::ARMarkerConstPtr& marker, int camera, int m_number) {


	// First marker detection
	if(n_camera == 0)
	n_camera = camera;
	// Only one camera will update the tf
	// In case that camera has been inactive for a while the other
	// will take over the control of updating the pose of the marker
	if(n_camera != camera) {
		n_times++;
		if(n_times > TIME_LIMIT) {
			if(n_camera == 1) n_camera = 2;
			else if(n_camera == 2) n_camera = 1;
			n_times = 0;
		} else {
			// We don't update the information in tf
			return; 
		}
	} else {
		n_times = 0;
	}

	tf::Vector3 position(marker->pose.pose.position.x, marker->pose.pose.position.y, marker->pose.pose.position.z);
	tf::Quaternion q(marker->pose.pose.orientation.x, marker->pose.pose.orientation.y,
					marker->pose.pose.orientation.z, marker->pose.pose.orientation.w);


	// Fix image frame
	// Sometimes ar_pose will place the marker behind the camera in its opposite position
	// This will put it in the correct place
	if(marker->pose.pose.position.z < 0) {
		
		tf::Vector3 position_im(-marker->pose.pose.position.x, -marker->pose.pose.position.y, -marker->pose.pose.position.z);

		tf::Quaternion q_im(marker->pose.pose.orientation.y, -marker->pose.pose.orientation.x, 
					marker->pose.pose.orientation.w, -marker->pose.pose.orientation.z);

		position = position_im;
		q = q_im;
		
		std::cout << "Fixing cam position - marker: " << m_number << " cam: " << camera << std::endl;
	}

	cam_to_robot.setOrigin( position );
	cam_to_robot.setRotation( q );

	if(camera == 1 && m_number == 1) {
		br->sendTransform(tf::StampedTransform(cam_to_robot, ros::Time::now(), "camera_left", "marker"));
	} else if (camera == 2 && m_number == 1) {
		br->sendTransform(tf::StampedTransform(cam_to_robot, ros::Time::now(), "camera_right", "marker"));
	}


	// Save the odom position in this instant
	prev_pos_x  	= 	odom_pos_x;
	prev_pos_y  	= 	odom_pos_y;
	prev_pos_th 	= 	odom_pos_th; 
}

void global_Callback(const ar_pose::ARMarkerConstPtr& marker, int camera) {
	
	
	tf::Vector3 position(marker->pose.pose.position.x, marker->pose.pose.position.y, marker->pose.pose.position.z);
	tf::Quaternion q(marker->pose.pose.orientation.x, marker->pose.pose.orientation.y,
					marker->pose.pose.orientation.z, marker->pose.pose.orientation.w);

	// Fix image frame
	// Sometimes ar_pose will place the marker behind the camera in its opposite position
	// This will put it in the correct place
	if(marker->pose.pose.position.z < 0) {
		tf::Vector3 position_im(-marker->pose.pose.position.x, -marker->pose.pose.position.y, -marker->pose.pose.position.z);

		tf::Quaternion q_im(marker->pose.pose.orientation.y, -marker->pose.pose.orientation.x, 
					marker->pose.pose.orientation.w, -marker->pose.pose.orientation.z);

		position = position_im;
		q = q_im;
		
		if(camera == 1 && count_c1 < NUMBER_TF)
		std::cout << "Fixing cam position - global cam: " << camera << std::endl;
		if(camera == 2 && count_c2 < NUMBER_TF)
		std::cout << "Fixing cam position - global cam: " << camera << std::endl;

	}
	

	// Camera 1
	if(camera == 1 && count_c1 < NUMBER_TF) {
		if(count_c1 == 0) std::cout << "-- Receiving tf data / Cam 1 --" << std::endl;
		count_c1++;

		meanP_to_cam1 = meanP_to_cam1 + position;

		if(!(count_c1 < NUMBER_TF)) {

			meanP_to_cam1 = meanP_to_cam1 / NUMBER_TF;
			global_to_cam1.setOrigin( meanP_to_cam1 );

			
			global_to_cam1.setRotation( q ); // Set quaternion of last message
 			std::cout << "-- Got global position / Cam 1 --" << std::endl;

		}
    	}

	// Camera 2
	if(camera == 2 && count_c2 < NUMBER_TF) {
		if(count_c2 == 0) std::cout << "-- Receiving tf data / Cam 2 --" << std::endl;
		count_c2++;

		meanP_to_cam2 = meanP_to_cam2 + position;

		if(!(count_c2 < NUMBER_TF)) {

			meanP_to_cam2 = meanP_to_cam2 / NUMBER_TF;
			global_to_cam2.setOrigin( meanP_to_cam2 );

			global_to_cam2.setRotation( q ); // Set quaternion of last message
 			std::cout << "-- Got global position / Cam 2 --" << std::endl;

		}
    	}

}



void odom_Callback(const nav_msgs::OdometryConstPtr& odometry) {
	
	// Save the odometry
	odom_pos_x 	= odometry->pose.pose.position.x;
	odom_pos_y 	= odometry->pose.pose.position.y; 
	odom_pos_th 	= tf::getYaw(odometry->pose.pose.orientation);

	//std::cout << "getYaw " << odom_pos_th << std::endl;

	// Difference between this odometry and the odometry when the marker was found
	double x_dif, y_dif, th_dif;
	// Real position of the robot compared to the last position of the marker
	double final_x, final_y;

	x_dif 	= odom_pos_x - prev_pos_x; 
	y_dif 	= odom_pos_y - prev_pos_y;
	th_dif 	= odom_pos_th - prev_pos_th;

	//std::cout << "odom_pos_th " 	<< odom_pos_th 	<< std::endl;
	//std::cout << "prev_pos_th " 	<< prev_pos_th 	<< std::endl;
	//std::cout << "th_dif " 		<< th_dif 	<< std::endl;

	// Transform the vector difference into polar coordinates
	double module, p_angle;
	module 	= sqrt(x_dif*x_dif + y_dif*y_dif);
	p_angle = atan2(y_dif, x_dif);

	// Positions compared to the previous marker pose
	final_x	= module * cos(p_angle-prev_pos_th);
	final_y	= module * sin(p_angle-prev_pos_th);
	
	tf::Vector3 position(final_x, final_y, 0);
	tf::Quaternion odom_quat = tf::createQuaternionFromYaw(th_dif);
	
	transform.setOrigin( position );
	transform.setRotation( odom_quat );

	// Send the transform of the pose based on the odometry
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker", "you_bot"));
}


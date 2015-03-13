
#include "app_msys.h"


int main(int argc, char **argv)
{	

  	ros::init(argc, argv, "ubotcontrol");

  	ros::NodeHandle n;

	
	br = new tf::TransformBroadcaster();

  	ros::Rate loop_rate(25); // Number in hz

	std::cout << "Starting to get the " << NUMBER_CP_MARK << " positions that the robot must go" << std::endl;


		
	int i = 0;
	std::cout << "Detecting position nº "  << i << " please press enter to continue" << std::endl;
	std::string s;
	std::getline(std::cin, s, '\n');
	
		cps_cam1	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
						("ar_pose/cam1/global", 1, boost::bind(&cp_Callback, _1, 1, i)));
		cps_cam2	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
						("ar_pose/cam2/global", 1, boost::bind(&cp_Callback, _1, 2, i)));

	while (ros::ok())
 	{
    
    		ros::spinOnce();

		if(cam_number_cp[i]!=0) {

			delete cps_cam1;
			delete cps_cam2;

			std::cout << "Position nº "  << i << " detected" << std::endl;

			i++;

			if(i >= NUMBER_CP_MARK) {
				std::cout << "All positions tracked" << std::endl;
				continue;
			}

			std::cout << "Detecting position nº "  << i << " please press enter to continue" << std::endl;
			std::getline(std::cin, s, '\n');

			cps_cam1	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
						("ar_pose/cam1/global", 1, boost::bind(&cp_Callback, _1, 1, i)));
			cps_cam2	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
						("ar_pose/cam2/global", 1, boost::bind(&cp_Callback, _1, 2, i)));


		}





		// Publish the check point frames
		for( int i = 0; i < NUMBER_CP_MARK; i++) {
			std::ostringstream num;	
			num <<  i;

			if(cam_number_cp[i] == 1) {
				br->sendTransform(tf::StampedTransform(cp_to_cam1[i], ros::Time::now(), "camera_left", "cp_"+num.str()));
			} else if(cam_number_cp[i] == 2) {
				br->sendTransform(tf::StampedTransform(cp_to_cam2[i], ros::Time::now(), "camera_right", "cp_"+num.str()));
			}
		}
	
  	}


}


void cp_Callback(const ar_pose::ARMarkerConstPtr& marker, int camera, int check_point) {

	std::cout << "Getting cp " << check_point << " with camera " << camera << std::endl;
	
	
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
		
		/*if(camera == 1 && count_c1 < NUMBER_TF)
		std::cout << "Fixing cam position - global cam: " << camera << std::endl;
		if(camera == 2 && count_c2 < NUMBER_TF)
		std::cout << "Fixing cam position - global cam: " << camera << std::endl;*/

	}
	

	// Camera 1
	if(camera == 1 && count_c1[check_point] < NUMBER_TF) {

		count_c1[check_point]++;
		meanP_to_cam1[check_point] = meanP_to_cam1[check_point] + position;

		if(!(count_c1[check_point] < NUMBER_TF)) {

			meanP_to_cam1[check_point] = meanP_to_cam1[check_point] / NUMBER_TF;

			cp_to_cam1[check_point] .setOrigin( meanP_to_cam1[check_point] );	
			cp_to_cam1[check_point] .setRotation( q ); // Set quaternion of last message

			if(cam_number_cp[check_point]==0) cam_number_cp[check_point] = 1;

		}
    	}

	// Camera 2
	if(camera == 2 && count_c2[check_point] < NUMBER_TF) {

		count_c2[check_point]++;
		meanP_to_cam2[check_point] = meanP_to_cam2[check_point] + position;

		if(!(count_c2[check_point] < NUMBER_TF)) {

			meanP_to_cam2[check_point] = meanP_to_cam2[check_point] / NUMBER_TF;

			cp_to_cam2[check_point] .setOrigin( meanP_to_cam2[check_point] );
			cp_to_cam2[check_point] .setRotation( q ); // Set quaternion of last message
 			
			if(cam_number_cp[check_point]==0) cam_number_cp[check_point] = 2;
		}
    	}

}




/***
int main(int argc, char **argv)
{	

  	ros::init(argc, argv, "ubotcontrol");

  	ros::NodeHandle n;

	
	br = new tf::TransformBroadcaster();

  	ros::Rate loop_rate(25); // Number in hz

	// Subscribe to check points pose
	for( int i = 0; i < NUMBER_CP_MARK; i++) {
		std::ostringstream num;	
		num <<  i;

		cps_cam1[i]	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
						("ar_pose/cam1/cp_"+num.str(), 1, boost::bind(&cp_Callback, _1, 1, i)));
		cps_cam2[i]	=	new ros::Subscriber(n.subscribe<ar_pose::ARMarker>
						("ar_pose/cam2/cp_"+num.str(), 1, boost::bind(&cp_Callback, _1, 2, i)));
	}
		


	while (ros::ok())
 	{
    
    		ros::spinOnce();


		// Publish the check point frames
		for( int i = 0; i < NUMBER_CP_MARK; i++) {
			std::ostringstream num;	
			num <<  i;

			if(cam_number_cp[i] == 1) {
				br->sendTransform(tf::StampedTransform(cp_to_cam1[i], ros::Time::now(), "camera_left", "cp_"+num.str()));
			} else if(cam_number_cp[i] == 2) {
				br->sendTransform(tf::StampedTransform(cp_to_cam2[i], ros::Time::now(), "camera_right", "cp_"+num.str()));
			}
		}
	
  	}


}


void cp_Callback(const ar_pose::ARMarkerConstPtr& marker, int camera, int check_point) {

	std::cout << "Getting cp " << check_point << " with camera " << camera << std::endl;
	
	
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
		
		//if(camera == 1 && count_c1 < NUMBER_TF)
		//std::cout << "Fixing cam position - global cam: " << camera << std::endl;
		//if(camera == 2 && count_c2 < NUMBER_TF)
		//std::cout << "Fixing cam position - global cam: " << camera << std::endl;

	}
	

	// Camera 1
	if(camera == 1 && count_c1[check_point] < NUMBER_TF) {

		count_c1[check_point]++;
		meanP_to_cam1[check_point] = meanP_to_cam1[check_point] + position;

		if(!(count_c1[check_point] < NUMBER_TF)) {

			meanP_to_cam1[check_point] = meanP_to_cam1[check_point] / NUMBER_TF;

			cp_to_cam1[check_point] .setOrigin( meanP_to_cam1[check_point] );	
			cp_to_cam1[check_point] .setRotation( q ); // Set quaternion of last message

			if(cam_number_cp[check_point]==0) cam_number_cp[check_point] = 1;

		}
    	}

	// Camera 2
	if(camera == 2 && count_c2[check_point] < NUMBER_TF) {

		count_c2[check_point]++;
		meanP_to_cam2[check_point] = meanP_to_cam2[check_point] + position;

		if(!(count_c2[check_point] < NUMBER_TF)) {

			meanP_to_cam2[check_point] = meanP_to_cam2[check_point] / NUMBER_TF;

			cp_to_cam2[check_point] .setOrigin( meanP_to_cam2[check_point] );
			cp_to_cam2[check_point] .setRotation( q ); // Set quaternion of last message
 			
			if(cam_number_cp[check_point]==0) cam_number_cp[check_point] = 2;
		}
    	}

}*/

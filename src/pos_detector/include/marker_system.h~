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
 * 	measuring it several times. 	
 *
 *	Author: Carlos Pérez
 *
 */

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>	
#include <sstream>
#include <ar_pose/ARMarker.h>		// In case this doesn't work, please check CMakeLists.text and
					// include the directory where ARMarker.h is.
					// This was done as ar_pose couldn't be added in the find_package
					// for some reason. (to do: fix this)

#include <boost/thread/thread.hpp>


# define M_PI           3.14159265358979323846  /* pi */
# define NUMBER_TF       20 /* Number of times that we get a tf to get the mean value */  



// For publishing the position of the map frame
tf::Transform 	transformMap,
// For publishing the position of the global frame
		global_to_cam1,
		global_to_cam2,
// For publishing the rest of frames
		transform;

static tf::TransformBroadcaster *br;

// Counters to check if we got the global marker position enough times
int count_c1 = 0, count_c2 = 0;

// Mean position of global frames
tf::Vector3 	meanP_to_cam1(0.0, 0.0, 0.0),
		meanP_to_cam2(0.0, 0.0, 0.0);


// Subscriber to the markers pose
ros::Subscriber		*global_cam1, 
			*global_cam2, 
			*marker1_cam1, 
			*marker1_cam2;


void create_map_frame(void);

void global_Callback(const ar_pose::ARMarkerConstPtr& marker, int camera);
void marker_Callback(const ar_pose::ARMarkerConstPtr& marker, int camera, int m_number);








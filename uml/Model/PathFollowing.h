/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _PATHFOLLOWING_H
#define _PATHFOLLOWING_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class PathFollowing: public SteeringBehavior {
public: 
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	void update(pose myPose);
	
	/**
	 * @param objectives
	 * @param id
	 * @param weight
	 */
	void PathFollowing(pose* objectives, unsigned int id, float weight);
	
	void ~PathFollowing();
private: 
	vector<pose*> target;
};

#endif //_PATHFOLLOWING_H
/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _INTERPOSE_H
#define _INTERPOSE_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class Interpose: public SteeringBehavior {
public: 
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	void update(pose myPose);
	
	/**
	 * @param ids
	 * @param id
	 * @param weight
	 */
	void Interpose(unsigned int* ids, unsigned int id, float weight);
	
	void ~Interpose();
private: 
	vector<unsigned int*> robotIdToInterpose;
};

#endif //_INTERPOSE_H
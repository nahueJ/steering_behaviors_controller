/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _SEEK_H
#define _SEEK_H

#include "SteeringBehavior.h"
 
#include <geometry_msgs/Pose.h>

class Seek: public SteeringBehavior {
public: 
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	void update(pose myPose);
	
	/**
	 * @param objective
	 * @param id
	 * @param weight
	 */
	void Seek(pose objective, unsigned int id, float weight);
	
	void ~Seek();
private: 
	pose target;
};

#endif //_SEEK_H
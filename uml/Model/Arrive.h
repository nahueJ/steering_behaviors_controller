/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _ARRIVE_H
#define _ARRIVE_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class Arrive: public SteeringBehavior {
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
	void Arrive(pose objective, unsigned int id, float weight);
	
	void ~Arrive();
private: 
	pose target;
};

#endif //_ARRIVE_H
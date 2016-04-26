/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _FLEE_H
#define _FLEE_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class Flee: public SteeringBehavior {
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
	void Flee(pose objective, unsigned int id, float weight);
	
	void ~Flee();
private: 
	pose target;
};

#endif //_FLEE_H
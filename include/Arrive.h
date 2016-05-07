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
	 * @param objective
	 * @param id
	 * @param weight
	 */
	Arrive(pose objective, unsigned int id, float weight);
	
	~Arrive();

	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	virtual void update() const;
	
private: 
	pose target;
};

#endif //_ARRIVE_H
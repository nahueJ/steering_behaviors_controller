/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _WALLAVOIDANCE_H
#define _WALLAVOIDANCE_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Twist.h>

class WallAvoidance: public SteeringBehavior {
public: 
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myTwist
	 */
	void update(twist myTwist);
	
	/**
	 * @param mySensor
	 * @param unasigned int id
	 * @param float weight
	 */
	void WallAvoidance(string mySensor, void unasigned int id, void float weight);
	
	void ~WallAvoidance();
private: 
	string sensor;
};

#endif //_WALLAVOIDANCE_H
/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _OBSTACLEAVOIDANCE_H
#define _OBSTACLEAVOIDANCE_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Twist.h>

class ObstacleAvoidance: public SteeringBehavior {
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
	void ObstacleAvoidance(string mySensor, void unasigned int id, void float weight);
	
	void ~ObstacleAvoidance();
private: 
	string sensor;
};

#endif //_OBSTACLEAVOIDANCE_H
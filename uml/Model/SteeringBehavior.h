/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _STEERINGBEHAVIOR_H
#define _STEERINGBEHAVIOR_H

#include "ros/ros.h"

 #include <geometry_msgs/Twist.h>
 
class SteeringBehavior {
public: 
	
	/**
	 * @param unasigned int id
	 * @param float weight
	 */
	void SteeringBehavior(void unasigned int id, void float weight);
	
	void ~SteeringBehavior();
	
	void update();
	
	void getDesiredTwist();
	
	/**
	 * modify weight value
	 * @param float w
	 */
	void setWeight(void float w);
	
	/**
	 * return weight variable value
	 */
	void getWeight();
private: 
	float weight;
	unsigned int robotId;
	twist desiredTwist;
};

#endif //_STEERINGBEHAVIOR_H
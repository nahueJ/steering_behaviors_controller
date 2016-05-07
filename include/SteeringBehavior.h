/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _STEERINGBEHAVIOR_H
#define _STEERINGBEHAVIOR_H

#include "ros/ros.h"
 #include <iostream>
using std::cout;
using std::cin;
using std::endl;


//Behavior List //YO PENSABA Q TIENEN QUE ESTAR EN LOS INCLUDES, PERO NO
//#include "Seek.h"
//#include "Arrive.h"
//#include "WallAvoidance.h"

#include <geometry_msgs/Twist.h>
 
class SteeringBehavior {
public: 
	
	/**
	 * @param unasigned int id
	 * @param float weight
	 */
	SteeringBehavior(unsigned int id);
	
	~SteeringBehavior();
	
	virtual void update() const = 0; // función virtual pura
	
	virtual geometry_msgs::Twist getDesiredTwist() const; // función virtual 
	
protected: 
	unsigned int robotId;
	geometry_msgs::Twist desiredTwist;
};

#endif //_STEERINGBEHAVIOR_H
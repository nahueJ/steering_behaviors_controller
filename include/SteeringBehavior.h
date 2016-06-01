/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _STEERINGBEHAVIOR_H
#define _STEERINGBEHAVIOR_H

#include "ros/ros.h"
#include <string>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;


//Behavior List //YO PENSABA Q TIENEN QUE ESTAR EN LOS INCLUDES, PERO NO
//#include "Seek.h"
//#include "Arrive.h"
//#include "WallAvoidance.h"

//#include <geometry_msgs/Twist.h>
 
class SteeringBehavior {
public: 
	
	/**
	 * @param unasigned int id
	 * @param float weight
	 */
	SteeringBehavior(unsigned int id, std::string pre);
	
	~SteeringBehavior();
	
	virtual void update() ; // función virtual pura
	
	virtual float getDesiredW(); 
	
	void setDesiredW(float z);

protected: 

	unsigned int robotId;

	std::string pretopicname;

	double desiredW;
};

#endif //_STEERINGBEHAVIOR_H
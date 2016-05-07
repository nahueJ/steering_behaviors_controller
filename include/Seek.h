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
	 * @param objective
	 * @param id
	 * @param weight
	 */
	Seek(pose objective, unsigned int id, float weight);
	
	~Seek();
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	virtual void update() const;
	
private: 
	pose target;
};

#endif //_SEEK_H
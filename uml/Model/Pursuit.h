/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _PURSUIT_H
#define _PURSUIT_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class Pursuit: public SteeringBehavior {
public: 
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	void update(pose myPose);
	
	/**
	 * @param idPursuit
	 * @param unasigned int id
	 * @param float weight
	 */
	void Pursuit(unsigned int idPursuit, void unasigned int id, void float weight);
	
	void ~Pursuit();
private: 
	unsigned int robotIdToPursuit;
};

#endif //_PURSUIT_H
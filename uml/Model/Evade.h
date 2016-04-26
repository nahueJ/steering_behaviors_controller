/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _EVADE_H
#define _EVADE_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class Evade: public SteeringBehavior {
public: 
	
	/**
	 * gets the last data and actualizes the desiredTwist
	 * @param myPose
	 */
	void update(pose myPose);
	
	/**
	 * @param idEvade
	 * @param unasigned int id
	 * @param float weight
	 */
	void Evade(unsigned int idEvade, void unasigned int id, void float weight);
	
	void ~Evade();
private: 
	unsigned int robotIdToEvade;
};

#endif //_EVADE_H
/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _OFFSETPURSUIT_H
#define _OFFSETPURSUIT_H

#include "SteeringBehavior.h"

#include <geometry_msgs/Pose.h>

class OffsetPursuit: public SteeringBehavior {
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
	void OffsetPursuit(unsigned int idPursuit, void unasigned int id, void float weight);
	
	void ~OffsetPursuit();
private: 
	unsigned int robotIdToPursuit;
};

#endif //_OFFSETPURSUIT_H
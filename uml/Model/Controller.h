/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class Controller {
public: 
	
	/**
	 * @param unsigned int id
	 */
	void Controller(void unsigned int id);
	
	void ~Controller();
	
	void update();
	
	/**
	 * @param weight
	 */
	void seekOn(float weight);
	
	void seekOff();
	
	/**
	 * @param weight
	 */
	void fleeOn(float weight);
	
	void fleeOff();
	
	/**
	 * @param weight
	 */
	void wanderOn(float weight);
	
	void wanderOff();
	
	/**
	 * @param weight
	 */
	void arriveOn(float weight);
	
	void arriveOff();
	
	/**
	 * @param weight
	 * @param sensor
	 */
	void obstacleAvoidanceOn(float weight, string sensor);
	
	void obstacleAvoidanceOff();
	
	/**
	 * @param weight
	 */
	void evadeOn(float weight);
	
	void evadeOff();
	
	/**
	 * @param weight
	 */
	void hideOn(float weight);
	
	void hideOff();
	
	/**
	 * @param weight
	 * @param sensor
	 */
	void wallAvoidanceOn(float weight, string sensor);
	
	void wallAvoidanceOff();
	
	/**
	 * @param weight
	 */
	void pathFollowingOn(float weight);
	
	void pathFollowingOff();
	
	/**
	 * @param weight
	 */
	void offsetPursuitOn(float weight);
	
	void offsetPursuitOff();
	
	/**
	 * @param weight
	 */
	void pursuitOn(float weight);
	
	void pursuitOff();
	
	/**
	 * @param weight
	 */
	void interposeOn(float weight);
	
	void interposeOff();
private: 
	unsigned int robotId;
	ros::NodeHandle* node;
	vector<SteeringBehavior*> behaviors;
	pose myPosition;
	twist myTwist;
};

#endif //_CONTROLLER_H
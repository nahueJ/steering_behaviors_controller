/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "ros/ros.h"

//#include "SteeringBehavior.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "ros/message.h"
#include <string>
#include <vector>

#include <iostream>
using std::cout;
using std::cin;
using std::endl;


class Controller {
public: 
	
	/**
	 * @param unsigned int id
	 */
	Controller(unsigned int id);

	//Controller(i,behaviors[i], FactoryPtr);
	
	~Controller();
	
	void update();
	
private: 
	//Id del robot del controlador
	unsigned int robotId;

	//Variables para publicar por un topic
	ros::NodeHandle* rosNode;
	ros::Publisher* ctrlPublisher;
	ros::Rate* rosRate;

	//std::vector<SteeringBehavior*> behaviors;
	geometry_msgs::Pose myPosition;
	geometry_msgs::Twist myTwist;
	
};

#endif //_CONTROLLER_H
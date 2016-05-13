/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "ros/ros.h"
#include "ros/message.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "SteeringBehavior.h"

//para test-->> pasa a factory
#include "WallAvoidance.h"
#include "Seek.h"

#include <string>
#include <vector>
#include <sstream>
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
	//ros::Subscriber* ctrlSubscriber;

	std::vector<SteeringBehavior*> behaviors;
	//para test
	SteeringBehavior* behavior1;
	SteeringBehavior* behavior2;

	//TODO AGREGAR VECTOR O ARREGLO CON LOS PESOS CORRESPONDIENTES

	geometry_msgs::Pose myPosition;
	geometry_msgs::Twist myTwist;
	
};

#endif //_CONTROLLER_H
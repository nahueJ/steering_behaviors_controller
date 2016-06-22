/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _AGENT_H
#define _AGENT_H

#include "ros/ros.h"
#include "ros/message.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "SteeringBehavior.h"

// //para test-->> pasa a factory
// #include "ObstacleAvoidance.h"
// #include "Seek.h"

#include "Factory.h"

#include <string>
#include <vector>
#include <sstream>
// #include <iostream>
// using std::cout;
// using std::cin;
// using std::endl;


class Agent {
public: 
	
	/**
	 * @param unsigned int id
	 */
	Agent(unsigned int id, Factory* FactoryPtr);
	
	~Agent();
	
	void update();

	int imAlone();

	float addAngle(float, float);

	float deltaAngle(float, float);

	float turningVel(float);

	float toScale(float);
	
private: 
	//Id del robot del controlador
	unsigned int robotId;

	//Variables para publicar por un topic
	ros::NodeHandle* rosNode;
	ros::Subscriber* odomSubscriber;
	ros::Publisher* ctrlPublisher;
	//ros::Subscriber* ctrlSubscriber;
	std::stringstream* pretopicname;
	
	std::vector<SteeringBehavior*> behaviors;
	//float* weights;
	float seekWeight;
	float ObsAvWeight;
	
	geometry_msgs::Pose myPosition;
	geometry_msgs::Twist myTwist;

	//Funciones de Callback para las suscripciones al odometro 
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

	nav_msgs::Odometry*	myData;
	
};

#endif //_AGENT_H
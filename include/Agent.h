/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _AGENT_H
#define _AGENT_H

#include "ros/ros.h"
#include "ros/message.h"
#include "tf/tf.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "SteeringBehavior.h"
#include "Weights.h"
#include "Factory.h"

#include <string>
#include <vector>
#include <sstream>

#include <iostream>
#include <fstream>
#include <errno.h>

#define PI 3.14159265

class Agent {
public:

	Agent(unsigned int id, Factory* factoryPtr);

	~Agent();

	int update();

	void setNewObjective(std::pair<float, float>);

private:
	//Id del robot del controlador
	unsigned int robotId;
	string myType;
	std::vector< float > state;
	std::vector< float > ansState;
	int roundCounter;

	//Variables para publicar por un topic
	ros::NodeHandle* rosNode;
	ros::Subscriber* odomSubscriber;
	ros::Publisher* ctrlPublisher;
	//ros::Subscriber* ctrlSubscriber;
	std::stringstream* pretopicname;

	std::vector<SteeringBehavior*> behaviors;
	std::vector<float> w;
	Weights* weights;
	int nbBehaviors;

	geometry_msgs::Pose target;

	geometry_msgs::Pose myPosition;
	geometry_msgs::Twist myTwist;

	//Funciones de Callback para las suscripciones al odometro
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

	//variables para almacenar los datos del odometro
	float x;
	float y;
	float tita;

	float addAngle(float, float);

	float deltaAngle(float, float);

	float turningVel(float);

	float toScale(float);

	void updateState();

	void printState();

	void restartRoutine();

};

#endif //_AGENT_H

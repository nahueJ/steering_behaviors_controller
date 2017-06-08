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
#include "Factory.h"

#include <string>
#include <vector>
#include <sstream>

#include <math.h>

#include <iostream>
#include <fstream>
#include <errno.h>

#define PI 3.14159265

class Agent {
public:

	Agent(unsigned int id, string type, Factory* factoryPtr);

	~Agent();

	int update();

	void setNewObjective(std::pair<float, float>);

protected:
	string myType;

private:
	//Id del robot del controlador
	unsigned int robotId;
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

	//Funciones de Callback para las suscripciones al odometro
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

	//variables para almacenar los datos del odometro
	float x;
	float y;
	float tita;

	// void updateState();
	//
	// void printState();

	virtual std::vector<float> getWeights(std::vector<float> estado);

};

#endif //_AGENT_H

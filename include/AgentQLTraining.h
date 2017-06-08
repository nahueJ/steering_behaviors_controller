/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _AGENTQLTRAINING_H
#define _AGENTQLTRAINING_H

#include "ros/ros.h"
#include "ros/message.h"
#include "tf/tf.h"

#include "Agent.h"

#include <math.h>
#include <sstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#define PI 3.14159265

#include <libconfig.h++>
using namespace libconfig;

class AgentQLTraining: public Agent {
public:

	AgentQLTraining(unsigned int id, string type, Factory* factoryPtr);

	~AgentQLTraining();

private:

	virtual std::vector<float> getWeights(std::vector<float>);

	std::vector<float> pesos;
};

#endif //_AGENTQLTRAINING_H

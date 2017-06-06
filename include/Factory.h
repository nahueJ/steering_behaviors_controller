/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _FACTORY_H
#define _FACTORY_H

#include "SteeringBehavior.h"
#include "ObstacleAvoidance.h"
#include "Seek.h"

#include <string>
#include <vector>
#include <sstream>

#include <libconfig.h++>
using namespace libconfig;

#define CONFIGFILE "./src/steering_behaviors_controller/simulation.cfg"

class Factory {

public:

	Factory();

	~Factory();

	int instanciateBehaviors(unsigned int id, std::string pre,
							std::vector<SteeringBehavior*>* behaviors,
							std::string* type);

	int getAgents();

	std::vector<float> getConstantWeights();

private:

	Config* cfg;
	int nbAgents;
	std::stringstream agentType;

	SteeringBehavior* pickBehavior(std::string behaviorName, int id, std::string pre);

	//Weights* pickWeights(std::string, Setting&, std::vector<SteeringBehavior*>, std::vector< std::vector<float> >*);

};

#endif //_FACTORY_H

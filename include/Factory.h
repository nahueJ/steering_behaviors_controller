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

	/**
	 * @param unsigned int id
	 */
	Factory();

	//Factory(i,behaviors[i], FactoryPtr);
	
	~Factory();

	int instanciateBehaviors(unsigned int id, std::string pre, std::vector<SteeringBehavior*> behaviors, std::vector<float> weights);

	int getAgents();

private: 

	int nbAgents;

	// Setting& agentsType;

	Config* cfg;
};

#endif //_FACTORY_H
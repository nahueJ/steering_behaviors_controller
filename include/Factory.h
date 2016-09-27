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

	std::vector<SteeringBehavior*> instanciateBehaviors(unsigned int id, std::string pre);

private: 

	std::vector<SteeringBehavior*>* vectorPtr;

	int nbAgents;

	string agentsType[];

	Config* cfg;
};

#endif //_FACTORY_H
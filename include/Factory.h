/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _FACTORY_H
#define _FACTORY_H

#include "SteeringBehavior.h"
#include "Weights.h"
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
	
	~Factory();

	int instanciateBehaviors(unsigned int id, std::string pre, 
							std::vector<SteeringBehavior*>* behaviors, 
							Weights** weights,
							std::string* type);

	int getAgents();

private: 

	int nbAgents;

	int nbAgentsCfg;

	// Setting& agentsType;

	Config* cfg;

	SteeringBehavior* pickBehavior(std::string behaviorName, int id, std::string pre);
};

#endif //_FACTORY_H
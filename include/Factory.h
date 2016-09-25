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

class Factory {

private: 

	static Factory *s_instance;

	std::vector<SteeringBehavior*>* vectorPtr;

	// config_t config;

	int nbAgents;

	string agentsType[];

	Factory();

public: 

	/**
	 * @param unsigned int id
	 */
	
	static Factory *instance(int s);
	
	~Factory();

	std::vector<SteeringBehavior*> instanciateBehaviors(unsigned int id, std::string pre);


};

#endif //_FACTORY_H
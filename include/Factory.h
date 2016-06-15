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
// #include <iostream>
// using std::cout;
// using std::cin;
// using std::endl;

class Factory {
public: 

	/**
	 * @param unsigned int id
	 */
	Factory();

	//Factory(i,behaviors[i], FactoryPtr);
	
	~Factory();

	std::vector<SteeringBehavior*> instanciateBehaviors(geometry_msgs::Pose objective, unsigned int id, std::string pre);

private: 

	std::vector<SteeringBehavior*>* vectorPtr;
};

#endif //_FACTORY_H
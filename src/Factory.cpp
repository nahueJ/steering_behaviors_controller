/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Factory.h"

Factory::Factory()
{

}
	
Factory::~Factory()
{

}

Factory *Factory::instance(int s = 0)
{
	inst = 
	if (s)
	{
		s_instance = new Factory();
		inst = 1;
	}
	return s_instance;
}

std::vector<SteeringBehavior*> Factory::instanciateBehaviors(unsigned int id, std::string pre)
{
	vectorPtr = new std::vector<SteeringBehavior*>;

	//Instanciate Seek
	SteeringBehavior* seekPtr = new Seek (id,pre);

	//Instanciate ObstacleAvoidance
	SteeringBehavior* obstacleavoidancePtr = new ObstacleAvoidance (id,pre);

	vectorPtr->push_back(seekPtr);
	vectorPtr->push_back(obstacleavoidancePtr);

	return *vectorPtr;
}
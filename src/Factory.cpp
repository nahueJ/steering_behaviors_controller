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

std::vector<SteeringBehavior*> Factory::instanciateBehaviors(geometry_msgs::Pose objective, unsigned int id, std::string pre)
{
	vectorPtr = new std::vector<SteeringBehavior*>;

	//Instanciate Seek
	SteeringBehavior* seekPtr = new Seek (objective,id,pre);

	//Instanciate ObstacleAvoidance
	SteeringBehavior* obstacleavoidancePtr = new ObstacleAvoidance (id,pre);

	vectorPtr->push_back(seekPtr);
	vectorPtr->push_back(obstacleavoidancePtr);

	return *vectorPtr;
}
/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Factory.h"

Factory::Factory(Configuration* configurationPtr)
{
	//Cargo los valores en el archivo de configuracion
	config = *configurationPtr;
}
	
Factory::~Factory()
{

}

std::vector<SteeringBehavior*> Factory::instanciateBehaviors(unsigned int id, std::string pre)
{
	vectorPtr = new std::vector<SteeringBehavior*>;

	//Instanciate Seek
	SteeringBehavior* seekPtr = new Seek (id,pre,&config);

	//Instanciate ObstacleAvoidance
	SteeringBehavior* obstacleavoidancePtr = new ObstacleAvoidance (id,pre,&config);

	vectorPtr->push_back(seekPtr);
	vectorPtr->push_back(obstacleavoidancePtr);

	return *vectorPtr;
}
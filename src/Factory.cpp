/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Factory.h"

Factory::Factory()
{
	//Cargo los valores en el archivo de configuracion
	cfg = new Config;

	cfg->readFile(CONFIGFILE);

	string test = "mistake";

	cfg->lookupValue("test", test);

	cout << "this was a " << test << endl;
}
	
Factory::~Factory()
{

}

std::vector<SteeringBehavior*> Factory::instanciateBehaviors(unsigned int id, std::string pre)
{
	vectorPtr = new std::vector<SteeringBehavior*>;

	//Instanciate Seek
	SteeringBehavior* seekPtr = new Seek (id,pre,cfg);

	//Instanciate ObstacleAvoidance
	SteeringBehavior* obstacleavoidancePtr = new ObstacleAvoidance (id,pre,cfg);

	vectorPtr->push_back(seekPtr);
	vectorPtr->push_back(obstacleavoidancePtr);

	return *vectorPtr;
}
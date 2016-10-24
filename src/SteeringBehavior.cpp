/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "SteeringBehavior.h"

/**
 * Abstract Class SteeringBehavior implementation
 */


/**
 * @param unasigned int id
 * @param float weight
 */
SteeringBehavior::SteeringBehavior(
	unsigned int id, 
	std::string pre, 
	Setting* configurationPtr) : 
	robotId( id ), 
	pretopicname(pre),
	config (configurationPtr)
{
	myName = (*configurationPtr)["name"].c_str();
	myType = (*configurationPtr)["type"].c_str();
	variables = (*configurationPtr)["variables"];
}

SteeringBehavior::~SteeringBehavior() {

}

string SteeringBehavior::getName() 
{
	return myName;
}

string SteeringBehavior::getType() 
{
	return myType;
}

int SteeringBehavior::getNbVbles() 
{
	return variables;
}

float SteeringBehavior::getDesiredV() 
{
	return desiredV;
}

float SteeringBehavior::getDesiredW() 
{
	return desiredW;
}

void SteeringBehavior::setDesiredV(float y) 
{
	desiredV = ( abs(y) <= 1.0 ) ? y : (y/abs(y));
}


void SteeringBehavior::setDesiredW(float z) 
{
	desiredW = ( abs(z) <= 1.0 ) ? z : (z/abs(z));
}

void SteeringBehavior::update(){

}

void SteeringBehavior::getVbles(std::vector<float>* v){
	if (variables>0)
	{
		(*v)[0]=getVble1();
		if (variables>1)
		{
			(*v)[1]=getVble2();
			if (variables>2)
			{
				(*v)[2]=getVble3();
			}
		}
	}
}

float SteeringBehavior::getVble1()
{
	return 0.0;
}
float SteeringBehavior::getVble2()
{
	return 0.0;
}
float SteeringBehavior::getVble3()
{
	return 0.0;
}

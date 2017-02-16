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

	//Variables para la definicion de estado
	int nbVar = (*configurationPtr)["variablesDeEstado"];
	std::string discret = (*configurationPtr)["discret"].c_str();

	if (discret == "iregular") {
		int nbVarPosibles = (*configurationPtr)["vectorEstados"].getLength();
		//inicializo el array con los posibles valores para las variables de estado
		Setting& sVal =(*configurationPtr)["vectorEstados"];
		for (int i = 0; i < nbVarPosibles; ++i)
		{
			valoresEstado.push_back(sVal[i]);
		}
	} else {
		float minValState = (*configurationPtr)["minEstado"];
		float maxValState = (*configurationPtr)["maxEstado"];
		float stepValState = (*configurationPtr)["paso"];

		int nbVarPosibles = ((maxValState-minValState)/stepValState)+1;

		//inicializo el array con los posibles valores para las variables de estado
		for (int i = 0; i < nbVarPosibles; ++i)
		{
			valoresEstado.push_back(minValState+i*stepValState);
		}
	}
	//inicializo el vector de estado con algun valor de los posibles valores
	for (int i = 0; i < nbVar; ++i)
	{
		state.push_back(valoresEstado.back());
	}
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
	return state.size();
}

std::vector<float> SteeringBehavior::getPosibleValues()
{
	return valoresEstado;
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

std::vector<float> SteeringBehavior::getState()
{

}

void SteeringBehavior::updateState()
{

}

void SteeringBehavior::setGoal(float xg, float yg){

}

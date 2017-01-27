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

	float minValState = (*configurationPtr)["minEstado"];
	float maxValState = (*configurationPtr)["maxEstado"];	
	float stepValState = (*configurationPtr)["paso"];

	int nbVarPosibles = ((maxValState-minValState)/stepValState)+1;

	//inicializo el array con los posibles valores para las variables de estado
	for (int i = 0; i < nbVarPosibles; ++i)
	{
		valoresEstado.push_back(minValState+i*stepValState);
		cout << valoresEstado[i] << " ";
	}
	cout << endl;
	//inicializo el vector de estado con algun valor de los posibles valores
	for (int i = 0; i < nbVar; ++i)
	{
		state.push_back(valoresEstado[valoresEstado.size()-1]);
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

int SteeringBehavior::getNbPosibleValues() 
{
	return valoresEstado.size();
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
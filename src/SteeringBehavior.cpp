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

	//genero un vector con todos los valores discretos que puede tomar la variable de estado, este puede ser regular (i.e. [0;0.5;1.0;1.5;...]) o irregular (i.e. [0;0.2;0.6;1.0;2.0])
	if (discret == "iregular")
	{
		int nbVarPosibles = (*configurationPtr)["vectorEstados"].getLength();
		Setting& sVal =(*configurationPtr)["vectorEstados"];
		for (int i = 0; i < nbVarPosibles; ++i)
		{
			valoresEstado.push_back(sVal[i]);
		}
	}
	else
	{
		float minValState = (*configurationPtr)["minEstado"];
		float maxValState = (*configurationPtr)["maxEstado"];
		float stepValState = (*configurationPtr)["paso"];
		int nbVarPosibles = ((maxValState-minValState)/stepValState)+1;

		for (int i = 0; i < nbVarPosibles; ++i)
		{
			valoresEstado.push_back(minValState+i*stepValState);
		}
	}
	//inicializo el vector de estado con algun valor de los posibles valores
	stateDiscrete=valoresEstado.back();
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

int SteeringBehavior::update(){

}

float SteeringBehavior::getState()
{
	return stateDiscrete;
}

void SteeringBehavior::updateState()
{

}

void SteeringBehavior::setGoal(float xg, float yg){

}

void SteeringBehavior::discretizarEstado ()
{
	//buscar dentro de los posibles valores aquel mas proximo al valor continuo
	int indexMin = 0;
	float min = stateContinuous - valoresEstado[indexMin];
	for (int i = 1; i < valoresEstado.size(); ++i)
	{
		if ((stateContinuous - valoresEstado[i]) > 0)
		{
			if ((stateContinuous - valoresEstado[i])<min)
			{
				min = stateContinuous - valoresEstado[i];
				indexMin=i;
			}
		}
		else{
			break;
		}
	}
	stateDiscrete = valoresEstado[indexMin];
}

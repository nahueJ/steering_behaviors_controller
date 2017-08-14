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
	nbVar = (*configurationPtr)["variablesDeEstado"];
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
			valoresEstado.push_back(ceil(minValState+i*stepValState*1000)/1000);
		}
	}
	int velDiscretization = 2;
	float velMaxVal = 1.00;
	for (int i = 1; i <= velDiscretization; i++) { //avoid vel=0, only (0.33;0.66;1)
		desiredVValues.push_back( ceil(velMaxVal*i/velDiscretization*1000)/1000 );
	}
	int orientDiscretization = 4;
	float orientMaxVal = 2*PI;
	for (int i = 1; i <= orientDiscretization; i++) { //avoid vel=0, only (0.33;0.66;1)
		desiredOValues.push_back( ceil(orientMaxVal*i/orientDiscretization*1000)/1000 );
	}
	//inicializo el vector de estado con algun valor de los posibles valores
	for (int i = 0; i < nbVar+2; i++) { //plus two to module and orientation of control action
		stateContinuous.push_back(0.0);
		stateDiscrete.push_back(0.0);
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

float SteeringBehavior::getDesiredV()
{
	return desiredV;
}

float SteeringBehavior::getDesiredO()
{
	return desiredO;
}

void SteeringBehavior::setDesiredV(float y)
{
	if ( fabs(y) <= 1.0 ){
		desiredV = y;
	} else {
		cout << myName << " ERROR EN DESIRED V: " << y << endl << endl << endl << endl;
	}
}


void SteeringBehavior::setDesiredO(float z)
{
	if ( fabs(z) <= PI ){
		desiredO = z;
	} else if (z < (-PI)) {
		desiredO = 2*PI+z;
	} else {
		desiredO = 0.0;
	}
}

int SteeringBehavior::update(){

}

std::vector<float> SteeringBehavior::getState()
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
	for (int i = 0; i < nbVar; i++) {
		//buscar dentro de los posibles valores aquel mas proximo al valor continuo
		int indexMin = 0;
		float min = stateContinuous[i] - valoresEstado[indexMin];
		for (int j = 1; j < valoresEstado.size(); ++j)
		{
			if ((stateContinuous[i] - valoresEstado[j]) > 0)
			{
				if ((stateContinuous[i] - valoresEstado[j])<min)
				{
					min = stateContinuous[i] - valoresEstado[j];
					indexMin=j;
				}
			}
			else{
				break;
			}
		}
		stateDiscrete[i] = ceil(valoresEstado[indexMin]*1000)/1000;
	}
	int indexMinVel = 0;
	float minVel = stateContinuous[nbVar] - desiredVValues[indexMinVel];
	for (int j = 1; j < desiredVValues.size(); ++j)
	{
		if ((stateContinuous[nbVar] - desiredVValues[j]) > 0)
		{
			if ((stateContinuous[nbVar] - desiredVValues[j])<minVel)
			{
				minVel = stateContinuous[nbVar] - desiredVValues[j];
				indexMinVel=j;
			}
		}
		else{
			break;
		}
	}
	stateDiscrete[nbVar] = ceil(desiredVValues[indexMinVel]*1000)/1000;

	int indexMinO = 0;
	float minO = stateContinuous[nbVar+1] - desiredOValues[indexMinO];
	for (int j = 1; j < desiredOValues.size(); ++j)
	{
		if ((stateContinuous[nbVar+1] - desiredOValues[j]) > 0)
		{
			if ((stateContinuous[nbVar+1] - desiredOValues[j])<minO)
			{
				minO = stateContinuous[nbVar+1] - desiredOValues[j];
				indexMinO=j;
			}
		}
		else{
			break;
		}
	}
	stateDiscrete[nbVar+1] = ceil(desiredOValues[indexMinO]*1000)/1000;
}

std::vector< std::vector<float> > SteeringBehavior::getPosibleValues()
{
	std::vector< std::vector<float> > aux;
	for (int i = 0; i < nbVar; i++) {
		aux.push_back(valoresEstado);
	}
	aux.push_back(desiredVValues);
	aux.push_back(desiredOValues);
	return aux;
}

std::vector<float> SteeringBehavior::getPosibleVelCtrlActValues()
{
	return desiredVValues;
}

std::vector<float> SteeringBehavior::getPosibleOCtrlActValues()
{
	return desiredOValues;
}

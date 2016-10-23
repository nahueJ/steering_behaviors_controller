/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "../include/Critic.h"

Critic::Critic(unsigned int id, Setting* configurationPtr,std::vector<SteeringBehavior*>* behaviorsPtr)
{	
	pesos = 0;
	coeficientes = 0;
	behaviors = *behaviorsPtr;
}

Critic::~Critic() 
{

}

float Critic::update(int wid,std::string type)
{
	//verif 1ro si paso el tpo actualizar los coef segun los refuerzos

	//actualizar todos las vbles
	//seekVbles, oaVbles. etc

	int bhIndex = getIndex(type);
	float w = 0.0;

	for (int i = 0; i < weights[bhIndex].size(); ++i)
	{
		/* code */
		//calc w como  sum pond del vect de coef por los del bhVbles
	}
	return w;
}

int Critic::addW(std::string type, float value)
{
	std::vector<float> v;
	for (int i = 0; i < pesos*2; ++i)
	{
		v.push_back(0.0);	//completo los coef
	}
	weights.push_back(v);
	pesos = weights.size();	
	for (int i = 0; i < pesos; ++i)
	{
		weights[i].push_back(0.0);	//completo los coef
		weights[i].push_back(0.0);	//completo los coef
	}
	coeficientes = weights[0].size();
	return 1;
}

int Critic::seekVbles(float* a, float* b)
{
	//calcular las 2 bles para la actualizacion de pesos
	//dist y delta ang
	return 1;
}

int Critic::obstacleAvoidanceVbles(float* a, float* b)
{
	//calcular las 2 bles para la actualizacion de pesos
	//area libre y dist mas cercana
	return 1;
}

int Critic::fleeVbles(float* a, float* b)
{
	//calcular las 2 bles para la actualizacion de pesos
	//dist y delta ang
	return 1;
}

int Critic::getIndex(std::string type)
{
	for (int i = 0; i < weights.size(); ++i)
	{
		if (type==behaviors[i]->getType())
		{
			return i;
		}
	}
	return -1;
}
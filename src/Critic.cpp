/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "../include/Critic.h"

Critic::Critic(unsigned int id, Setting* configurationPtr,std::vector<SteeringBehavior*>* behaviorsPtr)
{	
	sets = configurationPtr;
	behaviors = *behaviorsPtr;
	pesos = behaviors.size();
	coeficientes = 0;
	for (int i = 0; i < pesos; ++i)
	{
		coeficientes += behaviors[i]->getNbVbles(); //cuento cuantas vlbes representativas del estado de cada behavior tengo
	}
	for (int i = 0; i < pesos; ++i)
	{
		std::vector<float> vAux = fillCoefsInit(i);	//cargo el vector con valores iniciales de coefs para calcular el peso de un comportamiento
		weights.push_back(vAux);	//lo cargo a la lista de pesos		
	}
	showCoefsW();	//muestro el estado inicial
}

Critic::~Critic() 
{

}

std::vector<float> Critic::update()
{
	//verif 1ro si paso el tpo actualizar los coef segun los refuerzos

	//actualizar todos las vbles
	//seekVbles, oaVbles. etc
	std::vector<float> vbles;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		behaviors[i]->getVbles(&vbles);
	}
	//update coefs mat weights
	updateCoefsW();
	//calculo los w para el blend del agente
	std::vector<float> w;
	for (int i = 0; i < weights.size(); ++i)
	{
		w.push_back(0.0);
		for (int j = 0; j < vbles.size(); ++i)
		{
			w[i] += weights[i][j] * vbles[j];
		}
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

std::vector<float> Critic::fillCoefsInit(int b)
{
	std::vector<float> vAux;
	float aux;
	for (int i = 0; i < coeficientes; ++i)
	{
		if (behaviors[b]->getType()=="seek")
		{
			vAux.push_back((*sets)["seekI"][i]);
		}
		else if (behaviors[b]->getType()=="avoidObstacles")
		{
			vAux.push_back((*sets)["oaI"][i]);
		}
		else if (behaviors[b]->getType()=="flee")
		{
			vAux.push_back((*sets)["fleeI"][i]);
		}
	}
	return vAux; 
}

void Critic::showCoefsW(){
	for (int i = 0; i < pesos; ++i)
	{
		cout << "W" << i << ": " ;
		for (int j = 0; j < coeficientes; ++j)
		{
			cout << weights[i][j] << "  ";
		}
		cout << endl;
	}
}

void Critic::updateCoefsW(){
	//como actualizo los coeficientes
	//implementacion de la ec de q learning
	//necesito la medida de la utilidad...
}
/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _WEIGHTS_H
#define _WEIGHTS_H

#include <string>
#include <vector>

#include "Critic.h"

#include <libconfig.h++>
using namespace libconfig;

class Weights {
public: 

	Weights(std::string aprendizaje); // y el tipo de critico
	
	~Weights();

	std::vector<float> getWeights(); //paso el nombre solo para verificar que estoy devolviendo el que corresponde, pero si han sido bien cargados, agente deberia tener en el mismo orden los comportamientos y los pesos

	int addWeight(std::string name, float value);

	int setCritic(Critic* c);

private: 

	std::vector<float> weights;

	std::vector<std::string> names;
	
	std::string optimization;

	Critic* critic;
};

#endif //_WEIGHTS_H
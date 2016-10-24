/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "../include/Weights.h"


Weights::Weights(std::string aprendizaje){
	optimization = aprendizaje;
}

Weights::~Weights(){

}

std::vector<float> Weights::getWeights(){
	if (optimization == "si")
	{
		weights = critic->update();
	}
	return weights;
}

int Weights::addWeight(std::string name, float value){
	names.push_back(name);
	weights.push_back(value);
	if (optimization == "si")
	{
		//añadir al critico
	}
	return(names.size()-1);
}

int Weights::setCritic(Critic* c){
	critic = c;
	return 1;
}

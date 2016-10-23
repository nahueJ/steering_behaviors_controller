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
		if(update()){
			//cout de un OK 
			//guardar un historico...
		}
		else{
			//cout de error de update
		}
	}
	return weights;
}

int Weights::addWeight(std::string name, float value){
	names.push_back(name);
	weights.push_back(value);
	//informar al critico si hay
	return(names.size()-1);
}

int Weights::setCritic(Critic* c){
	critic = c;
	return 1;
}

int Weights::update(){
	//pedirle al critico q actualize el valor del peso
//	return	critico.update(wid,names[wid]);
	return 0.0;
}
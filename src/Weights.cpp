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

float Weights::getWeight(int id, std::string name){
	if (name == names[id])
	{
		if (optimization == "si")
		{
			if(update(id)){
				//cout de un OK 
				//guardar un historico...
			}
			else{
				//cout de error de update
			}
		}
		return weights[id];
	}
	else{
		return -1;
	}
}

int Weights::addWeight(std::string name, float value){
	names.push_back(name);
	weights.push_back(value);
	return(names.size()-1);
}

int Weights::setCritic(/* puntero del critico */){
	//asignarlo al atributo...
	return 1;
}

int Weights::update(int wid){
	//pedirle al critico q actualize el valor del peso
//	return	critico.update(wid);
	return 0.0;
}
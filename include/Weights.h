/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _WEIGHTS_H
#define _WEIGHTS_H

#include <string>
#include <vector>
#include <map>

#include <libconfig.h++>
using namespace libconfig;

struct ceroRuleStruct
{
	int behaviorNb;
	float ceroOver;
};

class Weights {
public: 

	Weights(std::vector<float>, Setting*); //constructor para pesos constantes contW

	Weights(std::vector< std::vector<float> >, Setting*); //constructor para pesos segun qTable que se actualiza qvalueW

//	Weights(); //constructor para pesos segun qTable constante constQvalueW	

//	Weights(); //constructor para pesos constantes constAnnW
	
	~Weights();

	std::vector<float> getWeights(std::vector< std::vector<float> >); //paso el nombre solo para verificar que estoy devolviendo el que corresponde, pero si han sido bien cargados, agente deberia tener en el mismo orden los comportamientos y los pesos

private: 

	std::string myType;

	//Vbles para pesos constantes
	std::vector<float>* weights;
	std::vector<ceroRuleStruct> ceroRules;
	std::vector<float> checkCeroRules(std::vector< std::vector<float> >);

};

#endif //_WEIGHTS_H
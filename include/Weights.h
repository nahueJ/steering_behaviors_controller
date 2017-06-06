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

#include <fstream>
#include <errno.h>

#include <libconfig.h++>
using namespace libconfig;

#include <stdlib.h>	//for system use
#include "ros/ros.h"	// for sleep use
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <iomanip>

#include <floatfann.h>
#include <fann.h>

struct ceroRuleStruct
{
	int behaviorNb;
	float ceroOver;
};

struct qTableOutput
{
	int visits;
	float qValue;
};

struct reinforcement
{
	int behaviorNb;
	float reinforcementState;
	float reinforcementValue;
	std::string message;
};

class Weights {
public:

	Weights(std::vector<float>, Setting*); //constructor para pesos constantes contW

	Weights(std::vector< std::vector< std::vector<float> > >, Setting*); //constructor para pesos segun qTable que se actualiza qvalueW

	Weights(Setting*); //constructor para pesos segun qTable constante constQvalueW

//	Weights(); //constructor para pesos constantes constAnnW

	~Weights();

	int getWeights(std::vector< float >, std::vector<float>*); //paso el nombre solo para verificar que estoy devolviendo el que corresponde, pero si han sido bien cargados, agente deberia tener en el mismo orden los comportamientos y los pesos

private:

	std::string myType;
	std::string file;

	//Vbles para pesos constantes
	std::vector<float>* weights;
	std::vector<ceroRuleStruct> ceroRules;
	std::vector<float> updateConstW(std::vector< float >);

	//Vbles para aprendizaje de pesos qTable
	//Vbles
	std::map<std::vector<float> , qTableOutput, std::less< std::vector<float> >, std::allocator< std::pair<std::vector<float> , qTableOutput> > > qTable;
	std::vector< std::vector<float> > wCombinacionesPosibles;
	std::vector< std::map<std::vector<float> , qTableOutput>::iterator > memoria;
	std::vector<reinforcement> critic;
	float gamma;
	int allocateNb;
	std::pair< std::vector<float> , qTableOutput>* allocP;

	int maxVisitasDif;
	//Fcs
	void wPermutaciones(std::vector<float>, std::vector<float>, int, std::vector< std::vector<float> >*);
	void sPermutaciones(std::vector< std::vector<float> >, std::vector<float>, int, std::vector< std::vector<float> >*);
	void printPerm(std::vector< std::vector<float> >);
	int writeQTableToFile(std::string fname);
	std::vector<float> getRandomWfromQTable(std::vector<float>);
	int criticCheck(std::vector< float >);
	void actualizarQTable(int);
	void instanciarWcombinaciones(int, int);
	int checkVisits(std::vector<float>);

	//Vbles para probar pesos qTable
	void loadQTable(std::string, int);
	std::vector<float> getBestWfromQTable(std::vector<float>);
	void printQTable();

	//Vbles para continuar el aprendizaje

	int missStartFlag;
	std::vector<float> wNull;

};

#endif //_WEIGHTS_H

/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "AgentQLTraining.h"

AgentQLTraining::AgentQLTraining(unsigned int id, string type, Factory* factoryPtr) : Agent(id, type, factoryPtr)
{
	file = (*configurationPtr)["file"].c_str();

	if (myType == "qlInit") {
		newQTable();
	}
}

AgentQLTraining::~AgentQLTraining()
{

}

std::vector<float> AgentQLTraining::getWeights(std::vector<float> estado)
{
	return pesos;
}

int AgentQLTraining::loadQTable(){

}

int AgentQLTraining::newQTable(){
	//Valores de entradas de la Qtable (permutaciones de los estados de los comportamientos)
	std::vector< std::vector<float> > state;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		std::vector<float> auxVect = behaviors[i]->getState();
		state.push_back(auxVect);
	}
	std::vector< std::vector< std::vector<float> > > statePosibilities;
	for (int i = 0; i < state.size(); ++i) //para el estado de cada comportamiento
	{
		std::vector< std::vector<float> > auxv;
		for (int j = 0; j < state[i].size(); ++j) //para cada variable de estado del comportamientos
		{
			auxv.push_back(behaviors[i]->getPosibleValues());
		}
		statePosibilities.push_back(auxv);
	}
	//Valores de salidas de la Qtable (permutaciones de los pesos)
	int wCantDiscretizacion = (*configurationPtr)["wPosibles"];
	instanciarWcombinaciones(wCantDiscretizacion, statePosibilities.size());
	//	posibles estados
	std::vector< std::vector<float> > sCombinacionesPosibles;
	std::vector< std::vector<float> > sValPosibles;
	for (std::vector< std::vector< std::vector< float > > >::iterator ita = statePosibilities.begin(); ita != statePosibilities.end(); ++ita)
	{
		for (std::vector< std::vector<float> >::iterator itb = ita->begin(); itb != ita->end(); ++itb)
		{
			sValPosibles.push_back(*itb);
		}
	}
	std::vector<float> individuo;
	sPermutaciones(sValPosibles, individuo, sValPosibles.size(), &sCombinacionesPosibles);
	//	Instanciación de la estructura map para almacenar la qTable

	//Lista de inputs para la qTable
	std::vector<std::vector<float> > inputs;
	for (std::vector< std::vector<float> >::iterator its = sCombinacionesPosibles.begin(); its != sCombinacionesPosibles.end(); ++its)
	{
		for (std::vector< std::vector<float> >::iterator itw = wCombinacionesPosibles.begin(); itw != wCombinacionesPosibles.end(); ++itw)
		{
			std::vector<float> auxInput;
			auxInput = *its;
			auxInput.insert( auxInput.end(), itw->begin(), itw->end() );
			inputs.push_back(auxInput);
		}
	}
	//Output generico
	qTableOutput out;
	out.visits = 0;
	out.qValue = (*configurationPtr)["qValInit"];
	//Carga de la qTable
	int numberOfNodes = inputs.size();
	int nodeSize = sizeof(std::map<std::vector<float> , qTableOutput>::value_type);
	allocateNb = nodeSize * numberOfNodes;
	allocP = qTable.get_allocator().allocate( allocateNb );
	for (std::vector< std::vector<float> >::iterator i = inputs.begin(); i != inputs.end(); ++i)
	{
		qTable[*i] = out;
	}
	//Se guarda en un archivo
	int aux = writeQTableToFile(file);
}

void AgentQLTraining::instanciarWcombinaciones(int wCantDiscretizacion, int size)
{
	//	Instanciacion del vector de posibles outputs / decisiones (combinaciones de pesos)
	//	sagun la discretizacion tomada de la configuracion
	float step = 0.8 / (wCantDiscretizacion-1);
	std::vector<float> wValPosibles;
	for (int i = 0; i < wCantDiscretizacion; ++i)
	{
		wValPosibles.push_back(step * i + 0.1);
	}
	//la suma de los pesos (en primera instancia) debe ser 1. La estrategia es hacer todas las combinaciones posibles de valores
	//multiplos del step entre 0 y 1 (para step 0.2 [0.0 0.2 0.4 0.6 0.8 1.0]) y almacenar aquellas combinaciones donde la suma sea 1
	std::vector<float> individuo;
	wPermutaciones(wValPosibles, individuo, size, &wCombinacionesPosibles);
}

void AgentQLTraining::wPermutaciones(std::vector<float> valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor){
	if (longitud == 0) {
		float add = 0;
		for (std::vector<float>::iterator ii = individuo.begin(); ii != individuo.end(); ++ii)
		{
			add += *ii;
		}
		if (add == 1.000)
		{
			(*contenedor).push_back(individuo);
		}
	} else {
		for (int i = 0; i < valores.size(); i++) {
			std::vector<float> variante = individuo;
			variante.push_back(valores[i]);
			wPermutaciones(valores, variante, (longitud - 1), contenedor);
		}
	}
}

void AgentQLTraining::sPermutaciones(std::vector< std::vector<float> > valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor){
	if (longitud == 0) {
		(*contenedor).push_back(individuo);
	} else {
		std::vector<float> auxv = valores[valores.size() - longitud];
		for (int i = 0; i < auxv.size(); i++) {
			std::vector<float> variante = individuo;
			variante.push_back(auxv[i]);
			sPermutaciones(valores, variante, (longitud - 1), contenedor);
		}
	}
}

int AgentQLTraining::writeQTableToFile(std::string fname) {
	int count = 0;
	if (qTable.empty())
			return 0;
	FILE *fp = fopen(fname.c_str(), "w");
	if (!fp)
			return -errno;
	for(std::map<std::vector<float> , qTableOutput>::iterator itm = qTable.begin(); itm != qTable.end(); itm++) {
		std::vector<float> auxv = itm->first;
		for (std::vector<float>::iterator itv = auxv.begin(); itv != auxv.end(); ++itv)
		{
			fprintf(fp, "%2.3f ", *itv);
		}
		fprintf(fp, "= %i %1.3f\n", itm->second.visits, itm->second.qValue);
		count++;
	}
	fclose(fp);
	return count;
}

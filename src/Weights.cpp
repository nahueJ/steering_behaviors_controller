/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "../include/Weights.h"

//constructor para constW
Weights::Weights(std::vector<float> w, Setting* configurationPtr)
{
	myType = (*configurationPtr)["type"].c_str();
	weights = new std::vector<float>;
	*weights = w;
	ceroRules.clear();
	int nbRules = (*configurationPtr)["ceroRules"].getLength();
	if (nbRules>0)
	{
		Setting& rules =(*configurationPtr)["ceroRules"];
		for (int i = 0; i < nbRules; ++i)
		{
			ceroRuleStruct auxRule;
			auxRule.behaviorNb = rules[i][0];
			auxRule.ceroOver = rules[i][1];
			ceroRules.push_back(auxRule);
		}
	}
}

//constructor para qTableW
Weights::Weights(std::vector< std::vector< std::vector<float> > > statePosibles, Setting* configurationPtr){
	myType = (*configurationPtr)["type"].c_str();
	/**************************************************************************************/
	/*	Instanciacion del vector de posibles outputs / decisiones (combinaciones de pesos)*/
	/*	sagun la discretizacion tomada de la configuracion								  */
	/**************************************************************************************/
	int wCantDiscretizacion = (*configurationPtr)["wPosibles"];
	float step = 1.0 / (wCantDiscretizacion-1);
	std::vector<float> wValPosibles;
	for (int i = 0; i < wCantDiscretizacion; ++i)
	{
		wValPosibles.push_back(step * i);
	}
	
	//la suma de los pesos (en primera instancia) debe ser 1. La estrategia es hacer todas las combinaciones posibles de valores
	//multiplos del step entre 0 y 1 (para step 0.2 [0.0 0.2 0.4 0.6 0.8 1.0]) y almacenar aquellas combinaciones donde la suma sea 1
	std::vector<float> individuo;
	wPermutaciones(wValPosibles, individuo, statePosibles.size(), &wCombinacionesPosibles);
	cout << "combinaciones W obtenidas " << wCombinacionesPosibles.size() << endl;

	/*	posibles estados */
	std::vector< std::vector<float> > sCombinacionesPosibles;
	std::vector< std::vector<float> > sValPosibles;
	for (std::vector< std::vector< std::vector< float > > >::iterator ita = statePosibles.begin(); ita != statePosibles.end(); ++ita)
	{
		for (std::vector< std::vector<float> >::iterator itb = ita->begin(); itb != ita->end(); ++itb)
		{
			sValPosibles.push_back(*itb);
		}
	}
	sPermutaciones(sValPosibles, individuo, sValPosibles.size(), &sCombinacionesPosibles);
	cout << "combinaciones S obtenidas " << sCombinacionesPosibles.size() << endl;
	// printPerm(sCombinacionesPosibles);

	/****************************************************************/
	/*	Instanciación de la estructura map para almacenar la qTable */
	/****************************************************************/	

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
	cout << "inputs: " << inputs.size() << endl;
	//Output generico
	qTableOutput out;
	out.visits = 0;
	out.qValue = (*configurationPtr)["qValInit"];
	//Carga de la qTable
	for (std::vector< std::vector<float> >::iterator i = inputs.begin(); i != inputs.end(); ++i)
	{
		qTable[*i] = out;
	}
	//Se guarda en un archivo

	int aux = writeQTableToFile(QTABLEFILE);
	cout << "Se almaceno en " << QTABLEFILE << " " << aux << " entradas." << endl;
}


Weights::~Weights(){

}

std::vector<float> Weights::getWeights(std::vector< std::vector<float> > state){
	if (myType == "constW")
	{
		if (!ceroRules.empty())
		{
			return updateConstW(state);
		}
	}else if (myType == "qvalueW")
	{
		std::vector<float> statePart;
		for (std::vector< std::vector< float > >::iterator ita = state.begin(); ita != state.end(); ++ita)
		{
			for (std::vector<float>::iterator itb = ita->begin(); itb != ita->end(); ++itb)
			{
				statePart.push_back(*itb);
			}
		}
		return updateQTableW(statePart);
	}
	return *weights;
}

std::vector<float> Weights::updateConstW(std::vector< std::vector<float> > state){
	std::vector<float> wAux = (*weights);
	float amountToDistribute = 0;
	int wToDistribute = 0;
	for (std::vector<ceroRuleStruct>::iterator irule = ceroRules.begin(); irule != ceroRules.end(); ++irule)
	{
		bool flag = true;
		for (std::vector<float>::iterator istate = (state[(*irule).behaviorNb]).begin(); istate != (state[(*irule).behaviorNb]).end(); ++istate)
		{
			if (*istate < (*irule).ceroOver)
			{
				//si se rompe la regla
				flag = false;
			}
		}
		//si la regla nunca se rompio
		if (flag)
		{
			amountToDistribute = wAux[(*irule).behaviorNb];
			wAux[(*irule).behaviorNb] = 0.000;
			wToDistribute++;
		}
	}
	//si se cumplio alguna regla para obviar un comportamiento, distribuyo el peso en los demas comportamientos
	if (wToDistribute > 0)
	{
		float added = amountToDistribute/(state.size()-wToDistribute);
		for (std::vector<float>::iterator i = wAux.begin(); i != wAux.end(); ++i)
		{
			if (*i != 0.000)
			{
				*i += added;
			}
		}
	}
	return wAux;
}

void Weights::wPermutaciones(std::vector<float> valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor){
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

void Weights::sPermutaciones(std::vector< std::vector<float> > valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor){
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

void Weights::printPerm(std::vector< std::vector<float> > perm )
{
	cout << "Combinaciones posibles: " << endl;
	for (std::vector< std::vector<float> >::iterator ita = perm.begin(); ita < perm.end(); ++ita)
	{
		for (std::vector<float>::iterator itb = (*ita).begin(); itb < (*ita).end(); ++itb)
		{
			cout << *itb << " ";
		}		
		cout << endl;
	}
	cout << endl;
}

int Weights::writeQTableToFile(std::string fname) {
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

std::vector<float> Weights::updateQTableW(std::vector<float> state){
	//Genero todos los posibles inputs estado/pesos, correspondientes al estado actual
	std::vector< std::vector<float> > options;
	for (std::vector< std::vector<float> >::iterator itw = wCombinacionesPosibles.begin(); itw != wCombinacionesPosibles.end(); ++itw)
	{
		std::vector<float> aux = state;
		aux.insert( aux.end(), itw->begin(), itw->end() );
		options.push_back(aux);
	}
	//Extraigo de la qTable los valores de visitas y qValue correspondiente a las posibilidades
	std::vector<qTableOutput> outputs;
	for (std::vector< std::vector<float> >::iterator iti = options.begin(); iti != options.end(); ++iti)
	{
		outputs.push_back(qTable[*iti]);
	}
	//Evaluando las salidas elijo la mejor opcion para el estado actual
	int best = 0;
	int index = 0;
	for (std::vector<qTableOutput>::iterator ito = outputs.begin(); ito != outputs.end(); ++ito, index++)
	{
		if ((ito->qValue)>(outputs[best].qValue))
		{
			best = index;
		}
	}
	//Agrego el mejor elegido a la lista de estados visitados, para la posterior actualizacion de los qValues correspondientes en funcion de los refuerzos recibidos en un futuro
	std::map<std::vector<float> , qTableOutput>::iterator itaux = qTable.find(options[best]);
	memoria.push_back(itaux);

	//SOLO PARA TEST
	// cout << "Se almaceno en la memoria puntero al elemento de la qTable ";
	// std::vector<float> auxv = (memoria.back())->first;
	// for (std::vector<float>::iterator iv = auxv.begin(); iv != auxv.end(); ++iv)
	// {
	// 	cout << *iv << " ";		
	// }
	// qTableOutput auxo = (memoria.back())->second;
	// cout << "= " << auxo.visits << " " << auxo.qValue << endl;
	// cout << "updateQTableW returns ";
	// for (std::vector<float>::iterator i = wCombinacionesPosibles[best].begin(); i != wCombinacionesPosibles[best].end(); ++i)
	// {
	// 	cout << *i << " ";
	// }
	// cout << endl;

	return wCombinacionesPosibles[best];
}

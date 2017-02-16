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
	// struct fann *ann = fann_create_standard(4, 2, 8, 9, 1);
	myType = (*configurationPtr)["type"].c_str();
	gamma = (*configurationPtr)["relativeValue"];
	int wCantDiscretizacion = (*configurationPtr)["wPosibles"];
	file = (*configurationPtr)["file"].c_str();
	maxVisitasDif = (*configurationPtr)["minDeltaVisitas"];

	instanciarWcombinaciones(wCantDiscretizacion, statePosibles.size());
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
	std::vector<float> individuo;
	sPermutaciones(sValPosibles, individuo, sValPosibles.size(), &sCombinacionesPosibles);
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
	//se cargan los estados a los que corresponden los refuerzos
	critic.clear();
	int nbReinforcements = (*configurationPtr)["refuerzos"].getLength();
	if (nbReinforcements>0)
	{
		Setting& reinf =(*configurationPtr)["refuerzos"];
		for (int i = 0; i < nbReinforcements; ++i)
		{
			reinforcement auxReinf;
			auxReinf.behaviorNb = reinf[i][0];
			auxReinf.reinforcementState = reinf[i][1];
			auxReinf.reinforcementValue = reinf[i][2];
			auxReinf.message = reinf[i][3].c_str();
			critic.push_back(auxReinf);
		}
	}
	for (int i = 0; i < statePosibles.size(); ++i)
	{
		wNull.push_back(0.000);
	}
}

//constructor para qTableW
Weights::Weights(Setting* configurationPtr){
	myType = (*configurationPtr)["type"].c_str();
	int wCantDiscretizacion = (*configurationPtr)["wPosibles"];
	int wSize = (*configurationPtr)["wSize"];
	instanciarWcombinaciones(wCantDiscretizacion, wSize);
	file = (*configurationPtr)["file"].c_str();
	int stateSize = (*configurationPtr)["sSize"];
	int weightSize = (*configurationPtr)["wSize"];
	int inputSize = stateSize + weightSize;
	loadQTable(file,inputSize);
	if (myType == "qvalueW") {
		gamma = (*configurationPtr)["relativeValue"];
		maxVisitasDif = (*configurationPtr)["minDeltaVisitas"];
		//se cargan los estados a los que corresponden los refuerzos
		critic.clear();
		int nbReinforcements = (*configurationPtr)["refuerzos"].getLength();
		if (nbReinforcements>0)
		{
			Setting& reinf =(*configurationPtr)["refuerzos"];
			for (int i = 0; i < nbReinforcements; ++i)
			{
				reinforcement auxReinf;
				auxReinf.behaviorNb = reinf[i][0];
				auxReinf.reinforcementState = reinf[i][1];
				auxReinf.reinforcementValue = reinf[i][2];
				auxReinf.message = reinf[i][3].c_str();
				critic.push_back(auxReinf);
			}
		}
	}
}

Weights::~Weights(){
	qTable.get_allocator().deallocate(allocP,allocateNb);
}

int Weights::getWeights(std::vector< std::vector<float> > state, std::vector<float>* wOut){
	if (myType == "constW")
	{
		if (!ceroRules.empty())
		{
			*wOut = updateConstW(state);
			return 0;
		}
		*wOut = *weights;
		return 0;
	} else {
		std::vector<float> statePart;
		std::vector<float> wAux;
		for (std::vector< std::vector< float > >::iterator ita = state.begin(); ita != state.end(); ++ita)
		{
			for (std::vector<float>::iterator itb = ita->begin(); itb != ita->end(); ++itb)
			{
				statePart.push_back(*itb);
			}
		}
		if (myType == "qvalueW") {
			wAux = getRandomWfromQTable(statePart);
			//Verifico que el estado no corresponde a ningun refuerzo
			int testigo = criticCheck(state);
			if (testigo != -1) {
				if (memoria.size()==1) {
					memoria.clear();
					*wOut = wNull;
					return 0;
				}else{
					//si corresponde a algun refuerzo, se actualizan los valores de la tabla,
					actualizarQTable(testigo);
					//se limpia la memoria,
					memoria.clear();
					//y se devuelve un vector de pesos nulos, para reiniciar aprendizaje a partir del proximo estado
					*wOut = wNull;
					return 1;
				}
			}
		} else if (myType == "constQvalueW") {
			wAux = getBestWfromQTable(statePart);
		}
		*wOut = wAux;
		return 0;
	}
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

std::vector<float> Weights::getRandomWfromQTable(std::vector<float> state){
	//Eleccion: si la diferencia de visitas a diferentes acciones en el mismo estado es mayor q la dada en la configuracion, se elije la menos visitada, si no se elije aleatoriamente
	int eleccion = checkVisits(state);
	//Genero el input para la qTable
	state.insert( state.end(), wCombinacionesPosibles[eleccion].begin(), wCombinacionesPosibles[eleccion].end() );
	//Agrego el elegido a la lista de estados visitados, para la posterior actualizacion de los qValues correspondientes en funcion de los refuerzos recibidos en un futuro
	std::map<std::vector<float> , qTableOutput, std::less< std::vector<float> >, std::allocator< std::pair<std::vector<float> , qTableOutput> > >::iterator itaux = qTable.find(state);
	memoria.push_back(itaux);
	return wCombinacionesPosibles[eleccion];
}

std::vector<float> Weights::getBestWfromQTable(std::vector<float> state){
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
	float bestQval=outputs[best].qValue;
	for (std::vector<qTableOutput>::iterator ito = outputs.begin(); ito != outputs.end(); ++ito, index++)
	{
		if ((ito->qValue)>(outputs[best].qValue))
		{
			best = index;
			bestQval =ito->qValue;
		}
	}
	cout<< "lo mejor que habia " << bestQval<< endl;
	return wCombinacionesPosibles[best];
}

int Weights::criticCheck(std::vector< std::vector<float> > state){
	int index = 0;
	for (std::vector<reinforcement>::iterator icritic = critic.begin(); icritic != critic.end(); ++icritic, index++)
	{
		bool flag = true;
		for (std::vector<float>::iterator istate = (state[(*icritic).behaviorNb]).begin(); istate != (state[(*icritic).behaviorNb]).end(); ++istate)
		{
			if (*istate == (*icritic).reinforcementState)
			{
				//si se encuentra en un estado de refuerzo, se devuelve el indice del refuerzo en cuestion
				cout << (*icritic).message.c_str() << endl;
				return index;
			}
		}
	}
	//si no se aplica ningun refuerzo se envia -1
	return -1;
}

void Weights::actualizarQTable(int refuerzo){
	std::string mensaje = critic[refuerzo].message;
	cout << "Aplicando refuerzo " << mensaje << " a " << memoria.size() << " estados" << endl;
	int count = 0;
	for (std::vector< std::map<std::vector<float> , qTableOutput>::iterator >::reverse_iterator itmem = memoria.rbegin() ; itmem != memoria.rend(); itmem++, count++)
	{
		std::vector<float> printv = (*itmem)->first;
		for (std::vector<float>::iterator itv = printv.begin(); itv < printv.end(); itv++){
			cout << std::fixed << std::setprecision(3) << *itv << " ";
		}

		cout << "prevQVal=" << (*itmem)->second.qValue ;
		//NONDETERMINISTIC REWARDS & TEMPORAL DIFFERENCE LEARNING
		float alpha = 1/(1+(*itmem)->second.visits);

		(*itmem)->second.qValue = (1-alpha)*(*itmem)->second.qValue + alpha* pow(gamma,count) *critic[refuerzo].reinforcementValue;

		(*itmem)->second.visits++;

		cout << " actQVal=" << (*itmem)->second.qValue << " visits=" << (*itmem)->second.visits << endl;

	}
	memoria.clear();
	int aux = writeQTableToFile(file);
}

void Weights::loadQTable(std::string file, int inputSize){
	std::ifstream aFile (file.c_str());
	int numberOfNodes=std::count(std::istreambuf_iterator<char>(aFile), std::istreambuf_iterator<char>(), '\n');
	int nodeSize = sizeof(std::map<std::vector<float> , qTableOutput>::value_type);
	allocateNb = nodeSize * numberOfNodes;
	allocP = qTable.get_allocator().allocate( allocateNb );
	cout << allocateNb << "=" << nodeSize << "*" << numberOfNodes << endl;
	//para cada linea
	aFile.seekg(0);
	for (size_t line = 0; line < numberOfNodes; line++) {
		//extraigo el input
		std::vector<float> inV;
		for (size_t in = 0; in < inputSize; in++) {
			float aux;
			aFile >> aux;
			inV.push_back(aux);
		}
		//extraigo el Output
		char dummy; //para el "="
		qTableOutput outAux;
		aFile >> dummy >> outAux.visits >> outAux.qValue ;
		//lo cargo al mapa
		qTable[inV] = outAux;
	}
	cout << "Se copiaron a la tabla final " << qTable.size() << endl;
	// printQTable();
}

void Weights::instanciarWcombinaciones(int wCantDiscretizacion, int size){
	/**************************************************************************************/
	/*	Instanciacion del vector de posibles outputs / decisiones (combinaciones de pesos)*/
	/*	sagun la discretizacion tomada de la configuracion								  */
	/**************************************************************************************/
	float step = 1.0 / (wCantDiscretizacion-1);
	std::vector<float> wValPosibles;
	for (int i = 0; i < wCantDiscretizacion; ++i)
	{
		wValPosibles.push_back(step * i);
	}
	//la suma de los pesos (en primera instancia) debe ser 1. La estrategia es hacer todas las combinaciones posibles de valores
	//multiplos del step entre 0 y 1 (para step 0.2 [0.0 0.2 0.4 0.6 0.8 1.0]) y almacenar aquellas combinaciones donde la suma sea 1
	std::vector<float> individuo;
	wPermutaciones(wValPosibles, individuo, size, &wCombinacionesPosibles);
}

void Weights::printQTable(){
	for(std::map<std::vector<float> , qTableOutput>::iterator itm = qTable.begin(); itm != qTable.end(); itm++) {
		std::vector<float> auxv = itm->first;
		for (std::vector<float>::iterator itv = auxv.begin(); itv != auxv.end(); ++itv)
		{
			cout << (*itv) << " ";
		}
		cout << "=" << itm->second.visits << " " << itm->second.qValue << endl;
	}
}

int Weights::checkVisits(std::vector<float> state){
	//Genero todos los posibles inputs estado/pesos, correspondientes al estado actual
	std::vector< std::vector<float> > options;
	for (std::vector< std::vector<float> >::iterator itw = wCombinacionesPosibles.begin(); itw != wCombinacionesPosibles.end(); ++itw){
		std::vector<float> aux = state;
		aux.insert( aux.end(), itw->begin(), itw->end() );
		options.push_back(aux);
	}
	//Evaluo las visitas a las diferentes estados
	int lessIndex = 0;
	qTableOutput output = qTable[options[lessIndex]];
	int most = output.visits;
	int less = output.visits;
	int index = 0;
	for (std::vector< std::vector<float> >::iterator iti = options.begin(); iti != options.end(); ++iti, index++)
	{
		output = qTable[*iti];
		if (less > output.visits) {
			less = output.visits;
			lessIndex = index;
		}
		if (output.visits > most) {
			most = output.visits;
		}
	}
	if ((most-less)>maxVisitasDif) {
		cout << "opcion con " << less << " visitas" << endl;
		return lessIndex;
	} else{
		return rand()% wCombinacionesPosibles.size();
	}
}

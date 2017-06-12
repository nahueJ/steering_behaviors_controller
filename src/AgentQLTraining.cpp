/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "AgentQLTraining.h"

AgentQLTraining::AgentQLTraining(unsigned int id, string type, Factory* factoryPtr) : Agent(id, type, factoryPtr)
{
	//parametros para instanciar la estructura qtable
	file = (*configurationPtr)["file"].c_str();

	//Valores de salidas de la Qtable (permutaciones de los pesos)
	int wCantDiscretizacion = (*configurationPtr)["wPosibles"];
	int weightSize = (*configurationPtr)["behaviors"].getLength();
	instanciarWcombinaciones(wCantDiscretizacion, weightSize);
	if (myType == "qlInit") {
		newQTable();
	}
	else if (myType == "qlLoad") {
		//Valores de entradas de la Qtable (permutaciones de los estados de los comportamientos)
		std::vector< float > behaviorState;
		for (int i = 0; i < behaviors.size(); ++i)
		{
			std::vector<float> aux = behaviors[i]->getState();
			behaviorState.insert(behaviorState.end(), aux.begin(), aux.end());
		}
		int stateSize = behaviorState.size();
		int inputSize = stateSize + weightSize;
		loadQTable(file,inputSize);
	}

	//Parametros de aprendizaje
	Setting* trainCfgPtr = factoryPtr->getTypeSetting("qlTrain");

	gamma = (*trainCfgPtr)["gamma"];
	maxVisitasDif = (*trainCfgPtr)["minDeltaVisitas"];
	//se cargan los estados a los que corresponden los refuerzos
	critic.clear();
	int nbReinforcements = (*trainCfgPtr)["refuerzos"].getLength();
	if (nbReinforcements>0)
	{
		Setting& reinf =(*trainCfgPtr)["refuerzos"];
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

AgentQLTraining::~AgentQLTraining()
{
	qTable.get_allocator().deallocate(allocP,allocateNb);
}

int AgentQLTraining::loadQTable(std::string file, int inputSize)
{
	std::ifstream aFile (file.c_str());
	int numberOfNodes=std::count(std::istreambuf_iterator<char>(aFile), std::istreambuf_iterator<char>(), '\n');
	int nodeSize = sizeof(std::map<std::vector<float> , qTableOutput>::value_type);
	allocateNb = nodeSize * numberOfNodes;
	allocP = qTable.get_allocator().allocate( allocateNb );
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
	return aux;
}

void AgentQLTraining::instanciarWcombinaciones(int wCantDiscretizacion, int cantComportamientos)
{
	//	Instanciacion del vector de posibles outputs / decisiones (combinaciones de pesos)
	//	sagun la discretizacion tomada de la configuracion
	float step = 0.8 / (wCantDiscretizacion-1);
	std::vector<float> wValPosibles;
	for (int i = 0; i < wCantDiscretizacion; ++i)
	{
		wValPosibles.push_back(step * i + 0.1);
		// cout << step * i + 0.1 << " " ;
	}
	//la suma de los pesos (en primera instancia) debe ser 1. La estrategia es hacer todas las combinaciones posibles de valores
	//multiplos del step entre 0 y 1 (para step 0.2 [0.0 0.2 0.4 0.6 0.8 1.0]) y almacenar aquellas combinaciones donde la suma sea 1
	std::vector<float> individuo;
	wPermutaciones(wValPosibles, individuo, cantComportamientos, &wCombinacionesPosibles);
}

void AgentQLTraining::wPermutaciones(std::vector<float> valores, std::vector<float> individuo , int longitud, std::vector< std::vector<float> >* contenedor){
	if (longitud == 0) {
		float add = 0;
		for (std::vector<float>::iterator ii = individuo.begin(); ii != individuo.end(); ++ii)
		{
			add += *ii;
		}
		if (fabs(add-1.000) < 0.01)
		{
			(*contenedor).push_back(individuo);
		}
	}
	else
	{
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

/*void AgentQLTraining::printPerm(std::vector< std::vector<float> > perm )
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
}*/

std::vector<float> AgentQLTraining::getWeights(std::vector<float> estado)
{
	pesos = getRandomWfromQTable(estado);
	//Verifico que el estado no corresponde a ningun refuerzo
	int testigo = criticCheck();
	if (testigo != -1) {
		if (memoria.size()==1) {
			memoria.clear();
		}else{
			//si corresponde a algun refuerzo, se actualizan los valores de la tabla,
			actualizarQTable(testigo);
			//se limpia la memoria,
			memoria.clear();
		}
	}
	return pesos;
}

std::vector<float> AgentQLTraining::getRandomWfromQTable(std::vector<float> state)
{
	//Eleccion: si la diferencia de visitas a diferentes acciones en el mismo estado es mayor q la dada en la configuracion, se elije la menos visitada, si no se elije aleatoriamente
	int eleccion = checkVisits(state);
	//Genero el input para la qTable
	state.insert( state.end(), wCombinacionesPosibles[eleccion].begin(), wCombinacionesPosibles[eleccion].end() );
	//Agrego el elegido a la lista de estados visitados, para la posterior actualizacion de los qValues correspondientes en funcion de los refuerzos recibidos en un futuro
	std::map<std::vector<float> , qTableOutput, std::less< std::vector<float> >, std::allocator< std::pair<std::vector<float> , qTableOutput> > >::iterator itaux = qTable.find(state);
	memoria.push_back(itaux);
	return wCombinacionesPosibles[eleccion];
}

int AgentQLTraining::criticCheck()
{
	std::vector< std::vector<float> > state;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		std::vector<float> auxVect = behaviors[i]->getState();
		state.push_back(auxVect);
	}
	int index = 0;
	for (std::vector<reinforcement>::iterator icritic = critic.begin(); icritic != critic.end(); ++icritic, index++)
	{
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

void AgentQLTraining::actualizarQTable(int refuerzo)
{
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

int AgentQLTraining::checkVisits(std::vector<float> state)
{
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

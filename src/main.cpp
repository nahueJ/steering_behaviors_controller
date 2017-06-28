//------------------------------------------------------------------------
//
//  Name:   main.cpp
//
//  Desc:   Instancia los controladores para la cantidad de robots
//          existentes en el mundo del simulador
//
//  Author: Nahuel Jose 2016 (nahuelj91@gmail.com)
//
//------------------------------------------------------------------------

#include "ros/ros.h"
#include "../include/Agent.h"
#include "../include/AgentReactive.h"
#include "../include/AgentQLTraining.h"
#include "../include/Factory.h"

#include <stdio.h>
#include <stdlib.h> //for system use
#include <string>
#include <iostream>
#include <fstream>

//Recibe la posicion inicial del robot y las posiciones de objetivos posibles para elegir uno aleatorio y calcular la posicion objetivo en el sistema de cordenadas referido al vehiculo
std::pair<float, float> calcObjective(string strPos, std::vector< std::pair<float, float> > initPosition)
{
	std::stringstream pos(strPos);
	float data[4];
	//extraigo de la cadena x, y y el angulo de inicio
	for(int i=0;i<4;i++){
	  pos >> data[i];
	}
	//separar los puntos alejados del objetivo
	std::vector< std::pair<float, float> > auxObj;
	std::pair<float, float> auxP = std::make_pair(data[0], data[1]);
	for (std::vector< std::pair<float, float> >::iterator itp = initPosition.begin(); itp != initPosition.end(); ++itp)
	{
		float dx= auxP.first - itp->first;
		float dy= auxP.second - itp->second;
		float dist = sqrt(pow(dx,2.0)+pow(dy,2.0));
		if (dist>13.0) {
			auxObj.push_back(*itp);
		}
	}
	//elegir uno aleatorio
	int randnro = rand()% auxObj.size();
	cout << "OBJETIVO:(" << auxObj[randnro].first << "," << auxObj[randnro].second << ")" << endl;
	//Convertir a coordenadas odometricas la posicion final
	float angCoordOdom = data[3]*PI/180; //en el que esta orientado el robot
	//Traslacion de centro del sist de coordenadas
	std::pair<float, float> auxIntermedio = std::make_pair(auxObj[randnro].first-data[0], auxObj[randnro].second-data[1]);
	//Rotación del sist de coordenadas
	auxP.first = cos(angCoordOdom) * auxIntermedio.first + sin(angCoordOdom) * auxIntermedio.second;
	auxP.second = cos(angCoordOdom) * auxIntermedio.second - sin(angCoordOdom) * auxIntermedio.first;
	return auxP;
}

//genera el archivo .world para una nueva simulacion
void newSession(string strNewS, string strNewPose,string command)
{
	system("killall stageros &");
	sleep(1);

	//las strings que hay que reemplazar
	string strReplaceS = "bitmap";
	string strReplaceP = "robotPose";

	string strNewP = "pose [";
	string strNewPend = "]";
	strNewP.insert( strNewP.end(), strNewPose.begin(), strNewPose.end() );
	strNewP.insert( strNewP.end(), strNewPend.begin(), strNewPend.end() );

	//Abrimos el archivo base y el que se usará para la simulacion
	std::ifstream filein("./src/steering_behaviors_controller/world/setBase.world"); //File to read from
	std::ofstream fileout("./src/steering_behaviors_controller/world/set.world"); //Temporary file
	if(!filein || !fileout)
	{
		cout << "Error opening files!" << endl;
	} else {
		//buscamos las lineas a reemplazar (mapa y posicion) y las ocupamos con un mapa y una posicion aleatoria (x,y, orientacion)
		string strTemp;
		while(std::getline(filein, strTemp))
		{
			if(strTemp == strReplaceS){
				strTemp = strNewS;
			} else if (strTemp == strReplaceP){
				strTemp = strNewP;
			}
			strTemp += "\n";
			fileout << strTemp;
		}
		system(command.c_str());
		filein.close();
		fileout.close();
	}
}

void printEndStats(qlearningStats stat){
	cout << "qvalAcumulado " << stat.qvalTotal << endl;
	cout << "DistanciaRecorrida " << stat.distanciaRecorrida << endl;
	cout << "Tpo (secs): " << stat.tiempo << endl;
	cout << "minimos: ";
	for (std::vector< float >::iterator ita = stat.mins.begin(); ita < stat.mins.end(); ++ita)
	{
		cout << *ita << " ";
	}
	cout << endl;

	cout << "maximos: ";
	for (std::vector< float >::iterator ita = stat.maxs.begin(); ita < stat.maxs.end(); ++ita)
	{
		cout << *ita << " ";
	}
	cout << endl;
}

void statsToFile(std::vector<qlearningStats> stats, int rondas,std::vector< std::pair< std::string , int > > refuerzos)
{
	//inicializo
	qlearningStats promedio;
	promedio.distanciaRecorrida = 0;
	promedio.tiempo = 0;
	for (int i = 0; i < stats[0].mins.size(); i++) {
		promedio.mins.push_back(0.0);
		promedio.maxs.push_back(0.0);
	}
	promedio.qvalTotal = 0;
	//cargo
	int divPromedio = stats.size();
	for (int i = 0; i < stats.size(); i++) {
		if (stats[i].tiempo>120) {
			divPromedio--;
		}
	}
	for (int i = 0; i < stats.size(); i++) {
		if (!(stats[i].tiempo>120)) {
			promedio.distanciaRecorrida += stats[i].distanciaRecorrida / divPromedio;
			promedio.tiempo += stats[i].tiempo / divPromedio;
			for (int j = 0; j < stats[0].mins.size(); j++) {
				promedio.mins[j] += stats[i].mins[j] / divPromedio;
				promedio.maxs[j] += stats[i].maxs[j] / divPromedio;
			}
			promedio.qvalTotal += stats[i].qvalTotal / divPromedio;
		}
	}
	//imprimo
	std::ofstream outputFile;
	string filename = "/home/nahuel/catkin_ws/src/steering_behaviors_controller/stats.csv";
	outputFile.open(filename.c_str(), std::ofstream::app);
	cout << endl << "Resultado luego de " << rondas << " rondas" << endl;
	outputFile << rondas;
	outputFile << ";" << promedio.qvalTotal;
	outputFile << ";" << promedio.distanciaRecorrida;
	outputFile << ";" << promedio.tiempo;
	for (int j = 0; j < promedio.mins.size(); j++) {
		outputFile << ";" << promedio.mins[j];
	}
	for (int j = 0; j < promedio.maxs.size(); j++) {
		outputFile << ";" << promedio.maxs[j];
	}
	for (int j = 0; j < refuerzos.size(); j++) {
		outputFile << ";" << refuerzos[j].second;
	}
	outputFile << endl;

	outputFile.close();
	printEndStats(promedio);
	cout << endl;
}


int main(int argc, char **argv)
{
	//Inicializa el nodo de ros
	ros::init(argc, argv, "controllersHandler");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	//Simulation variables
	std::vector<string> sets;
	std::vector<string> robotPose;
	std::vector< std::pair<float, float> > initPosition;
	std::pair<float, float> auxPair;
	std::vector< std::pair< std::string , int > > refuerzos;


	//Simulation parameters
	//Paths de los mapas para las simulaciones
	sets.push_back("bitmap \"setD.png\"");
	sets.push_back("bitmap \"setE.png\"");

	// posiciones iniciales xyz del robot en el mapa
	std::vector<string> robotPoseAux;
	robotPoseAux.push_back("-6.25 -6.25 0 ");
	robotPoseAux.push_back("-1.25 -6.25 0 ");
	robotPoseAux.push_back("6.25 -6.25 0 ");
	robotPoseAux.push_back("6.25 -1.25 0 ");
	robotPoseAux.push_back("6.25 6.25 0 ");
	robotPoseAux.push_back("1.25 6.25 0 ");
	robotPoseAux.push_back("-6.25 6.25 0 ");
	robotPoseAux.push_back("-6.25 1.25 0 ");

	//el vector de posiciones se almacena en el mismo orden!
	initPosition.push_back(std::make_pair(-6.25, -6.25));
	initPosition.push_back(std::make_pair(-1.25, -6.25));
	initPosition.push_back(std::make_pair(6.25 , -6.25));
	initPosition.push_back(std::make_pair(6.25 , -1.25));
	initPosition.push_back(std::make_pair(6.25 , 6.25));
	initPosition.push_back(std::make_pair(1.25 , 6.25));
	initPosition.push_back(std::make_pair(-6.25, 6.25));
	initPosition.push_back(std::make_pair(-6.25, 1.25));

	// orientaciones iniciales en grados del robot en el mapa
	std::vector<string> robotOrientationAux;
	robotOrientationAux.push_back("0");
	robotOrientationAux.push_back("45");
	robotOrientationAux.push_back("90");
	robotOrientationAux.push_back("135");
	robotOrientationAux.push_back("180");
	robotOrientationAux.push_back("225");
	robotOrientationAux.push_back("270");
	robotOrientationAux.push_back("315");

	//combino las posiciones xyz y orientaciones, generando el vector robotPose con todas las posibilidades de puntos iniciales del robot
	for (std::vector< std::string >::iterator itp = robotPoseAux.begin(); itp != robotPoseAux.end(); ++itp)
	{
		for (std::vector< std::string >::iterator ito = robotOrientationAux.begin(); ito != robotOrientationAux.end(); ++ito)
		{
			std::string auxStrP = *itp;
			std::string auxStrO = *ito;
			auxStrP.insert( auxStrP.end(), auxStrO.begin(), auxStrO.end() );
			robotPose.push_back(auxStrP);
		}
	}

	// Instanciar la clase factory que genera los bh para la clase agente segun el archivo de conf
	Factory* factoryPtr;
	factoryPtr = new Factory(); //pasar direccion de un archivo de conf?

	//eleccion aleatoria del mapa
	int randnroS = rand()% sets.size();
	//eleccion aleatoria de posicion
	int randnroP = rand()% robotPose.size();
	std::string auxstring = factoryPtr->getCommand();
	newSession(sets[randnroS], robotPose[randnroP], auxstring);

	sleep(1);

	//sets experimento
	Setting* configurationPtr = factoryPtr->getExperimentSetting();
	int roundCounter = 1;
	int rondasAprendizaje;
	int rondasTest;
	int ciclos=0;
	std::vector<qlearningStats> stats;
	int nbExp = (*configurationPtr)["nbExp"];
	Agent* agent;
	if (nbExp == 0) {
		agent = new AgentReactive(0,"blendConstante",factoryPtr);  //agente que toma pesos constantes para el blend
	}else if (nbExp == 1) {
		rondasAprendizaje = (*configurationPtr)["rondasAprendizaje"];
		rondasTest = (*configurationPtr)["rondasTest"];
		int freshStart = (*configurationPtr)["fresh"];
		if (freshStart==0) {
			agent = new AgentQLTraining(0,"qlInit",factoryPtr); //agente que entrena la qtable
		} else {
			agent = new AgentQLTraining(0,"qlLoad",factoryPtr);
		}
	}
	auxPair = calcObjective(robotPose[randnroP], initPosition);
	agent->setNewObjective(auxPair);
	//rutina de trabajo
	while(ros::ok())
	{
		// system("clear"); //limpia la consola
		//actualizar cada controlador, analizar el entorno por cada behavior, sumar, ponderar y actualizar la actuacion
		
		int flag = agent->update();

		if (flag == 0) {
			//se termino el ejercicio
			//siguiente
			if (nbExp == 0) {
				cout << "Round: " << roundCounter << endl;
			} else if (nbExp == 1) {
				cout << "Round: " << (roundCounter+((rondasAprendizaje+rondasTest)*ciclos)) << endl;
			}
			//eleccion aleatoria del mapa
			randnroS = rand()% sets.size();
			//eleccion aleatoria de posicion
			randnroP = rand()% robotPose.size();
			std::string auxstring = factoryPtr->getCommand();
			newSession(sets[randnroS], robotPose[randnroP], auxstring);

			if (nbExp == 1) {
				//print stats
				qlearningStats finalStats;
				if (roundCounter <= rondasAprendizaje) {
					std::vector< std::pair< std::string , int > > auxrefs = agent->getRefsAcumulados();
					if(refuerzos.empty()){
						refuerzos = auxrefs;
					} else {
						for (std::vector< std::pair< std::string , int > >::iterator itr = auxrefs.begin(); itr != auxrefs.end(); ++itr)
						{
							for (std::vector< std::pair< std::string , int > >::iterator itb = refuerzos.begin(); itb != refuerzos.end(); ++itb)
							{
								if (itb->first == itr->first) {
									itb->second += itr->second;
									break;
								}
							}
						}
					}
					finalStats = agent->getStats();
					printEndStats(finalStats);
				} else if (roundCounter > rondasAprendizaje) {
					finalStats = agent->getStats();
					printEndStats(finalStats);
					stats.push_back(finalStats);
				}


				//si hay q cambiar de control de agente
				if (roundCounter == rondasAprendizaje) {
					free(agent);
					agent = new AgentQLTraining(0,"qlTest",factoryPtr); //agente que prueba la qtable
					stats.clear();
				}else if (roundCounter == (rondasAprendizaje+rondasTest)) {
					ciclos++;
					statsToFile(stats,(rondasAprendizaje*ciclos),refuerzos); //promedia los resultados y los imprime en un csv
					free(agent);
					agent = new AgentQLTraining(0,"qlLoad",factoryPtr); //agente que entrena la qtable
					roundCounter = -1;
				}
				sleep(2);
			}
			//siguiente round
			roundCounter++;
			cout << endl;
			//set objetivo aleatorio
			auxPair = calcObjective(robotPose[randnroP], initPosition);
			agent->setNewObjective(auxPair);
		}
		ros::spinOnce();
		loop_rate.sleep(); //sleep por el resto del ciclo
	}

	return 0;
}

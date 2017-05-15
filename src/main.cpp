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
#include "../include/Factory.h"

#include <stdio.h>
#include <stdlib.h> //for system use
#include <string>

// unsigned int getNumberOfRobots()
// {
// 	char robotsChar[10];
//
// 	FILE* fp;
//
// 	/*Open the commando for reading*/
// 	fp = popen("/opt/ros/indigo/bin/rostopic list -s | /bin/grep -c 'cmd_vel'","r");
//
// 	/*Read the output*/
// 	fgets(robotsChar, sizeof(robotsChar), fp);
//
// 	return atoi(robotsChar);
// }

std::pair<float, float> calcObjective(string strPos, std::vector< std::pair<float, float> > initPosition){
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
		if (dist>12.0) {
			auxObj.push_back(*itp);
		}
	}
	//elegir uno aleatorio
	int randnro = rand()% auxObj.size();
	cout << "NEW OBJ:(" << auxObj[randnro].first << "," << auxObj[randnro].second << ")" << endl;
	//Convertir a coordenadas odometricas la posicion final
	float angCoordOdom = data[3]*PI/180; //en el que esta orientado el robot
	//Traslacion de centro del sist de coordenadas
	std::pair<float, float> auxIntermedio = std::make_pair(auxObj[randnro].first-data[0], auxObj[randnro].second-data[1]);
	//Rotación del sist de coordenadas
	auxP.first = cos(angCoordOdom) * auxIntermedio.first + sin(angCoordOdom) * auxIntermedio.second;
	auxP.second = cos(angCoordOdom) * auxIntermedio.second - sin(angCoordOdom) * auxIntermedio.first;
	return auxP;
}

void newSession(string strNewS, string strNewPose){
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
		system("rosrun stage_ros stageros /home/nahuel/catkin_ws/src/steering_behaviors_controller/world/set.world &");
		filein.close();
		fileout.close();
	}
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

	//Simulation parameters
	sets.push_back("bitmap \"setD.png\"");
	sets.push_back("bitmap \"setE.png\"");

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

	std::vector<string> robotOrientationAux;
	robotOrientationAux.push_back("0");
	robotOrientationAux.push_back("45");
	robotOrientationAux.push_back("90");
	robotOrientationAux.push_back("135");
	robotOrientationAux.push_back("180");
	robotOrientationAux.push_back("225");
	robotOrientationAux.push_back("270");
	robotOrientationAux.push_back("315");

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

	newSession(sets[randnroS], robotPose[randnroP]);

	sleep(1);

	if (factoryPtr->getAgents()==1)	//Si los hay definicion para todos los agentes en el simulador
	{
		//controlador para el robot
		Agent* agent = new Agent(0,factoryPtr);
		auxPair = calcObjective(robotPose[randnroP], initPosition);
		agent->setNewObjective(auxPair);
		int flag;
		int roundCounter = 0;
		//rutina de trabajo
		while(ros::ok())
		{
			// system("clear"); //limpia la consola
			//actualizar cada controlador, analizar el entorno por cada behavior, sumar, ponderar y actualizar la actuacion
			flag = agent->update();
			if (flag == 1) {
				//eleccion aleatoria del mapa
				randnroS = rand()% sets.size();
				//eleccion aleatoria de posicion
				randnroP = rand()% robotPose.size();
				newSession(sets[randnroS], robotPose[randnroP]);
				roundCounter++;
				cout << "Round: " << roundCounter << endl;
				//set objetivo aleatorio
				auxPair = calcObjective(robotPose[randnroP], initPosition);
				agent->setNewObjective(auxPair);
			}
			ros::spinOnce();
			loop_rate.sleep(); //sleep por el resto del ciclo
		}
	}
	else{
		cout << "Faltan/Sobran configuraciones para algunos robots." << endl;
	}
	return 0;
}

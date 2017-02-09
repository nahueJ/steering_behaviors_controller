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

unsigned int getNumberOfRobots()
{
	char robotsChar[10];

	FILE* fp;

	/*Open the commando for reading*/
	fp = popen("/opt/ros/indigo/bin/rostopic list -s | /bin/grep -c 'cmd_vel'","r");

	/*Read the output*/
	fgets(robotsChar, sizeof(robotsChar), fp);

	return atoi(robotsChar);
}

int main(int argc, char **argv)
{
	//Iniciar el simulador
	system("rosrun stage_ros stageros /home/nahuel/catkin_ws/src/steering_behaviors_controller/world/willow-four-erratics.world &");

	sleep(1);

	//Inicializa el nodo de ros
	ros::init(argc, argv, "controllersHandler");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	// Instanciar la clase factory que genera los bh para la clase agente segun el archivo de conf
	Factory* factoryPtr;
	factoryPtr = new Factory(); //pasar direccion de un archivo de conf?

	int robots = getNumberOfRobots();

	if (robots == factoryPtr->getAgents())	//Si los hay definicion para todos los agentes en el simulador
	{
		//controlador para cada robot
		Agent* agents[robots];
		// instanciar los controladores
		for (int i = 0; i < robots; ++i)
		{
			//intanciar el control para cada robot.
			agents[i] = new Agent(i,factoryPtr);
		}

		//rutina de trabajo
		while(ros::ok())
		{
			// system("clear"); //limpia la consola
			//actualizar cada controlador, analizar el entorno por cada behavior, sumar, ponderar y actualizar la actuacion
			for (int i = 0; i < robots; ++i)
			{
				agents[i]->update();
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

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
#include <stdlib.h>
#include <string>
#include <libconfig.h++>
using namespace libconfig;

#define CONFIGFILE "./src/steering_behaviors_controller/simulation.cfg"

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

	//Inicializa el nodo de ros

	ros::init(argc, argv, "controllersHandler");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	unsigned int robots;

	robots = getNumberOfRobots();

	// Config cfg;

	// // Read the file. If there is an error, report it and exit.

	// cfg.readFile(CONFIGFILE);
	// cout << "kinda succes" << endl;
	
	Factory* factoryPtr;
	factoryPtr = Factory::instance(1);

	//controlador para cada robot
	Agent* agents[robots];
	// instanciar los controladores
	for (int i = 0; i < robots; ++i)
	{
		//intanciar el control para cada robot.
		agents[i] = new Agent(i);
	}

	//rutina de trabajo

	while(ros::ok())
	{
		system("clear"); //limpia la consola
		//actualizar cada controlador, analizar el entorno por cada behavior, sumar, ponderar y actualizar la actuacion
		for (int i = 0; i < robots; ++i)
		{
			agents[i]->update();
		}
		ros::spinOnce();
		loop_rate.sleep(); //sleep por el resto del ciclo
	}
	return 0;
}
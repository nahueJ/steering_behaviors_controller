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
#include "../include/Controller.h"
#include <stdio.h>
#include <stdlib.h>

//solo para el test
#include <geometry_msgs/Twist.h>

#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <string>

unsigned int getNumberOfRobots();

int main(int argc, char **argv)
{

	//Inicializa el nodo de ros

	ros::init(argc, argv, "controllersHandler");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	unsigned int robots;

	robots = getNumberOfRobots();

	/*	TODO

		//comportamientos que quiero activar para cada robot
		int behaviors[robots]; //representar en binario

		//leer de un archivo de configuracion los comportamientos para cada robot
		//por el momento los controladores instancian los mismos comportamientos para todos
		//definir los comportamientos para los robots
		for (int i = 0; i < robots; ++i)
		{
			behaviors[i] = fgets (...); //para evaluar en binario, se podria tomar de los argv
		}
	*/

	/*	TODO
		//instanciar Factory
	*/

	//controlador para cada robot
	Controller* ctrls[robots];
	// instanciar los controladores
	for (int i = 0; i < robots; ++i)
	{
		//intanciar el control para cada robot.
		ctrls[i] = new Controller(i);
		//ctrls[i] = Controller(i,behaviors[i], FactoryPtr);  Arg: i para saber en que topic publicar y behaviors que representa los comport a activar en el ctrl y ptrFactory, puntero de la fabrica de behaviors
	}

	//rutina de trabajo

	while(ros::ok())
	{
		//actualizar cada controlador, analizar el entorno por cada behavior, sumar, ponderar y actualizar la actuacion
		for (int i = 0; i < robots; ++i)
		{
			ctrls[i]->update();
		}
		ros::spinOnce();
		loop_rate.sleep(); //sleep por el resto del ciclo
	}
	return 0;
}

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
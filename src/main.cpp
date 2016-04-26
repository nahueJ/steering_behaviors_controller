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
#include "headers/controller.h"

int main(int argc, char const *argv[])
{
	int robots;
	
	//obtener la cantidad de robots
	robots = 4; //=algo con rostopic list o enviarlo por los arg con el nro de robot o con que intervalo de robots

	//comportamientos que quiero activar para cada robot
	int behaviors[robots]; //representar en binario

	//controlador para cada robot
	controller *ctrls[robots];

	//definir los comportamientos para los robots
	for (int i = 0; i < robots; ++i)
	{
		behaviors[i] = 15; //para evaluar en binario, se podria tomar de los argv
	}

	//instanciar los controladores
	for (int i = 0; i < robots; ++i)
	{
		ctrls[i] = controller(i,behaviors); //intanciar el control para cada robot. Arg: i para saber en que topic publicar y behaviors que representa los comport a activar en el ctrl
	}

	//rutina de trabajo
	while(ros::ok()){
		//actualizar cada controlador, analizar el entorno por cada behavior, sumar, ponderar y actualizar la actuacion
		for (int i = 0; i < robots; ++i)
		{
			ctrls[i].actualize();
		}

		ros::spinOnce();
		loop_rate.sleep(); //sleep por el resto del ciclo
	}

	return 0;
}
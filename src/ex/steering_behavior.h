#ifndef steering_behavior_H
#define steering_behavior_H

//------------------------------------------------------------------------
//
//  Name:   steering_behavior.h
//
//  Desc:   Clase base para los diferentes comportamientos
//
//  Author: Nahuel Jose 2016 (nahuelj91@gmail.com)
//
//------------------------------------------------------------------------

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include //el tipo de dato del sensor
#include //el tipo de dato de posicion del robot pose creo

class SteeringBehavior
{
	private:

		int robotId;	//para saber a los topics de los sensores de qué robot suscribirse
		float linear;	//velocidad
		float angular;	//direccion
		float weight;	//ponderacion o peso para el aporte del comportamiento

	public:
	
		SteeringBehavior();			//constructor

		~SteeringBehavior();		//destructor

		void	update();			//actualizar los valores de linear y angular

		Twist 	getForce();			//obtener el siguiente linear y angular 

		void 	setWeight(float w);	//set la variable weight

		float	getWeight();		//obtener el valor de la variable weight

};

#endif
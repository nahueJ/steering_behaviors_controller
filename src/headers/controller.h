#ifndef controller_H
#define controller_H

//------------------------------------------------------------------------
//
//  Name:   controller.h
//
//  Desc:   Recaba los aportes de los comportamientos, calcula la resultante
// 			y actúa sobre el agente
//
//  Author: Nahuel Jose 2016 (nahuelj91@gmail.com)
//
//------------------------------------------------------------------------

#include "../steering_behavior.h"
#include "geometry_msgs/Twist.h"

class Controller
{
	private:

		int numBehaviors;
		steering_behavior* behaviors[numBehaviors];
		int robotId;
		float linear;
		float angular;
	
	public:
	
		Controller(	int 	id,		//identificador del robot
					int 	b);		//b(ehaviors) comportamientos que se van a activar

		~Controller();

		void	update();

};

#endif
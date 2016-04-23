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

class SteeringBehavior
{
	private:

		int robotId;	//para saber a los topics de los sensores de qué robot suscribirse
	
	public:
	
		SteeringBehavior();

		~SteeringBehavior();

		void	Update();

};



#endif
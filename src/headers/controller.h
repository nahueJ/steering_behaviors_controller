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

class Controller
{
	private:

		steering_behavior* behaviors[];
	
	public:
	
		Controller(	int 	id,				//identificador del robot
					int 	behaviors);		//comportamientos que se van a activar

		~Controller();

		void	Update();

};

#endif
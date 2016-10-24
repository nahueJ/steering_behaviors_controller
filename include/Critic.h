/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _CRITIC_H
#define _CRITIC_H

#include "SteeringBehavior.h"
#include "ObstacleAvoidance.h"
#include "Seek.h"

#include <vector>
#include <string>

#include <libconfig.h++>
using namespace libconfig;

class Critic{

	public: 
		
		Critic(unsigned int id, Setting* configurationPtr, std::vector<SteeringBehavior*>* behaviorsPtr);
		
		~Critic();

		std::vector<float> update();

		int addW(std::string type, float value); 

	private: 
		std::vector<std::vector<float> > weights;
		std::vector<SteeringBehavior*> behaviors;
		int pesos;
		int coeficientes;

		Setting* sets;

		std::vector<float> fillCoefsInit(int b);

		void showCoefsW();
		void updateCoefsW();
};

#endif //_CRITIC_H
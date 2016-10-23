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

		float update(int wid,std::string type);

		int addW(std::string type, float value); 

	private: 
		std::vector<std::vector<float> > weights;
		std::vector<SteeringBehavior*> behaviors;
		int pesos;
		int coeficientes;

		int seekVbles(float* a, float* b);
		int obstacleAvoidanceVbles(float* a, float* b);
		int fleeVbles(float* a, float* b);

		int getIndex(std::string type);
};

#endif //_CRITIC_H
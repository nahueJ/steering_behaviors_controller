/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#include "../include/Weights.h"


Weights::Weights(std::vector<float> w, Setting* configurationPtr)
{
	myType = (*configurationPtr)["type"].c_str();
	weights = new std::vector<float>;
	*weights = w;
	ceroRules.clear();
	int nbRules = (*configurationPtr)["ceroRules"].getLength();
	if (nbRules>0)
	{
		Setting& rules =(*configurationPtr)["ceroRules"];
		for (int i = 0; i < nbRules; ++i)
		{
			ceroRuleStruct auxRule;
			auxRule.behaviorNb = rules[i][0];
			auxRule.ceroOver = rules[i][1];
			ceroRules.push_back(auxRule);
		}
	}
}

Weights::Weights(std::vector< std::vector<float> > v, Setting* configurationPtr){
	myType = (*configurationPtr)["type"].c_str();
}


Weights::~Weights(){

}

std::vector<float> Weights::getWeights(std::vector< std::vector<float> > state){
	if (myType == "constW")
	{
		if (!ceroRules.empty())
		{
			return checkCeroRules(state);
		}
	}
	return *weights;
}

std::vector<float> Weights::checkCeroRules(std::vector< std::vector<float> > state){
	std::vector<float> wAux = (*weights);
	float amountToDistribute = 0;
	int wToDistribute = 0;
	for (std::vector<ceroRuleStruct>::iterator irule = ceroRules.begin(); irule != ceroRules.end(); ++irule)
	{
		bool flag = true;
		for (std::vector<float>::iterator istate = (state[(*irule).behaviorNb]).begin(); istate != (state[(*irule).behaviorNb]).end(); ++istate)
		{
			if (*istate < (*irule).ceroOver)
			{
				//si se rompe la regla
				flag = false;
			}
		}
		//si la regla nunca se rompio
		if (flag)
		{
			amountToDistribute = wAux[(*irule).behaviorNb];
			wAux[(*irule).behaviorNb] = 0.000;
			wToDistribute++;
		}
	}
	//si se cumplio alguna regla para obviar un comportamiento, distribuyo el peso en los demas comportamientos
	if (wToDistribute > 0)
	{
		float added = amountToDistribute/(state.size()-wToDistribute);
		for (std::vector<float>::iterator i = wAux.begin(); i != wAux.end(); ++i)
		{
			if (*i != 0.000)
			{
				*i += added;
			}
		}
	}
	return wAux;
}

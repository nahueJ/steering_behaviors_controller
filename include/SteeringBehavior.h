/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _STEERINGBEHAVIOR_H
#define _STEERINGBEHAVIOR_H

#include "ros/ros.h"

#include <libconfig.h++>
using namespace libconfig;

#include <string>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;
using std::string;


//Behavior List //YO PENSABA Q TIENEN QUE ESTAR EN LOS INCLUDES, PERO NO
//#include "Seek.h"
//#include "Arrive.h"
//#include "WallAvoidance.h"

//#include <geometry_msgs/Twist.h>
 
class SteeringBehavior {
public: 

	SteeringBehavior(std::string myName, unsigned int id, std::string pre, Config* configurationPtr);
	
	~SteeringBehavior();
	
	virtual void update() ; // función virtual pura
	
	virtual string getName();
	virtual float getDesiredV();
	virtual float getDesiredW(); 
	
	void setDesiredV(float y);
	void setDesiredW(float z);

protected: 

	string myName;
	unsigned int robotId;

	std::string pretopicname;

	float desiredV;
	float desiredW;
	
	//Clase para el ingreso de parametros de configuración
	Config* config;
};

#endif //_STEERINGBEHAVIOR_H
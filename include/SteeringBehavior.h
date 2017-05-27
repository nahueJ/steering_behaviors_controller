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
#include <vector>
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

	SteeringBehavior(unsigned int id, std::string pre, Setting* configurationPtr);

	~SteeringBehavior();

	virtual int update() ; // función virtual pura
	virtual float getState() ;
	virtual void updateState() ;

	virtual string getName();
	virtual string getType();
	virtual float getDesiredV();
	virtual float getDesiredW();
	virtual void setGoal(float, float); //only for seek

	void setDesiredV(float y);
	void setDesiredW(float z);

protected:

	string myName;
	std::string myType;
	unsigned int robotId;

	float stateDiscrete;
	float stateContinuous;
	std::vector<float> valoresEstado;

	std::string pretopicname;

	float desiredV;
	float desiredW;

	//Clase para el ingreso de parametros de configuración
	Setting* config;

	//variables para almacenar los datos del odometro
	float x;
	float y;
	float tita;

	void discretizarEstado ();
};

#endif //_STEERINGBEHAVIOR_H

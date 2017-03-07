/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */

#ifndef _OBSTACLEAVOIDANCE_H
#define _OBSTACLEAVOIDANCE_H

#include "ros/ros.h"
#include "ros/message.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include "SteeringBehavior.h"

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <libconfig.h++>
using namespace libconfig;

#define PI 3.14159265

class ObstacleAvoidance : public SteeringBehavior {

	public:

		ObstacleAvoidance(unsigned int id, std::string pre, Setting* configurationPtr);

		~ObstacleAvoidance();

		/**
		 * gets the last data and actualizes the desiredTwist
		 */
		virtual void update();
		virtual std::vector<float> getState() ;
		virtual void updateState() ;

		virtual float getDesiredW();

	private:

		//Variables para suscribirse a un topic
		ros::NodeHandle* rosNode;

		//Funcion de Callback y variables para la suscripcion al topic del laser
		ros::Subscriber* sensorSubscriber;
		void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure);
		//Array para almacenar los valores recibidos de las funciones de callback para el posterior calculo con las mismas
		float* laser;
		//Variables inherentes al sensor laser
		int haz;					//cantidad de haces del laser
		int sectores;				//cantidad de sectores en que se discretiza el barrido del laser
		std::vector<float> zona;	//vector en el que se carga los valores de cada seccion discreta
		int div;					//cantidad de haces por sector
		float prescicion;			//separacion entre mediciones del laser, en angulos, es decir, la prescicion del sensor
		float abanico;				//angulo total barrido por el sensor

		//Funcion de Callback y variables para la suscripcion al topic del odometro (posicion y twist)
		ros::Subscriber* odomSubscriber;
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
		nav_msgs::Odometry*	myData;
		//variables para almacenar los datos del odometro
		float x;
		float y;
		float tita;

		//distancia máxima a la que actua el comportamiento
		float distMax;
		float distMin;
		float closestObstacle;

		//Funciones privadas
		float estimateLasers(float xnext, float ynext);
		float nextDist( float dist, float angulo);
		int updateZona(int* min);
		float relativeToAbsolute(int relativeIndex);
		float escapeTo(int minArea);

		void updateAct();
		void updateFut();
		float emergencia();
		int zonaSafe();
		void printZona();
};

#endif //_OBSTACLEAVOIDANCE_H

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


class ObstacleAvoidance : public SteeringBehavior {

	private: 

		//Cantidad de lasers del robot
		unsigned int nroLasers;

		//Variables para suscribirse a un topic
		ros::NodeHandle* rosNode;
		ros::Subscriber* odomSubscriber;
		std::vector<ros::Subscriber*> sensorSubscriber;

		geometry_msgs::Twist myTwist;

		//Funcion para obtener la cantidad de lasers que posee el robot
		unsigned int getNumberOfLasers(unsigned int id);
		//funcion que devuelve el menor elementro del arreglo del primer parametro
		float calcMin(float matrix[],int* index);
		//funcion para sumarle al primer parametro (orientación) una cantidad de grados dada por el segundo parametro hacia izq o derecha (1 o 0) dada por el tercer parametro
		float addW(float,float,int);
		//funcion para definir cual obstaculo es mas prioritario o si no hay ninguno prioritario
		int minIndex(float,float,float);

		//Funciones de Callback para las suscripciones a los topic del laser y del odometro (posicion y twist)
		void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

		//variables para almacenar los valores recibidos de las funciones de callback para el posterior calculo con las mismas
		int haz;
		float* laserCentral;
		float* laserIzquierda;
		float* laserDerecha;

		float minCentral;
		float minIzquierda;
		float minDerecha;

		int minCentralIndex;
		int minIzquierdaIndex;
		int minDerechaIndex;
		
		nav_msgs::Odometry*	myData;

		//distancia máxima a la que actua el comportamiento
		float distMax;
		float distMin;

	public: 
		
		/**
		 * @param mySensor
		 * @param unsigned int id
		 */
		ObstacleAvoidance(unsigned int id, std::string pre);
		
		~ObstacleAvoidance();

		/**
		 * gets the last data and actualizes the desiredTwist
		 */
		virtual void update();

		virtual float getDesiredW(); 

};

#endif //_OBSTACLEAVOIDANCE_H
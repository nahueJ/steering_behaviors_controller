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

	public: 
		
		ObstacleAvoidance(unsigned int id, std::string pre, config_t* configurationPtr);
		
		~ObstacleAvoidance();

		/**
		 * gets the last data and actualizes the desiredTwist
		 */
		virtual void update();

		virtual float getDesiredW(); 

	private: 

		//Variables para suscribirse a un topic
		ros::NodeHandle* rosNode;
		ros::Subscriber* odomSubscriber;
		ros::Subscriber* sensorSubscriber;

		geometry_msgs::Twist myTwist;

		//Funciones de Callback para las suscripciones a los topic del laser y del odometro (posicion y twist)
		void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

		//cantidad de haces del laser
		int haz;
		//variables para almacenar los valores recibidos de las funciones de callback para el posterior calculo con las mismas
		float* laser;
		
		nav_msgs::Odometry*	myData;
		//variables para almacenar los datos del odometro
		float x;
		float y;
		float tita;

		//distancia máxima a la que actua el comportamiento
		float distMax;
		float distMin;

		//Funciones privadas
		float estimateLasers(float xnext, float ynext);
		float nextDist( float dist, float angulo);
		
};

#endif //_OBSTACLEAVOIDANCE_H
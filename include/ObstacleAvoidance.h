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
		void calcMin(float matrix[][3]);

		//Funciones de Callback para las suscripciones a los topic del laser y del odometro (posicion y twist)
		void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure);
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

		//variables para almacenar los valores recibidos de las funciones de callback para el posterior calculo con las mismas
		float* lasers;
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

};

#endif //_WALLAVOIDANCE_H
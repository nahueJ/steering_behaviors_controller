/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#ifndef _WALLAVOIDANCE_H
#define _WALLAVOIDANCE_H

#include "ros/ros.h"
#include "ros/message.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include "SteeringBehavior.h"

#include <string>
#include <sstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;


class WallAvoidance : public SteeringBehavior {

	public: 
		
		/**
		 * @param mySensor
		 * @param unsigned int id
		 */
		WallAvoidance(unsigned int id, unsigned int mySensor);
		
		~WallAvoidance();

		/**
		 * gets the last data and actualizes the desiredTwist
		 */
		virtual void update() const;

	private: 
		//Id del robot del controlador
		unsigned int laserId;

		//Variables para publicar por un topic
		ros::NodeHandle* rosNode;
		ros::Subscriber* ctrlSubscriber;

		geometry_msgs::Twist myTwist;

		void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure);
};

#endif //_WALLAVOIDANCE_H
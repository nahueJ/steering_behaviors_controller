/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "SteeringBehavior.h"

/**
 * Abstract Class SteeringBehavior implementation
 */


/**
 * @param unasigned int id
 * @param float weight
 */
SteeringBehavior::SteeringBehavior(unsigned int id) : robotId( id )
{
	
}

SteeringBehavior::~SteeringBehavior() {

}

geometry_msgs::Twist SteeringBehavior::getDesiredTwist() const
{
	return desiredTwist;
}

void SteeringBehavior::setDesiredTwist( float x,  float z) 
{
	desiredTwist.linear.x = ( x <= 1.0f ) ? x : 1.0f;
	desiredTwist.angular.z = ( abs(z) <= 1.0 ) ? z : (z/abs(z));
}

void SteeringBehavior::update(){

}
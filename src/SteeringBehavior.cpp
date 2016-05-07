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
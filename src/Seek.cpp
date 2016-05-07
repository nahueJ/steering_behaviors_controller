/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "Seek.h"

/**
 * Seek implementation
 * 
 * The seek steering behavior returns a force that directs an agent toward a
 * target position.
 */

/**
 * @param objective
 * @param id
 * @param weight
 */
Seek::Seek(pose objective, unsigned int id, float weight) {

}

Seek::~Seek() {

}

/**
 * gets the last data and actualizes the desiredTwist
 * @param myPose
 */
virtual void Seek::update() const{

}
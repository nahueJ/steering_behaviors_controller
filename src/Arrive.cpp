/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "Arrive.h"

/**
 * Arrive implementation
 * 
 * Seek is useful for getting an agent moving in the right direction, but often
 * you’ll want your agents to come to a gentle halt at the target position, and
 * as you’ve seen, seek is not too great at stopping gracefully. Arrive is a
 * behavior that steers the agent in such a way it decelerates onto the target
 * position.
 */


/**
 * gets the last data and actualizes the desiredTwist
 * @param myPose
 */
void Arrive::update(pose myPose) {

}

/**
 * @param objective
 * @param id
 * @param weight
 */
void Arrive::Arrive(pose objective, unsigned int id, float weight) {

}

void Arrive::~Arrive() {

}
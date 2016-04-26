/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Controller.h"

/**
 * Controller implementation
 */

/*--------------------------- Constructor -----------------------------------
 *	segun la cantidad de comportamientos solicitados en la variable behaviors
 *	inicializa el vector de comportamientos steering_behavior* behaviors[] 
 *	luego instancia un objeto de cada uno de los comportamientos, enviandoles 
 *	una ponderacion por defecto
 *	A continuacion inicializa la conexion por el topic del robot_id
 *
 *
 * @param unsigned int id
 ------------------------------------------------------------------------*/
void Controller::Controller(void unsigned int id) {
	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::init("controller_0");				
	rosNode = new n;						
	rosPublisher = new ctrlr_pub		//instancia un publicador
	ctrlr_pub = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1000); //adhiere el publicador al topic cmd_vel
	rosRate = new loop_rate(10);
}

void Controller::~Controller() {

}

void Controller::update() {

}

/**
 * @param weight
 */
void Controller::seekOn(float weight) {

}

void Controller::seekOff() {

}

/**
 * @param weight
 */
void Controller::fleeOn(float weight) {

}

void Controller::fleeOff() {

}

/**
 * @param weight
 */
void Controller::wanderOn(float weight) {

}

void Controller::wanderOff() {

}

/**
 * @param weight
 */
void Controller::arriveOn(float weight) {

}

void Controller::arriveOff() {

}

/**
 * @param weight
 * @param sensor
 */
void Controller::obstacleAvoidanceOn(float weight, string sensor) {

}

void Controller::obstacleAvoidanceOff() {

}

/**
 * @param weight
 */
void Controller::evadeOn(float weight) {

}

void Controller::evadeOff() {

}

/**
 * @param weight
 */
void Controller::hideOn(float weight) {

}

void Controller::hideOff() {

}

/**
 * @param weight
 * @param sensor
 */
void Controller::wallAvoidanceOn(float weight, string sensor) {

}

void Controller::wallAvoidanceOff() {

}

/**
 * @param weight
 */
void Controller::pathFollowingOn(float weight) {

}

void Controller::pathFollowingOff() {

}

/**
 * @param weight
 */
void Controller::offsetPursuitOn(float weight) {

}

void Controller::offsetPursuitOff() {

}

/**
 * @param weight
 */
void Controller::pursuitOn(float weight) {

}

void Controller::pursuitOff() {

}

/**
 * @param weight
 */
void Controller::interposeOn(float weight) {

}

void Controller::interposeOff() {

}
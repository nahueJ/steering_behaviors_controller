/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "WallAvoidance.h"

/**
 * WallAvoidance implementation
 * 
 * Wall avoidance steers to avoid potential collisions
 * with a wall.
 */

void WallAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure)
{
  //TODO comunicar lo que recibo
	cout << "receiving data" << endl;
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @param mySensor
 * @param unasigned int id
 * @param float weight
 */
WallAvoidance::WallAvoidance(unsigned int id, unsigned int mySensor) : SteeringBehavior(id)
{	
	laserId = mySensor;

	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;

	cout << "Construyendo para robot " << robotId << " laserId " << laserId << endl; 
	//generar el nombre del nodo con el robotId
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;

	remappingsArgs.insert(ros::M_string::value_type( "__master", nameMaster.str()));

	//generar el nombre del nodo con el robotId
	std::stringstream name;
	name << "wallavoidance_" << laserId;

	//inicializa el nodo
	ros::init(remappingsArgs, name.str());

	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	rosNode = new ros::NodeHandle;
	
	//generar el nombre del topic a partir del robotId
	std::stringstream substopic;
	substopic << "/robot_" << robotId << "/base_scan_" << laserId;

	//Crear el suscriptor y apuntarlo con la variable de la clase
	ctrlSubscriber = new ros::Subscriber;
	*ctrlSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(substopic.str(), 1000, &WallAvoidance::sensorCallback,this);
}

WallAvoidance::~WallAvoidance() {
	delete rosNode;
	delete ctrlSubscriber;
}


/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void WallAvoidance::update() const{

}
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
void Seek::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
	update();
}

/**
 * @param objective
 * @param id
 * @param weight
 */
Seek::Seek(geometry_msgs::Pose objective, unsigned int id, std::string pre) : SteeringBehavior(id,pre)
{
	target = objective;
	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;

	//generar el nombre del nodo con el robotId
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;

	remappingsArgs.insert(ros::M_string::value_type( "__master", nameMaster.str()));

	//generar el nombre del nodo con el robotId
	std::stringstream name;
	name << "seek_" << robotId;

	//inicializa el nodo
	ros::init(remappingsArgs, name.str());

	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	rosNode = new ros::NodeHandle;

	/* Subscripcion al topic base_pose_ground_truth de este robot*/
	//generar el nombre del topic a partir del robotId
	std::stringstream topicname;

	topicname << pretopicname << "/odom" ;

	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(topicname.str(), 1000, &Seek::odomCallback,this);

	myData = new nav_msgs::Odometry;
}

Seek::~Seek() {
	delete rosNode;
	delete odomSubscriber;
	delete myData;
}

/**
 * gets the last data and actualizes the desiredTwist
 * @param myPose
 */
void Seek::update() {

	errorx = target.position.x - myData->pose.pose.position.x;
	errory = target.position.y - myData->pose.pose.position.y;

	//calcular la orientacion ideal!
	float wi = wIdeal(errorx,errory);

	// cout << "Received data: " << endl;
	// cout << "x = " << myData->pose.pose.position.x << endl;
	// cout << "y = " << myData->pose.pose.position.y << endl;
	// cout << "w = " << myData->pose.pose.orientation.z << endl ;
	// cout << "Desired data: " << endl;
	// cout << "x = " << target.position.x << endl;
	// cout << "y = " << target.position.y << endl;
	// cout << "w = " << wi << endl ;

	setDesiredW(wi);

}

float Seek::wIdeal( float dx, float dy)
{
	float angulo = (atan2(dy,dx) * 180 / PI) + 180;
	float wIdeal = (2 * angulo / 360) -1;	 //graficar la recta para corroborar que angulo 0->1,90->0.5,180->0,270->-0.5,360->-1
	return wIdeal;
}
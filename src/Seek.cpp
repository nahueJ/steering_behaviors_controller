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
}

/**
 * @param objective
 * @param id
 * @param weight
 */
Seek::Seek(unsigned int id, std::string pre) : SteeringBehavior(std::string("seek"), id, pre)
{
	float vel;
	//Cargar Valores de configuracion 
	// if (config.lookupValue("targetX",	target.position.x)	&&
	//     config.lookupValue("targetY",  target.position.y)	&&
	//     config.lookupValue("desiredV",	vel)				&&
	//     config.lookupValue("toleranceToTarget", toleranceToTarget))
	if(1)
	{
	 
		cout << "Instanciando Seek" << endl;

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

		topicname << pretopicname << "odom" ;

		//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
		odomSubscriber = new ros::Subscriber;
		*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(topicname.str(), 1000, &Seek::odomCallback,this);

		myData = new nav_msgs::Odometry;

		setDesiredV(vel);
	}
	else
	{
	    cout << "SEEK " << robotId << ": Missing parameter in configuration file." << endl;
	}
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

	setDesiredW(wi);

	//verificar distancia al objetivo
	if (toleranceToTarget>sqrt(pow(errorx,2)+pow(errory,2)))
	{
		//si es menor que la tolerancia se detiene
		setDesiredV(0.0);
	}
}

float Seek::getDesiredW() 
{
	update();
	return desiredW;
}

float Seek::wIdeal( float dx, float dy)
{
	float angulo = (atan2(dy,dx) * 180 / PI) + 180;
	float wIdeal = (2 * angulo / 360) -1;	 //graficar la recta para corroborar que angulo 0->1,90->0.5,180->0,270->-0.5,360->-1
	return wIdeal;
}
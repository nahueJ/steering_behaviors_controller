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
void Seek::poseCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
	//cout << myData.pose.pose.position.x << " " << myData.pose.pose.position.y << endl;
	update();
}

/**
 * @param objective
 * @param id
 * @param weight
 */
Seek::Seek(geometry_msgs::Pose objective, unsigned int id) : SteeringBehavior(id)
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
	std::stringstream basetopic;
	basetopic << "/robot_" << robotId << "/base_pose_ground_truth" ;
	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(basetopic.str(), 1000, &Seek::poseCallback,this);

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
	
	dx = target.position.x - myData->pose.pose.position.x;
	dy = target.position.y - myData->pose.pose.position.y;

	//calcular la orientacion ideal!
	float wi = wIdeal(dx,dy);
	dw = wi - myData->pose.pose.orientation.w;
	

	if (dw > 1)
	{	
		//giro hacia la izq z negativo
		dw = 2 - dw;	//es la diferencia de orientacion real
		setDesiredTwist(1-dw,-dw);	//el segundo argumento es negativo para que gire hacia la izquierda
	
	}
	else if (dw==0.0)
	{
		setDesiredTwist(1,0);	//ya esta alineado, solo avanzo
	}
	else
	{
		// giro hacia la derecha z positivo
		setDesiredTwist(1-dw,dw);
	}
	cout << "Update de " << robotId << endl ; 
	cout << "wideal " << wi << " dx, dy, dw " << dx << " " << dy << " " << dw << endl ;


}

float Seek::wIdeal( float dx, float dy)
{
	float angulo = atan2(dy,dx) * 180 / PI;
	cout << angulo << endl ;
	float wIdeal = 1 - (2 * angulo / 360);	 //graficar la recta para corroborar que angulo 0->1,90->0.5,180->0,270->-0.5,360->-1
	cout << wIdeal << endl ;
	return wIdeal;
}
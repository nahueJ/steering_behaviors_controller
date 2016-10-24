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
Seek::Seek(unsigned int id, std::string pre, Setting* configurationPtr) : SteeringBehavior(id, pre, configurationPtr)
{
	//Cargar Valores de configuracion 
	target.position.x = (*configurationPtr)["targetX"];
	target.position.y = (*configurationPtr)["targetY"];
	float vel = (*configurationPtr)["desiredV"];
	toleranceToTarget = (*configurationPtr)["toleranceToTarget"];

	cout << "Instanciando Seek, objetivo (" << target.position.x << "," << target.position.y << ")" << endl;

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

	// Subscripcion al topic base_pose_ground_truth de este robot
	//generar el nombre del topic a partir del robotId
	std::stringstream topicname;

	topicname << pretopicname << "odom" ;

	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(topicname.str(), 1000, &Seek::odomCallback,this);

	myData = new nav_msgs::Odometry;

	setDesiredV(vel);
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
	float almost = sqrt(pow(errorx,2)+pow(errory,2));
	cout << "ALMOST! " << almost << endl;
	if (almost < toleranceToTarget*3)
	{
		setDesiredV(0.1);
	}
	if (toleranceToTarget>almost)
	{
		//si es menor que la tolerancia se detiene
		cout << "OBJETIVO ALCANZADO" << endl;
		setDesiredV(0.0);
		setDesiredW(0.0);
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

void Seek::getVbles(std::vector<float>* v){
	if (variables>0)
	{
		(*v)[0]=getVble1();
		if (variables>1)
		{
			(*v)[1]=getVble2();
			if (variables>1)
			{
				(*v)[2]=getVble3();
			}
		}
	}
}

float Seek::getVble1()
{
	return 0.0;
}
float Seek::getVble2()
{
	return 0.0;
}
float Seek::getVble3()
{
	return 0.0;
}
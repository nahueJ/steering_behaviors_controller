/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Agent.h"

/**
 * Agent implementation
 */

void Agent::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;	//almaceno en la variable correspondiente los ultimos valores recibidos
}

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
Agent::Agent(unsigned int id)
{

	robotId = id;

	//*****************//
	//Creacion del Nodo//
	//*****************//

	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;
	remappingsArgs.insert(ros::M_string::value_type( "__master", "controllerHandler"));
	//generar el nombre del nodo con el robotId
	std::stringstream name;
	name << "controller_" << robotId;
	//inicializa el nodo
	ros::init(remappingsArgs, name.str());
	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	rosNode = new ros::NodeHandle;

	//*************************************//
	//Suscripcion y Publicaciones en Topics//
	//*************************************//

	//generar el nombre del topic a partir del robotId
	pretopicname = new std::stringstream;
	if (!imAlone())
	{
		*pretopicname << "/robot_" << robotId ;
	}
	else
	{
		*pretopicname << "/" ;
	}
	std::stringstream pubtopicname ;
	pubtopicname << pretopicname->str() << "cmd_vel" ;
	//Crear el publicador y apuntarlo con la variable de la clase
	ctrlPublisher = new ros::Publisher;
	*ctrlPublisher = rosNode->advertise<geometry_msgs::Twist>(pubtopicname.str(), 100000);

	std::stringstream sustopicname ;
	sustopicname << pretopicname->str() << "odom" ;

	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(sustopicname.str(), 1000, &Agent::odomCallback,this);
	
	myData = new nav_msgs::Odometry;

	//************************************//
	//Instanciacion de los comportamientos//
	//************************************//

	//INSTANCIAR LOS BEHAVIORS --->>> FACTORY
	
	unsigned int laserId = 0;
	
	// // Test solo Seek
	// geometry_msgs::Pose target;
	// target.position.x = 10;
	// target.position.y = -4;
	// behaviortest = new Seek(target,robotId,pretopicname->str());
	
	// Test solo obstacle avoidance
	behaviortest = new ObstacleAvoidance(robotId,pretopicname->str());
}

Agent::~Agent() {
	delete rosNode;
	delete ctrlPublisher;
	delete odomSubscriber;
	delete myData;
	//LIBERAR LOS BEHAVIORS

}

/*--------------------------- Update -----------------------------------
 *	Hace update a los behavior q corresponden, luego sumar los twists de
 *	cada uno de ellos ponderadamente y comunicar eso a los actuadores del
 *	robot
 ------------------------------------------------------------------------*/

void Agent::update() 
{
	//Velocidad Linear
    myTwist.linear.x = 0.15;
	//Velocidad Angular
	myTwist.angular.z =  behaviortest->getDesiredW() - myData->pose.pose.orientation.z;

	if (myTwist.angular.z > 1.0)
	{	
		//giro hacia la izq z negativo
		myTwist.angular.z = 2.0 - myTwist.angular.z;	//es la diferencia de orientacion real
		myTwist.angular.z = -myTwist.angular.z;	//el segundo argumento es negativo para que gire hacia la izquierda
	}

    ctrlPublisher->publish(myTwist);
}

int Agent::imAlone(){
	char robotsChar[10];

	FILE* fp;

	/*Open the commando for reading*/
	fp = popen("/opt/ros/indigo/bin/rostopic list -s | /bin/grep -c 'cmd_vel'","r");

	/*Read the output*/
	fgets(robotsChar, sizeof(robotsChar), fp);

	if (atoi(robotsChar)>1)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
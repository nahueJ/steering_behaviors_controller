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
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	tf::Pose pose;
	tf::poseMsgToTF(odom->pose.pose, pose);
	tita = tf::getYaw(pose.getRotation());	//tita: orientación en radianes para el marco coordenadas 2D, transformado a partir del marco de referencia 3D expresado por el quaternion (x,y,z,w) de la estructura orientation.
	//a partir de la posición inicial (0rad) tiene un rango (-PI/2 ; +PI/2] siendo el giro positivo hacia la izquierda del vehículo
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
Agent::Agent(unsigned int id, string type, Factory* factoryPtr)
{
	robotId = id;
	myType = type;

	//Pido la configuracion para el tipo de agente
	configurationPtr = factoryPtr->getTypeSetting(myType);

	//Generar prefijo del nombre de los topics del robot del simulador para futuras suscripciones
	pretopicname = new std::stringstream;
	*pretopicname << "/" ;
	//Instancia los comportamientos
	factoryPtr->instanciateBehaviors( robotId, pretopicname->str(), &behaviors, myType);

	//*****************//
	//Creacion del Nodo//
	//*****************//
	ros::M_string remappingsArgs;
	remappingsArgs.insert(ros::M_string::value_type( "agente", "controllerHandler"));
	std::stringstream name;
	name << "controller_" << robotId;
	ros::init(remappingsArgs, name.str());
	rosNode = new ros::NodeHandle;

	//*************************************//
	//Suscripcion y Publicaciones en Topics//
	//*************************************//
	//Crear el publicador y apuntarlo con la variable de la clase
	std::stringstream pubtopicname ;
	pubtopicname << pretopicname->str() << "cmd_vel" ;
	ctrlPublisher = new ros::Publisher;
	*ctrlPublisher = rosNode->advertise<geometry_msgs::Twist>(pubtopicname.str(), 100000);
	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	std::stringstream sustopicname ;
	sustopicname << pretopicname->str() << "odom" ;
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(sustopicname.str(), 1000, &Agent::odomCallback,this);
}

Agent::~Agent()
{
	delete rosNode;
	delete ctrlPublisher;
	delete odomSubscriber;
	//LIBERAR LOS BEHAVIORS

}

/*--------------------------- Update ------------------------------------
 *	Hace update a los behavior q corresponden, luego sumar los twists de
 *	cada uno de ellos ponderadamente y comunicar eso a los actuadores del
 *	robot
 ----------------------------------------------------------------------*/
int Agent::update()
{
	int behaviorFlag[behaviors.size()];
	std::vector<float> behaviorState;
	geometry_msgs::Twist twists[behaviors.size()];
	for (int i = 0; i < behaviors.size(); ++i)
	{
		behaviorFlag[i] = behaviors[i]->update();
		if (behaviors[i]->getType()=="seek")
		{
			switch (behaviorFlag[i]) {
				case 0:	//obj alcanzado
					return 0;
					break;
				case 1: //seek normal
					twists[i].angular.z = behaviors[i]->getDesiredO();
					twists[i].linear.x = behaviors[i]->getDesiredV();
					break;
			}
			behaviorState = behaviors[i]->getState();
		}
		else if (behaviors[i]->getType()=="avoidObstacles")
		{
			switch (behaviorFlag[i]) {
				case 0:
					ctrlPublisher->publish(twists[i-1]);
					cout << "Solo Seek" << endl;
					return 1;
				case 1: //AO normal
					twists[i].angular.z = behaviors[i]->getDesiredO();
					twists[i].linear.x = behaviors[i]->getDesiredV();
					break;
				case -1: //solo AO, peligro de colision
					twists[i].angular.z = behaviors[i]->getDesiredO();
					twists[i].linear.x = behaviors[i]->getDesiredV();
					//ctrlPublisher->publish(twists[i]);
					//cout << "Solo AO" << endl;
					//return 1;
			}
			behaviorState = behaviors[i]->getState();
		}
	}
	/*for (std::vector<float>::iterator itb = behaviorState.begin(); itb != behaviorState.end(); ++itb)
	{
		cout << *itb << " ";
	}*/

	//suma ponderada de los desiresVs y Ws
	//cout << "seek: v=" << twists[0].linear.x << " o=" << twists[0].angular.z*180/PI << endl;
	//cout << "aObs: v=" << twists[1].linear.x << " o=" << twists[1].angular.z*180/PI << endl;

	cout << "Blending" << endl;

	std::vector<float> weights = getWeights(behaviorState);

	float ws = weights[0];
	float wao = weights[1];

	float rx = (ws * twists[0].linear.x * cos(twists[0].angular.z)) + (wao * twists[1].linear.x * cos(twists[1].angular.z));
	float ry = (ws * twists[0].linear.x * sin(twists[0].angular.z)) + (wao * twists[1].linear.x * sin(twists[1].angular.z));

	// cout << rx << "=" << ws << "*" << twists[0].linear.x << "*" << cos(twists[0].angular.z) << "+" << wao << "*" << twists[1].linear.x << "*" << cos(twists[1].angular.z) << endl << ry << "=" << ws << "*"  ;
	// cout << twists[0].linear.x << "*" << sin(twists[0].angular.z) << "+" << wao << "*" << twists[1].linear.x << "*" << sin(twists[1].angular.z) << endl;

	float v = sqrt(pow(rx,2)+pow(ry,2));
	float alpha = atan2(ry,rx);

	cout << "SEEK :  v  " << twists[0].linear.x << " o  " << twists[0].angular.z*180/PI << endl;
	cout << "RESU :  v  " << v << " o  " << alpha*180/PI << endl;
	cout << "AOBS :  v  " << twists[1].linear.x << " o  " << twists[1].angular.z*180/PI << endl;
	cout << endl << rx << " " << ry << endl << endl;

	geometry_msgs::Twist twist;
	twist.angular.z = alpha;
	twist.linear.x = v;
	ctrlPublisher->publish(twist);

	return 1;
}


/*void Agent::updateState()
{
	ansState = state;
	state.clear();
	for (int i = 0; i < behaviors.size(); ++i)
	{
		state.push_back( behaviors[i]->getState() );
	}
}*/

/*void Agent::printState()
{
	cout << "PS: ";
	for (std::vector< float >::iterator ita = state.begin(); ita < state.end(); ++ita)
	{
		cout << *ita << " ";
	}
	// cout << endl;
}*/

void Agent::setNewObjective(std::pair<float, float> auxP)
{
	//buscar el behavior seek y setear el objetivo
	for (std::vector<SteeringBehavior*>::iterator itb = behaviors.begin(); itb != behaviors.end(); ++itb)
	{
		if ((*itb)->getType() =="seek")
		{
			(*itb)->setGoal(auxP.first, auxP.second);
		}
	}
}

std::vector<float> Agent::getWeights(std::vector<float>)
{
}

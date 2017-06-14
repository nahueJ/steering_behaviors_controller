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
	std::vector<geometry_msgs::Twist> twists;
	bool llegada = false;
	bool danger = false;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		geometry_msgs::Twist tw;
		behaviorFlag[i] = behaviors[i]->update();
		if (behaviors[i]->getType()=="seek")
		{
			tw.angular.z = behaviors[i]->getDesiredO();
			tw.linear.x = behaviors[i]->getDesiredV();
			switch (behaviorFlag[i]) {
				case 0:	//obj alcanzado
					llegada = true;//return 0;
					break;
				case 1: //seek normal
					break;
			}
			twists.push_back(tw);

		}
		else if (behaviors[i]->getType()=="avoidObstacles")
		{
			switch (behaviorFlag[i]) {
				case 0:
					//cout << "Solo Seek" << endl;
					break;
				case 1: //AO normal
					tw.angular.z = behaviors[i]->getDesiredO();
					tw.linear.x = behaviors[i]->getDesiredV();
					twists.push_back(tw);
					break;
				case -1: //solo AO, peligro de colision
					danger = true;
					tw.angular.z = behaviors[i]->getDesiredO();
					tw.linear.x = behaviors[i]->getDesiredV();
					twists.push_back(tw);
					//ctrlPublisher->publish(twists[i]);
					//cout << "Solo AO" << endl;
					//return 1;
					break;
			}
		}
	}
	std::vector<float> behaviorState = getOneVectorState();

	//print state
	// for (std::vector<float>::iterator itb = behaviorState.begin(); itb != behaviorState.end(); ++itb)
	// {
	// 	cout << *itb << " ";
	// }
	// cout << endl;

	if (twists.size() == 1) {
		//nothing to blend
		ctrlPublisher->publish(twists[0]);
	} else {


		//suma ponderada de los desiresVs y Ws
		//cout << "seek: v=" << twists[0].linear.x << " o=" << twists[0].angular.z*180/PI << endl;
		//cout << "aObs: v=" << twists[1].linear.x << " o=" << twists[1].angular.z*180/PI << endl;
		ansState=actualState;
		actualState=behaviorState;

		if (ansState != actualState) {
			updateWeights(behaviorState);
			// cout << " Nuevos w: ";
		} /*else {
			// cout << " Mantiene: ";
		}*/
		// cout << pesos[0] << " , " << pesos[1] << endl;


		float rx = (pesos[0] * twists[0].linear.x * cos(twists[0].angular.z)) + (pesos[1] * twists[1].linear.x * cos(twists[1].angular.z));
		float ry = (pesos[0] * twists[0].linear.x * sin(twists[0].angular.z)) + (pesos[1] * twists[1].linear.x * sin(twists[1].angular.z));

		float v = sqrt(pow(rx,2)+pow(ry,2));
		float alpha = atan2(ry,rx);

		/*cout << "Blending" << endl;
		cout << "SEEK :  v  " << twists[0].linear.x << " o  " << twists[0].angular.z*180/PI << endl;
		cout << "RESU :  v  " << v << " o  " << alpha*180/PI << endl;
		cout << "AOBS :  v  " << twists[1].linear.x << " o  " << twists[1].angular.z*180/PI << endl;
		cout << endl << rx << " " << ry << endl << endl;*/

		geometry_msgs::Twist twist;
		twist.angular.z = alpha;
		twist.linear.x = v;
		ctrlPublisher->publish(twist);
	}
	if (llegada) {
		return 0;
	} else if (danger){
		return -1;
	}
	return 1;
}

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

void Agent::updateWeights(std::vector<float>)
{
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

std::vector<float> Agent::getOneVectorState()
{
	std::vector<float> behaviorState;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		std::vector<float> auxState = behaviors[i]->getState();
		behaviorState.insert(behaviorState.end(), auxState.begin(), auxState.end());
	}
	return behaviorState;
}

std::vector< std::vector<float> > Agent::getIndividualVectorState()
{
	std::vector< std::vector<float> > state;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		std::vector<float> auxVect = behaviors[i]->getState();
		state.push_back(auxVect);
	}
	return state;
}

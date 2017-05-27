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
Agent::Agent(unsigned int id, Factory* factoryPtr)
{
	robotId = id;

	std::stringstream name;
	name << "agent" << robotId;
	//Generar prefijo del nombre de los topics del robot del simulador para futuras suscripciones
	pretopicname = new std::stringstream;
	*pretopicname << "/" ;
	//Intento instanciar los comportamientos
	// if ( factoryPtr->instanciateSeekBehavior( robotId, pretopicname->str(), &behaviors, &myType) )
	if ( factoryPtr->instanciateOABehavior( robotId, pretopicname->str(), &behaviors, &myType) )
	{
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
	else
	{
		cout << "Agent " << robotId << ": Missing parameter in configuration file." << endl;
	}
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
	/* Recupero la orientación deseada de cada comportamiento */
	float desiredW;
	// float behaviorV[behaviors.size()];
	// float behaviorO[behaviors.size()];
	int behaviorFlag[behaviors.size()];
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
					myTwist.angular.z = behaviors[i]->getDesiredW();
					myTwist.linear.x = behaviors[i]->getDesiredV();
					ctrlPublisher->publish(myTwist);
					break;
			}
		}
		else if (behaviors[i]->getType()=="avoidObstacles")
		{
			switch (behaviorFlag[i]) {
				case 0:
					cout << "AO nada para hacer. Solo seek." << endl;
					break;
				case 1: //AO normal
					myTwist.angular.z = behaviors[i]->getDesiredW();
					myTwist.linear.x = behaviors[i]->getDesiredV();
					ctrlPublisher->publish(myTwist);
					break;
				case -1: //solo AO, peligro de colision
					myTwist.angular.z = behaviors[i]->getDesiredW();
					myTwist.linear.x = behaviors[i]->getDesiredV();
					ctrlPublisher->publish(myTwist);
					break;
			}
		}
	}
	return 1;

		//
		// behaviorO[i] = behaviors[i]->getDesiredW();
		// behaviorV[i] = behaviors[i]->getDesiredW();
		// if (behaviors[i]->getType()=="seek")
		// {
		// 	behaviors[i]->update();
		// 	 = desiredW;//error del angulo segun seek
		// 	myTwist.linear.x = behaviors[i]->getDesiredV();
		// }
		// else if (behaviors[i]->getType()=="avoidObstacles")
		// {
		// 	behaviors[i]->update();
		// 	behaviorDelta[i] = desiredW;//error del angulo segun obstacle avoidance
		// 	myTwist.linear.x = behaviors[i]->getDesiredV();
		// }
		// behaviorDelta[i] = 1- fabs(behaviorDelta[i]);
//	}
	/* Pido los pesos, weights evalua los casos donde se ignora algun comportamiento y disribuye el peso de este entre los demas */
	//Actualizo el estado
	//updateState();
	//Si el estado no cambia, los pesos son los mismos
	//printState();
	//cout << endl;
	//weights->getWeights(state,&w);
/*	if ((state != ansState) or (w.empty()))
	{
		//
		//print de la nueva elección de la tabla

		for (std::vector<float>::iterator itw = w.begin(); itw != w.end(); itw++) {
			cout << *itw << " ";
		}
		for (int i = 0; i < behaviors.size(); ++i)
		{
			cout << behaviorDelta[i]*180 << " ";
		}
		cout << endl;

	}*/
	/*
	// Efectuo la suma ponderada de las orientaciones de cada comportamiento

	float totalDelta  = 0;
	for (int i = 0; i < behaviors.size(); ++i)
	{
		totalDelta += w[i]*behaviorDelta[i];	//calculamos la suma de los errores ponderados
	}*/
	//float desiredAngle = addAngle(tita, totalDelta);		//angulo deseado = angulo + error

	//myTwist.angular.z = totalDelta;		//para este error que velocidad corresponde
	/*if (myType == "agenteOnlyAvoidObstacles")	//solo para test
	{
		myTwist.linear.x = 0.2;
	}*/

	// ctrlPublisher->publish(myTwist);				//envío la velocidad

	//cout << myTwist.angular.z << " = S(" << w[0] << " * " << behaviorDelta[0] << ") + OA(" << w[1] << " * " << behaviorDelta[1] << ")" << endl;
}

/*float Agent::addAngle(float initialAngle, float error)
{	//angulo deseado = angulo + error

	float desiredAngle = initialAngle - error ;

	if(desiredAngle>1)
	{
		desiredAngle = 2 - desiredAngle;
	}
	else if (desiredAngle<-1)
	{
		desiredAngle = 2 + desiredAngle;
	}
	if (desiredAngle>1.0 ^ desiredAngle<-1.0)
	{
		cout << "ERROR1 ADDANGLE EN AGENT " << desiredAngle << endl ;
	}
	return desiredAngle;
}*/

/*float Agent::deltaAngle(float initialAngle, float desiredAngle)
{

	//paso todo a angulos [0,360) (esta en la medida de orientacion de ros [-1,1) )
	//escala = 2/360 = 1/180
	initialAngle = toScale(initialAngle * 180) ;
	desiredAngle = toScale(desiredAngle * 180) ;

	float deltaAng = initialAngle - desiredAngle ;

	if (abs(deltaAng) > 180)
	{
		if(deltaAng < 0)
		{
			deltaAng = 360 + deltaAng ;
		}
		else if (deltaAng > 0)
		{
			deltaAng = deltaAng - 360 ;
		}
	}
	//vuelvo a la escala de ROS
	deltaAng = deltaAng / 180;

	//cout << "delta angulo R: " << deltaAng << endl ;

	if (deltaAng>1.0 ^ deltaAng<-1.0)
	{
		cout << "ERROR1 DELTAANGLE EN AGENT" << deltaAng << endl ;
	}

	return deltaAng;
}*/

/*float Agent::turningVel(float error)
{	//para este error que velocidad corresponde

	if (error>1.0 ^ error<-1.0)
	{
		cout << "ERROR1 TURNINGVEL EN AGENT " << error<< endl ;
	}

	return -error;
}*/

/*float Agent::toScale(float angle)
{	//pasa los angulos entre [0 , 360)
	if (angle < 0.0)
	{
		angle = 360 + angle;
	}
	else if (angle > 360)
	{
		angle = angle - 360;
	}
	return angle;
}*/

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

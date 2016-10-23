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
Agent::Agent(unsigned int id, Factory* factoryPtr)
{
	robotId = id;
	std::stringstream name;
	name << "agent" << robotId;

	//Generar prefijo del nombre de los topics del robot del simulador para futuras suscripciones
	pretopicname = new std::stringstream;
	if (!imAlone())
	{
		*pretopicname << "/robot_" << robotId << "/";
	}
	else
	{
		*pretopicname << "/" ;
	}

	//Intento instanciar los comportamientos

	if (factoryPtr->instanciateBehaviors(robotId,pretopicname->str(),&behaviors,&weights,&myType))
	{
		
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

		cout << "Agent " << robotId << ": GOOD." << endl << "Recibi " << behaviors.size() << " comportamientos:" << endl;
		for (int i = 0; i < behaviors.size(); ++i)
		{
			w = weights->getWeights();
			cout << "BH: " << behaviors[i]->getName() << " con W: " << w[i] << endl;
		}
		nbBehaviors = behaviors.size();
	}
	else
	{
	    cout << "Agent " << robotId << ": Missing parameter in configuration file." << endl;
	}
}

Agent::~Agent() {
	delete rosNode;
	delete ctrlPublisher;
	delete odomSubscriber;
	delete myData;
	//LIBERAR LOS BEHAVIORS

}

/*--------------------------- Update ------------------------------------
 *	Hace update a los behavior q corresponden, luego sumar los twists de
 *	cada uno de ellos ponderadamente y comunicar eso a los actuadores del
 *	robot
 ----------------------------------------------------------------------*/

void Agent::update() 
{
	// cout << "IN: " << myData->pose.pose.orientation.z << "R (" << myData->pose.pose.orientation.z*180 << "°)" << endl << endl;

	float totalDelta = pondSum();				//error definitivo = suma de los errores ponderada
//	cout << "suma de errores ponderado: " << totalDelta << endl;
	float desiredAngle = addAngle(myData->pose.pose.orientation.z, totalDelta);		//angulo deseado = angulo + error

	cout << "GLOBAL: " << endl << "DESIRED(R" << desiredAngle << ") = " << "INIT(R" << myData->pose.pose.orientation.z << ") + DELTA(R" << -totalDelta << ")" << endl;

	myTwist.angular.z = turningVel(totalDelta);		//para este error que velocidad corresponde

	// cout << "velocidad publicada: R" << myTwist.angular.z << endl << endl;

	if (myType == "agenteOnlyAvoidObstacles")	//solo para test
	{
	 	myTwist.linear.x = 0.2;
	} 

    ctrlPublisher->publish(myTwist);				//envío la velocidad
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

float Agent::addAngle(float initialAngle, float error){	//angulo deseado = angulo + error
	
	//cout << "ADDANGLE" <<endl;
	//cout << initialAngle << "R (" << initialAngle*180 << "°) - " << error << "R (" << error*180 << "°)" << endl; 
	
	//paso todo a angulos [0,360) (esta en la medida de orientacion de ros [-1,1) )
	//escala = 2/360 = 1/180

	initialAngle = initialAngle * 180 ;
	error = error * 180 ;

	float desiredAngle = initialAngle - error ;
	//cout << "= " << desiredAngle/180 << "R (" << desiredAngle << "°)" << endl ;
	desiredAngle = desiredAngle / 180;

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
}

float Agent::deltaAngle(float initialAngle, float desiredAngle){	//error del angulo

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

	cout << "deltaAng( " << deltaAng << "° ) = " << "initialAngle( " << initialAngle << "° ) - " << "desiredAngle( " << desiredAngle << "° ) " << endl ;

	//vuelvo a la escala de ROS
	deltaAng = deltaAng / 180;

//	cout << "delta angulo R: " << deltaAng << endl ;

	if (deltaAng>1.0 ^ deltaAng<-1.0)
	{
		cout << "ERROR1 DELTAANGLE EN AGENT" << deltaAng << endl ;
	}

	return deltaAng;
}

float Agent::turningVel(float error){	//para este error que velocidad corresponde

	if (error>1.0 ^ error<-1.0)
	{
		cout << "ERROR1 TURNINGVEL EN AGENT " << error<< endl ;
	}

	return -error;
}

float Agent::toScale(float angle){	//pasa los angulos entre [0 , 360)
	if (angle < 0.0)
	{
		angle = 360 + angle;
	}
	else if (angle > 360)
	{
		angle = angle - 360; 
	}
	return angle;
}

float Agent::pondSum()
{
	float desiredW;
	float behaviorDelta[nbBehaviors];
	float tempW[nbBehaviors];
	w = weights->getWeights();
	float sum = 0;
	float distribute = 0;								// de esto se tiene que encargar weights
	int zeros = 0;										// de esto se tiene que encargar weights
	// recupero los W de cada comportamiento	
	for (int i = 0; i < nbBehaviors; ++i)
	{
		desiredW = behaviors[i]->getDesiredW();
		if (behaviors[i]->getName()=="seekReactive")
		{
			cout << "SEEK:" << endl << "desiredW " << desiredW << "R (" << desiredW*180 << "°)" << endl;
			behaviorDelta[i] = deltaAngle(myData->pose.pose.orientation.z, desiredW);	//error del angulo segun seek
			tempW[i] = w[i];
			myTwist.linear.x = behaviors[i]->getDesiredV();
		}
		else if (behaviors[i]->getName()=="avoidObstaclesReactive")
		{
			if (desiredW == -1.0)
			{
				behaviorDelta[i] = 0;													//obstacle avoidance no percibe un obstaculo no modifica el angulo actual
				cout << "NO OBSTACLE" << endl << endl << endl;
				tempW[i] = 0;
				distribute +=  w[i];	// de esto se tiene que encargar weights
				zeros++;													// de esto se tiene que encargar weights
			}
			else 
			{
				cout << "OBSAV:" << endl << "desiredW "<< desiredW << "R (" << desiredW*180 << "°)" << endl;
				behaviorDelta[i] = deltaAngle(myData->pose.pose.orientation.z, desiredW);	//error del angulo segun seek									
				tempW[i] = w[i];	// de esto se tiene que encargar weights
			}
		}
		cout << endl;
	}

	//redistribuyo los pesos que no se utilizan		// de esto se tiene que encargar weights
	distribute = distribute / (nbBehaviors - zeros); //distribuyo equitativamente entre los comportamientos que estan activos
	for (int i = 0; i < nbBehaviors; ++i)
	{
		if (tempW[i] != 0)
		{
			tempW[i] += distribute;
		}
	}

	//ya los pesos distribuidos, hago la suma ponderada
	for (int i = 0; i < nbBehaviors; ++i)
	{
		sum += tempW[i]*behaviorDelta[i];
	}
	
	return sum;
}

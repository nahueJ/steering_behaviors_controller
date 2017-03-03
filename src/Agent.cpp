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
	roundCounter = 0;
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

	if (factoryPtr->instanciateBehaviors(robotId,pretopicname->str(),&behaviors,&weights,&myType,&state))
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

		//Simulation parameters
		sets.push_back("bitmap \"setD.png\"");
		sets.push_back("bitmap \"setE.png\"");

		std::vector<string> robotPoseAux;
		robotPoseAux.push_back("-6.25 -6.25 0 ");
		robotPoseAux.push_back("-1.25 -6.25 0 ");
		robotPoseAux.push_back("6.25 -6.25 0 ");
		robotPoseAux.push_back("6.25 -1.25 0 ");
		robotPoseAux.push_back("6.25 6.25 0 ");
		robotPoseAux.push_back("1.25 6.25 0 ");
		robotPoseAux.push_back("-6.25 6.25 0 ");
		robotPoseAux.push_back("-6.25 1.25 0 ");
		//el vector de posiciones se almacena en el mismo orden!
		initPosition.push_back(std::make_pair(-6.25, -6.25));
		initPosition.push_back(std::make_pair(-1.25, -6.25));
		initPosition.push_back(std::make_pair(6.25 , -6.25));
		initPosition.push_back(std::make_pair(6.25 , -1.25));
		initPosition.push_back(std::make_pair(6.25 , 6.25));
		initPosition.push_back(std::make_pair(1.25 , 6.25));
		initPosition.push_back(std::make_pair(-6.25, 6.25));
		initPosition.push_back(std::make_pair(-6.25, 1.25));

		std::vector<string> robotOrientationAux;
		robotOrientationAux.push_back("0");
		robotOrientationAux.push_back("45");
		robotOrientationAux.push_back("90");
		robotOrientationAux.push_back("135");
		robotOrientationAux.push_back("180");
		robotOrientationAux.push_back("225");
		robotOrientationAux.push_back("270");
		robotOrientationAux.push_back("315");
		for (std::vector< std::string >::iterator itp = robotPoseAux.begin(); itp != robotPoseAux.end(); ++itp)
		{
			for (std::vector< std::string >::iterator ito = robotOrientationAux.begin(); ito != robotOrientationAux.end(); ++ito)
			{
				std::string auxStrP = *itp;
				std::string auxStrO = *ito;
				auxStrP.insert( auxStrP.end(), auxStrO.begin(), auxStrO.end() );
				robotPose.push_back(auxStrP);
			}
		}
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
	float totalDelta = pondSum();//error definitivo = suma de los errores ponderada
	//float desiredAngle = addAngle(myData->pose.pose.orientation.z, totalDelta);		//angulo deseado = angulo + error
	myTwist.angular.z = turningVel(totalDelta);		//para este error que velocidad corresponde
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
	/**********************************************************/
	/* Recupero la orientación deseada de cada comportamiento */
	/**********************************************************/
	float desiredW;
	float behaviorDelta[behaviors.size()];
	for (int i = 0; i < behaviors.size(); ++i)
	{
		desiredW = behaviors[i]->getDesiredW();
		if (behaviors[i]->getType()=="seek")
		{
			behaviorDelta[i] = deltaAngle(myData->pose.pose.orientation.z, desiredW);	//error del angulo segun seek
			myTwist.linear.x = behaviors[i]->getDesiredV();
		}
		else if (behaviors[i]->getType()=="avoidObstacles")
		{
			behaviorDelta[i] = deltaAngle(myData->pose.pose.orientation.z, desiredW);	//error del angulo segun seek
		}
	}
	/**************************************************************************************************************/
	/* Pido los pesos, weights evalua los casos donde se ignora algun comportamiento y disribuye el peso de este entre los demas */
	/**************************************************************************************************************/
	//Actualizo el estado
	updateState();
	//Si el estado no cambia, los pesos son los mismos
	int restart = 0;
	if ((state != ansState) or (w.empty()))
	{
		printState();
		restart = weights->getWeights(state,&w);
		for (std::vector<float>::iterator itw = w.begin(); itw != w.end(); itw++) {
			cout << *itw << " ";
		}
		for (int i = 0; i < behaviors.size(); ++i)
		{
			cout << behaviorDelta[i] << " ";
		}
		cout << endl;
	}
	/*************************************************************************/
	/* Efectuo la suma ponderada de las orientaciones de cada comportamiento */
	/*************************************************************************/
	float sum = 0;
	if (restart == 0) {
		for (int i = 0; i < behaviors.size(); ++i)
		{
			sum += w[i]*behaviorDelta[i];
		}
	} else if (restart == 1) {	//Se ha recibido un refuerzo y hay que lanzar una nueva simulacion
		restartRoutine();
	}
	return sum;
}

void Agent::updateState()
{
	ansState = state;
	state.clear();
	for (int i = 0; i < behaviors.size(); ++i)
	{
		std::vector<float> auxVect = behaviors[i]->getState();
		state.push_back(auxVect);
	}
}

void Agent::printState()
{
	cout << "PS: ";
	for (std::vector< std::vector<float> >::iterator ita = state.begin(); ita < state.end(); ++ita)
	{
		for (std::vector<float>::iterator itb = (*ita).begin(); itb < (*ita).end(); ++itb)
		{
			cout << *itb << " ";
		}
	}
	// cout << endl;
}

void Agent::restartRoutine(){
	system("killall stageros &");
	sleep(1);
	roundCounter++;
	cout << "Round: " << roundCounter << endl;
	//las strings que hay que reemplazar
	string strReplaceS = "bitmap";
	string strReplaceP = "robotPose";
	//eleccion aleatoria del mapa
	int randnro = rand()% sets.size();
	string strNewS = sets[randnro];
	//eleccion aleatoria de posicion
	randnro = rand()% robotPose.size();
	string strNewP = "pose [";
	string strNewPend = "]";
	strNewP.insert( strNewP.end(), robotPose[randnro].begin(), robotPose[randnro].end() );
	strNewP.insert( strNewP.end(), strNewPend.begin(), strNewPend.end() );
	//set objetivo aleatorio
	//setSeekObjective(randnro/8);
	setSeekObjective(robotPose[randnro]);
	//Abrimos el archivo base y el que se usará para la simulacion
	std::ifstream filein("./src/steering_behaviors_controller/world/setBase.world"); //File to read from
	std::ofstream fileout("./src/steering_behaviors_controller/world/set.world"); //Temporary file
	if(!filein || !fileout)
	{
		cout << "Error opening files!" << endl;
	} else {
		//buscamos las lineas a reemplazar (mapa y posicion) y las ocupamos con un mapa y una posicion aleatoria (x,y, orientacion)
		string strTemp;
		while(std::getline(filein, strTemp))
		{
			if(strTemp == strReplaceS){
				strTemp = strNewS;
			} else if (strTemp == strReplaceP){
				strTemp = strNewP;
			}
			strTemp += "\n";
			fileout << strTemp;
		}
		system("rosrun stage_ros stageros /home/nahuel/catkin_ws/src/steering_behaviors_controller/world/set.world &");
		filein.close();
		fileout.close();
	}
}


void Agent::setSeekObjective(string strPos){
	std::stringstream pos(strPos);
	float data[4];
	//extraigo de la cadena x, y y el angulo de inicio
	for(int i=0;i<4;i++){
	  pos >> data[i];
	}
	//separar los puntos alejados del objetivo
	std::vector< std::pair<float, float> > auxObj;
	std::pair<float, float> auxP = std::make_pair(data[0], data[1]);
	for (std::vector< std::pair<float, float> >::iterator itp = initPosition.begin(); itp != initPosition.end(); ++itp)
	{
		float dx= auxP.first - itp->first;
		float dy= auxP.second - itp->second;
		float dist = sqrt(pow(dx,2.0)+pow(dy,2.0));
		if (dist>12.0) {
			auxObj.push_back(*itp);
		}
	}
	//elegir uno aleatorio
	int randnro = rand()% auxObj.size();
	//cout << "I from string: " << strPos << endl;
	//cout << "I:" << data[0] << " " << data[1] << " " << data[3] << " Obj:"
	cout << "NEW OBJ:(" << auxObj[randnro].first << "," << auxObj[randnro].second << ")" << endl;
	//Convertir a coordenadas odometricas la posicion final
	float angCoordOdom = data[3]*PI/180; //en el que esta orientado el robot
	//Traslacion de centro del sist de coordenadas
	std::pair<float, float> auxIntermedio = std::make_pair(auxObj[randnro].first-data[0], auxObj[randnro].second-data[1]);
	//Rotación del sist de coordenadas
	auxP.first = cos(angCoordOdom) * auxIntermedio.first + sin(angCoordOdom) * auxIntermedio.second;
	auxP.second = cos(angCoordOdom) * auxIntermedio.second - sin(angCoordOdom) * auxIntermedio.first;
	//buscar el behavior seek y setear el objetivo
	for (std::vector<SteeringBehavior*>::iterator itb = behaviors.begin(); itb != behaviors.end(); ++itb)
	{
		if ((*itb)->getType() =="seek")
		{
			(*itb)->setGoal(auxP.first, auxP.second);
		}
	}
}

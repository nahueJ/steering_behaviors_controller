/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "WallAvoidance.h"

/**
 * WallAvoidance implementation
 * 
 * Wall avoidance steers to avoid potential collisions
 * with a wall.
 */

void WallAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& messure)
{
	sensor_msgs::LaserScan tmpLaser = *messure;
	std::string tmpFrameId = tmpLaser.header.frame_id;	//separo el miembro frame_id para analizar de que laser viene la medicion
	int laserNumber = atoi(&tmpFrameId[tmpFrameId.size()-1]); //extraigo el ultimo caracter del frame_id que indica el id del laser
	lasers[laserNumber]=tmpLaser;	//almaceno la medicion en el lugar que le corresponde en el arrays con las ultimas mediciones recibidas de los laseres
}

void WallAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
}

/**			CONSTRUCTOR
 * @param unsigned int id
 * @param mySensor
 */
WallAvoidance::WallAvoidance(unsigned int id, unsigned int mySensor) : SteeringBehavior(id)
{	
	laserId = mySensor;

	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;

	cout << "Construyendo para robot " << robotId << " laserId " << laserId << endl; 
	//generar el nombre del nodo con el robotId
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;

	remappingsArgs.insert(ros::M_string::value_type( "__master", nameMaster.str()));

	//generar el nombre del nodo con el robotId
	std::stringstream name;
	name << "wallavoidance_" << laserId;

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
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(basetopic.str(), 1000, &WallAvoidance::odomCallback,this);


	/* Subscripcion a los topic de los laser*/
	//obtener la cantidad de lasers
	nroLasers = getNumberOfLasers(robotId);
	cout << "El robot " << robotId << " tiene " << nroLasers << " lasers" << endl ;
	//suscripcion al topic de cada uno de los lasers
	for (int i = 0; i < nroLasers; ++i)
	{
		//generar el nombre del topic a partir del robotId
		std::stringstream senstopic;
		senstopic << "/robot_" << robotId << "/base_scan_" << i;
		//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
		ros::Subscriber* tmpSubscriber;
		tmpSubscriber = new ros::Subscriber;
		*tmpSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(senstopic.str(), 1000, &WallAvoidance::sensorCallback,this);
		//lo añado a mi vector de suscripciones a lasers
		sensorSubscriber.push_back(tmpSubscriber);
	}

	//inicializo el puntero con las variables para almacenar los valores de los lasers
	lasers = new sensor_msgs::LaserScan[3];
}

WallAvoidance::~WallAvoidance() {
	delete rosNode;
//	delete sensorSubscriber;
	delete odomSubscriber;
}


/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void WallAvoidance::update() const{

//calculo del twist deseado para evitar chocar paredes
	//dist min 10
	//dist max 0.5
	//
	//revisar entre los 3 laseres cual tiene el mínimo (dist mas proxima a una pared)
	//
	//en el laser del minimo calcular junto con el del medio el angulo de la pared
	//
	//con eso puedo establecer dist y angulo de impacto
	//
	//angulo de impacto saco la normal
	//
	//con los datos de pose saco la dif de angulo entre el ideal y el real
	//
	//
	//
	//


}

/**
 * returns the number of lasers
 */
unsigned int WallAvoidance::getNumberOfLasers(unsigned int id)
{
	char robotsChar[10]; //buffer para obtener el resultado del comando ejecutado en la terminal

	FILE* fp;

	//generar el argumento para la busqueda de topics de lasers
	std::stringstream sscommand;
	sscommand << "/opt/ros/indigo/bin/rostopic list | /bin/grep -c '/robot_" << id << "/base_scan'" ;
	const std::string tmp = sscommand.str();
	const char* command = tmp.c_str();
	/*Open the commando for reading*/
	fp = popen(command ,"r");

	/*Read the output*/
	fgets(robotsChar, sizeof(robotsChar), fp);

	return atoi(robotsChar);
}
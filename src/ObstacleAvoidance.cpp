/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "ObstacleAvoidance.h"

/**
 * ObstacleAvoidance implementation
 * 
 * Wall avoidance steers to avoid potential collisions
 * with a wall.
 */

void ObstacleAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::LaserScan tmpLaser = *scan;
	std::string tmpFrameId = tmpLaser.header.frame_id;	//separo el miembro frame_id para analizar de que laser viene la medicion
	int laserNumber = atoi(&tmpFrameId[tmpFrameId.size()-1]); //extraigo el ultimo caracter del frame_id que indica el id del laser

	for (int i = 0; i < 3; ++i)
	{
		lasers[laserNumber,i]=scan->ranges[i];
	}
	//calcMin(lasers);
}

void ObstacleAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
	update();
}

/**			CONSTRUCTOR
 * @param unsigned int id
 * @param mySensor
 */
ObstacleAvoidance::ObstacleAvoidance(unsigned int id, std::string pre) : SteeringBehavior(id, pre)
{	
	distMax = 15.0 ;
	distMin = 0.5 ;

	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;

	//generar el nombre del nodo con el robotId
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;

	remappingsArgs.insert(ros::M_string::value_type( "__master", nameMaster.str()));

	//generar el nombre del nodo con el robotId
	std::stringstream name;
	name << "wallavoidance_" << robotId;

	//inicializa el nodo
	ros::init(remappingsArgs, name.str());

	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	rosNode = new ros::NodeHandle;

	/* Subscripcion al topic odometro de este robot*/
	//generar el nombre del topic a partir del robotId
	std::stringstream odomtopicname;
	odomtopicname << pretopicname << "odom" ;
	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(odomtopicname.str(), 1000, &ObstacleAvoidance::odomCallback,this);
	
	myData = new nav_msgs::Odometry;

	/* Subscripcion a los topic de los laser*/
	//obtener la cantidad de lasers
	nroLasers = getNumberOfLasers(robotId);

	//inicializo el puntero con las variables para almacenar los valores de los lasers
	lasers = new float[nroLasers,3];	//por construcción o representación los lasers devuelven 3 valores

	std::stringstream* lasertopicname;
	
	//suscripcion al topic de cada uno de los lasers
	for (int i = 0; i < nroLasers; ++i)
	{
		//generar el nombre del topic a partir del robotId
		lasertopicname = new std::stringstream ;
		*lasertopicname << pretopicname << "base_scan_" << i;
		cout << lasertopicname->str() << endl;
		//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
		ros::Subscriber* tmpSubscriber;
		tmpSubscriber = new ros::Subscriber;
		*tmpSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(lasertopicname->str(), 1000, &ObstacleAvoidance::sensorCallback,this);
		//lo añado a mi vector de suscripciones a lasers
		sensorSubscriber.push_back(tmpSubscriber);
		delete lasertopicname;
	}
}

ObstacleAvoidance::~ObstacleAvoidance() {
	delete rosNode;
	while (!sensorSubscriber.empty())
	{
		ros::Subscriber* tmpPtr= sensorSubscriber[sensorSubscriber.size()];
		sensorSubscriber.pop_back();
		delete tmpPtr;
	}
	//delete sensorSubscriber;	//una manera rapida de liberar vectores?
	delete [] lasers;
	delete odomSubscriber;
}


/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void ObstacleAvoidance::update() {
	//calculo del twist deseado para evitar chocar
	for (int i = 0; i < nroLasers; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			cout << "lasers[" << i << "," << j << "]= " << lasers[i,j] << endl ;
		}
	}

	setDesiredW(myData->pose.pose.orientation.z);

	//en el laser del minimo calcular junto con el del medio el angulo de la pared
	//con eso puedo establecer dist y angulo de impacto
	//angulo de impacto saco la normal
	//con los datos de pose saco la dif de angulo entre el ideal y el real
}

/**
 * returns the number of lasers
 */
unsigned int ObstacleAvoidance::getNumberOfLasers(unsigned int id)
{
	char robotsChar[10]; //buffer para obtener el resultado del comando ejecutado en la terminal

	FILE* fp;

	//generar el argumento para la busqueda de topics de lasers
	std::stringstream sscommand;
	sscommand << "/opt/ros/indigo/bin/rostopic list | /bin/grep -c '" << pretopicname << "base_scan'" ;
	const std::string tmp = sscommand.str();
	const char* command = tmp.c_str();
	/*Open the commando for reading*/
	fp = popen(command ,"r");

	/*Read the output*/
	fgets(robotsChar, sizeof(robotsChar), fp);

	return atoi(robotsChar);
}

/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void ObstacleAvoidance::calcMin(float matrix[][3]) {
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			cout << matrix[i,j] << " " ;
		}
		cout << endl;
	}
}
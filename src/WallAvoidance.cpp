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

void WallAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	sensor_msgs::LaserScan tmpLaser = *scan;
	std::string tmpFrameId = tmpLaser.header.frame_id;	//separo el miembro frame_id para analizar de que laser viene la medicion
	int laserNumber = atoi(&tmpFrameId[tmpFrameId.size()-1]); //extraigo el ultimo caracter del frame_id que indica el id del laser
//	*lasers[laserNumber]=scan;	//almaceno la medicion en el lugar que le corresponde en el arrays con las ultimas mediciones recibidas de los laseres
	for (int i = 0; i < 3; ++i)
	{
		lasers[laserNumber,i]=scan->ranges[i];
	}
}

void WallAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
}

/**			CONSTRUCTOR
 * @param unsigned int id
 * @param mySensor
 */
WallAvoidance::WallAvoidance(unsigned int id) : SteeringBehavior(id)
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
//	lasers = new sensor_msgs::LaserScan::ConstPtr[nroLasers];
	lasers = new float[nroLasers,3];	//por construcción o representación los lasers devuelven 3 valores
}

WallAvoidance::~WallAvoidance() {
	delete rosNode;
	//delete sensorSubscriber;	//una manera rapida de liberar vectores?
	delete [] lasers;
	delete odomSubscriber;
}


/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void WallAvoidance::update() {
	//calculo del twist deseado para evitar chocar paredes
	//revisar entre los 3 laseres cual tiene el mínimo (dist mas proxima a una pared)
	int imin = 0;	//coord i de la menor medida en el arreglo de medidas
	int jmin = 1;	//coord j de la menor medida en el arreglo de medidas

	for (int i = 0; i < nroLasers; ++i)
	{
		for (int j = 0; j < 3; ++j)	//cada laser tiene 3 rayos
		{
			if (lasers[imin,jmin] > lasers[i,j])
			{
				imin = i;
				jmin = j;
			}
		}
		
	}	

	float tmpX;
	float tmpZ;
	float menorMedida = lasers[imin,jmin];
	
	if (menorMedida <= distMax)
	{
		tmpX = ( 1.1f * menorMedida / 10.0f) - 0.1f;	//da 0 para dist=0.1 y da 1 para dist=10
		if (tmpX < 0.0f)
		{
			tmpZ = 1.0f;
		}
		else{
			tmpZ = 1.1f-tmpX;			
		}

		//verificar la dirección del giro
		if ((imin == 2) | ((imin == 0) ^ (lasers[0,0]>lasers[0,2])) )
		{
			tmpZ = -tmpZ;
		}

		//si estoy muy cerca de la pared es mejor retroceder y girar en el otro sentido
		if (menorMedida < distMin)
		{
			tmpX = -0.5;	//al estar tan cerca de la pared tmpX tiene un valor muy pequeno, entonces lo aumento arbitrariamente para retroceder
			tmpZ = -tmpZ;
		}
	}
	else if (menorMedida > distMax)	//EN ESTA SITUACION SE DEBERIA MANDAR UN IGNORENME
	{
		tmpX = 0.0f;		
		tmpZ = 0.0f;
	}

	setDesiredTwist(tmpX,tmpZ);
	//en el laser del minimo calcular junto con el del medio el angulo de la pared
	//con eso puedo establecer dist y angulo de impacto
	//angulo de impacto saco la normal
	//con los datos de pose saco la dif de angulo entre el ideal y el real
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
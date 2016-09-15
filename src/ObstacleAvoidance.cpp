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
	
	for (int i = 0; i < haz; ++i)
	{
		laser[i]=scan->ranges[i];
	}
}

void ObstacleAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
	x = myData->pose.pose.position.x;
	y = myData->pose.pose.position.y;
	tita = myData->pose.pose.orientation.w;
}

/**			CONSTRUCTOR
 * @param unsigned int id
 * @param mySensor
 */
ObstacleAvoidance::ObstacleAvoidance(unsigned int id, std::string pre, config_t* configurationPtr) : SteeringBehavior(std::string("obstacleAvoidance"), id, pre, configurationPtr)
{	
	//Cargar Valores de configuracion 
	// if (config.lookupValue("distMax", 	distMax)    &&
	//     config.lookupValue("distMin", 	distMin)    &&
	//     config.lookupValue("haz", 		haz))
	if(1)
	{
		cout << "Instanciando Obstacle Avoidance" << endl;
		//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
		ros::M_string remappingsArgs;

		//generar el nombre del nodo con el robotId
		std::stringstream nameMaster;
		nameMaster << "controller_" << robotId;

		remappingsArgs.insert(ros::M_string::value_type( "__master", nameMaster.str()));

		//generar el nombre del nodo con el robotId
		std::stringstream name;
		name << "obstacleavoidance_" << robotId;

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

		//inicializo el puntero con las variables para almacenar los valores de los lasers
		laser = new float[haz];	//almacena base_scan

		std::stringstream* lasertopicname;
		
		//generar el nombre del topic a partir del robotId
		lasertopicname = new std::stringstream ;
		*lasertopicname << pretopicname << "base_scan";

		//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
		sensorSubscriber = new ros::Subscriber;
		*sensorSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(lasertopicname->str(), 1000, &ObstacleAvoidance::sensorCallback,this);
		delete lasertopicname;
	}
	else
	{
	    cout << "OA " << robotId << ": Missing parameter in configuration file." << endl;
	}
}

ObstacleAvoidance::~ObstacleAvoidance() 
{
	delete rosNode;
	delete [] laser;
	delete odomSubscriber;
	delete sensorSubscriber;
}


/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void ObstacleAvoidance::update() 
{
	/* La idea del update es la siguiente
	asumo que tengo los datos actualizados del sensor laser: distancia de los rayos cada grado en 270 grados
	y tengo mi posicion actual x, y, tita

	cada laser inferior al rango max (8) representa un punto en el espacio
	y el espacio libre que rodea al laser es proporcional a la suma de las distancias del laser

	area actual = sumatoria(lasers)

	discretizamos las posibles velocidades angulares (w) del robot cada 0.1 en el intervalo (-1;1)

	para cada posible w

	estimo tita, x, y para la velocidad v=0.1
	
	x(t+1)=x+v*dt*cos(tita+w*dt)
	y(t+1)=y+v*dt*sen(tita+w*dt)

	en ese punto futuro las distancias a los puntos definidos por el laser son otras

	laser(t+1) = actualizarLaser()
	
	y el area libre que rodea al robot es otra
	area libre t+1 = sumatoria(lasers(t+1))

	El mejor w es aquel que promete mayor area libre

	*/

	float wideal;

	float maxArea = 0;
	for (int i = -9; i < 10; ++i)
	{
		float w = float(i/10);
		float xnext = x + 0.1 * 0.5 * cos(tita + w * 0.5);
		float ynext = y + 0.1 * 0.5 * sin(tita + w * 0.5);

		float area = estimateLasers(xnext, ynext);

		if (area > maxArea)
		{
			wideal = w;
		}
 	}
	setDesiredW(wideal);
}

float ObstacleAvoidance::getDesiredW()
{
	update();
	return desiredW;
}

float ObstacleAvoidance::estimateLasers(float xnext, float ynext)
{
	float sum = 0;
	for (int i = 0; i < haz; ++i)
	{
		float angulo = tita + i - haz/2; //angulo en el que se emitio el laser
		sum += nextDist(laser[i],angulo);
	}
}

float ObstacleAvoidance::nextDist( float dist, float angulo)
{
	//estima la posicion del punto percibido partiendo de la posicion del robot
	float xnext= x + dist * cos(angulo);
	float ynext = y + dist * sin(angulo);
	//la distancia en la siguiente posicion del robot sera
	float dnext = sqrt(pow(x-xnext,2.0) + pow(y-ynext,2.0));
}
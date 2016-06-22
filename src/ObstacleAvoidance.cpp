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

	if(laserNumber==0)
	{
		for (int i = 0; i < haz; ++i)
		{
			laserCentral[i]=scan->ranges[i];
		}
		minCentral = calcMin(laserCentral,&minCentralIndex);
	}
	else if (laserNumber==1)
	{
		for (int i = 0; i < haz; ++i)
		{
			laserIzquierda[i]=scan->ranges[i];
		}
		minDerecha = calcMin(laserDerecha,&minDerechaIndex);		
	}
	else if (laserNumber==2)
	{

		for (int i = 0; i < haz; ++i)
		{
			laserDerecha[i]=scan->ranges[i];
		}
		minIzquierda = calcMin(laserIzquierda,&minIzquierdaIndex);
	}
}

void ObstacleAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
}

/**			CONSTRUCTOR
 * @param unsigned int id
 * @param mySensor
 */
ObstacleAvoidance::ObstacleAvoidance(unsigned int id, std::string pre) : SteeringBehavior(id, pre)
{	
	//inicializar algunos valores
	distMax = 15.0 ;
	distMin = 0.5 ;
	
	minCentral = 0.0;
	minIzquierda = 0.0;
	minDerecha = 0.0;

	minCentralIndex = 0;
	minIzquierdaIndex = 0;
	minDerechaIndex = 0;

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

	haz = 3;		//por construcción o representación los lasers devuelven 3 valores

	//inicializo el puntero con las variables para almacenar los valores de los lasers
	laserCentral = new float[haz];	//almacena base_scan_0
	laserIzquierda = new float[haz];	//almacena base_scan_1
	laserDerecha = new float[haz];	//almacena base_scan_2

	std::stringstream* lasertopicname;
	
	//suscripcion al topic de cada uno de los lasers
	for (int i = 0; i < nroLasers; ++i)
	{
		//generar el nombre del topic a partir del robotId
		lasertopicname = new std::stringstream ;
		*lasertopicname << pretopicname << "base_scan_" << i;
		//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
		ros::Subscriber* tmpSubscriber;
		tmpSubscriber = new ros::Subscriber;
		*tmpSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(lasertopicname->str(), 1000, &ObstacleAvoidance::sensorCallback,this);
		//lo añado a mi vector de suscripciones a lasers
		sensorSubscriber.push_back(tmpSubscriber);
		delete lasertopicname;
	}
}

ObstacleAvoidance::~ObstacleAvoidance() 
{
	delete rosNode;
	while (!sensorSubscriber.empty())
	{
		ros::Subscriber* tmpPtr= sensorSubscriber[sensorSubscriber.size()];
		sensorSubscriber.pop_back();
		delete tmpPtr;
	}
	//delete sensorSubscriber;	//una manera rapida de liberar vectores?
	delete [] laserCentral;
	delete [] laserIzquierda;
	delete [] laserDerecha;
	delete odomSubscriber;
}


/**
 * gets the last data and actualizes the desiredTwist
 * @param myTwist
 */
void ObstacleAvoidance::update() 
{
	float wideal;
	int minLaser = minIndex(minCentral,minIzquierda,minDerecha);
	if (minLaser==0)
	{
		//El obstaculo más cercano está al frente
//		cout << "CENTRO" << endl;
		if (laserIzquierda[1] == laserDerecha[1])
		{
			if (laserCentral[0] > laserCentral [2])
			{
//				cout << "CENTRO IZQ > CENTRO DER" << endl;
				wideal = addW(myData->pose.pose.orientation.z, 90.0, 0);
			}
			else
			{
//				cout << "CENTRO DER > CENTRO IZQ" << endl;
				wideal = addW(myData->pose.pose.orientation.z, 90.0, 1);
			}
		}
		else if (laserIzquierda[1] < laserDerecha[1]) {
//			cout << "LASER DER > LASER IZQ" << endl;
			wideal = addW(myData->pose.pose.orientation.z, 90.0, 0);	
		}
		else if (laserIzquierda[1] > laserDerecha[1])
		{
//			cout << "LASER IZQ > LASER DER" << endl;
			wideal = addW(myData->pose.pose.orientation.z, 90.0, 1);
		}
	}
	if (minLaser==1)
	{
//		cout << "IZQUIERDA" << endl;
		//El obstaculo más cercano está a la Izquierda
		wideal = addW(myData->pose.pose.orientation.z, 60.0, 0);
	}
	if (minLaser==2)
	{
//		cout << "DERECHA" << endl;
		//El obstaculo más cercano está a la Derecha
		wideal = addW(myData->pose.pose.orientation.z, 60.0, 1);
	}
	if (minLaser==-1)
	{
		wideal = -1.0;	//NULL no esta implementado, como 2 no es un valor válido estoy usando eso
//		cout << "NO OBSTACLE" << endl;
	}
	setDesiredW(wideal);

	//en el laser del minimo calcular junto con el del medio el angulo de la pared
	//con eso puedo establecer dist y angulo de impacto
	//angulo de impacto saco la normal
	//con los datos de pose saco la dif de angulo entre el ideal y el real
}

float ObstacleAvoidance::getDesiredW()
{
	update();
	return desiredW;
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
float ObstacleAvoidance::calcMin(float matrix[], int* index) {
	*index = 0;
	for (int i = 1; i < haz; ++i)
	{
		if (matrix[*index] >= matrix[i])
		{
			*index = i ;
		}
	}
	return matrix[*index];
}

/**
 * funcion para sumarle al primer parametro (orientación) una cantidad de grados dada por el segundo parametro hacia izq o derecha (1 o 0) dada por el tercer parametro
 * @param myTwist
 */
float ObstacleAvoidance::addW(float w, float angle, int direction)
{
	w = (w + 1) * 180 ;
	float wi;
	if (direction == 0)		//left
	{
		wi = w - angle;
	}
	else if (direction == 1)	//right
	{
		wi = w + angle;
	}


	if (wi < 0.0)
	{
		wi = 360 + wi;
	}
	if (wi > 360)
	{
		wi = wi - 360 ;
	}

	if (wi>360.0 ^ wi<0.0)
	{
		cout << "ERROR1 ADDW EN OBS AV" << endl ;
	}

	wi = (2.0 * wi / 360) -1;

	if (wi>1.0 ^ wi<-1.0)
	{
		cout << "ERROR2 ADDW EN OBS AV" << endl ;
	}

	return wi;
}

/**
 * funcion para sumarle al primer parametro (orientación) una cantidad de grados dada por el segundo parametro hacia izq o derecha (1 o 0) dada por el tercer parametro
 * @param myTwist
 */
int ObstacleAvoidance::minIndex(float c, float l, float r) //c:minCentral l:minLeft r:minRight
{
	int min=-1;
	if (c<=4.0)				//distancia inferior a 4.0 en el centro
	{
		min = 0;
	}
	if (l<=c & l<=r)		//minimo en la izquierda
	{
		if (l<=2.0)			//distancia inferior a 2.0 en el laser de la izquierda
		{
			min = 1;
		}
	}
	else if (r<=c & r<=l)	//minimo en la derecha
	{
		if (r<=2.0)			//distancia inferior a 2.0 en el laser de la derecha
		{
			min = 2;
		}
	}
	return min;
}
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
	tita = myData->pose.pose.orientation.z;
}

/**			CONSTRUCTOR
 * @param unsigned int id
 * @param mySensor
 */
ObstacleAvoidance::ObstacleAvoidance(unsigned int id, std::string pre, Setting* configurationPtr) : SteeringBehavior(id, pre, configurationPtr)
{	
	//Cargar Valores de configuracion 
	distMax = (*configurationPtr)["distMax"];
	distMin = (*configurationPtr)["distMin"];
	haz = (*configurationPtr)["haz"];
	prescicion = (*configurationPtr)["prescicion"];
	sectores = (*configurationPtr)["sectores"];
	closestObstacle = 8.0;
	for (int i = 0; i < sectores; ++i)
	{
		zona.push_back(0.0);
	}
	div = haz/sectores;
	abanico = prescicion*haz;

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

ObstacleAvoidance::~ObstacleAvoidance() 
{
	delete rosNode;
	delete [] laser;
	delete odomSubscriber;
	delete sensorSubscriber;
}

void ObstacleAvoidance::update() 
{
	if (myName == "avoidObstaclesReactive")
	{
		updateAct();
	}
	else if (myName == "avoidObstaclesFuture")
	{
		updateFut();
	}
	else{
		cout << "Avoid Obstacles cant find update rules..." << endl;
	}
}

void ObstacleAvoidance::updateAct() 
{
	float wideal;
	int minArea;
	int maxArea = updateZona(&minArea);
	closestObstacle = zona[minArea];
	int medio = (sectores + 1) / 2;
	if (zonaSafe()){
		wideal = -1;		//codigo de no hay obstaculo
	}
	else if (zona[medio]<distMin)
	{
		wideal = emergencia();
		cout << "EMERGENCIA " ;
		printZona(); 
	}
	else if (zona[minArea] < distMin/2)	
	{
		wideal = escapeTo(minArea);
		cout << "ESCAPANDO " ;
		printZona();
	}
	else{
		wideal = relativeToAbsolute(maxArea);
	}
	setDesiredW(wideal);
}

void ObstacleAvoidance::updateFut() 
{
	// esta es una idea de toma de decision alternativo, en vez de decidir en base a la informacion actual,
	//a partir de la info actual, estimo la informacion que recibiría en un futuro si tomara cada una 
	//de las diferentes opciones que dispongo (diferentes velocidades angulares). Finalmente tomo aquella que
	//en un futuro me podría lle
	/*discretizamos las posibles velocidades angulares (w) del robot cada 0.1 en el intervalo (-1;1)

	para cada posible w

	estimo tita, x, y para la velocidad v=0.1
	
	x(t+1)=x+v*dt*cos(tita+w*dt)
	y(t+1)=y+v*dt*sen(tita+w*dt)

	en ese punto futuro las distancias a los puntos definidos por el laser son otras

	laser(t+1) = actualizarLaser()
	
	y el area libre que rodea al robot es otra
	area libre t+1 = sumatoria(lasers(t+1))

	El mejor w es aquel que promete mayor area libre*/

	float areaMaxima = 0.0;
	float wideal;
	for (int i = -9; i < 10; ++i)
	{
		float w = float(i/10);
		float xnext = x + 0.1 * 0.5 * cos(tita + w * 0.5);
		float ynext = y + 0.1 * 0.5 * sin(tita + w * 0.5);

		float area = estimateLasers(xnext, ynext);	// en vez de estimar la posicion futura de cada punto percibido, deberia estimar la minima que tendria cada zona del arco de lasers...lo que no se si 1)saco la minima de cada zona y estimo desde esos minimos,o 2) saco el futuro de cada uno de los lasers y estimo con esos lasers las minimas de cada zona

		if (area > areaMaxima)
		{
			wideal = w;
		}
 	}
	setDesiredW(wideal); // creo q falta unificar unidades estoy mandando un w en angulos, que deberia estar en radianes...o al revez
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

/*La idea del update es la siguiente
asumo que tengo los datos actualizados del sensor laser: distancia de los rayos cada grado en 270 grados
y tengo mi posicion actual x, y, tita

// cada laser inferior al rango max (8) representa un punto en el espacio
// y el espacio libre que rodea al laser es proporcional a la suma de las distancias del laser
// area actual = sumatoria(lasers)

Vamos a analizar las secciones de arco como una sola, entonces, el obstaculo va a estar representado por el
laser que devuelva la menor medida, es decir, tengo 270 haces, divididos en 3 sectores de 90 laseres cada uno,
si uno de los 90 haces de un sector devuelve una medida de x metros, se contabiliza como si los 90 ubiesen percibido esa minima
*/

int ObstacleAvoidance::updateZona(int* min)
{
	int indexMax = 0;
	int indexMin = 0;
	for (int i = 0; i < zona.size(); ++i)
	{
		float* first = &laser[div*i];
		float* last = &laser[div*(i+1)-1];
		if (first==last)
		{
			zona[i] = *last;
		}

		float* smallest = first;

		while (++first!=last)
		{	
			if (*first<*smallest)    // or: if (comp(*first,*smallest)) for version (2)
			{
				smallest=first;
			}
		}
		zona[i] = *smallest;
		if (i == 0)
		{
			indexMax = i;
			indexMin = i;
		}
		else{
			if (zona[i] > zona[indexMax])
			{
				indexMax = i;
			}
			if (zona[i] < zona[indexMin])
			{
				indexMin = i;
			}
		}
	}

	*min = indexMin;
	return indexMax;
}

//determinar de manera generica, en funcion del indice, la cantidad de sectores y el angulo del barrido,
//a partir de la orientacion leida del odometro, cual es la orientacion deseada
float ObstacleAvoidance::relativeToAbsolute(int relativeIndex)
{
	//el sensor devuelve las medidas de derecha a izq, pero como los calculos los hice al reves por ser mas
	//intuitivo, "doy vuelta" el indice y menos uno por que los indices empiezan de 0
	//relativeIndex = sectores - relativeIndex - 1;
	float resolucion = haz / sectores;
	float regla = relativeIndex - ( sectores - 1) / 2;
	float anguloRelativo = resolucion * regla;
	float test = anguloRelativo;
	anguloRelativo = anguloRelativo / 180;	 //graficar la recta para corroborar que angulo 0->1,90->0.5,180->0,270->-0.5,360->-1
	float wi = tita + anguloRelativo;	//angulo actual + angulo relativo deseado
//	cout << "AO AbsDes(" << wi << " " << wi*180 << "°) = AbsAct(" << tita << " " << tita*180 << "°) + RelDes("  << anguloRelativo << " " << anguloRelativo*180 << "°) " << relativeIndex << " -> " << test << endl;
	return wi;
}

float ObstacleAvoidance::emergencia()
{
	//sumar los valores de las zonas a la izq y a la derecha y ordenar doblar hacia la que haya mas espacio abierto
	int cantidad = sectores / 2 ;
	float izquierda = 0 ;
	float derecha = 0 ;
	float anguloRelativo = 90;
	for (int i = 0; i < cantidad; ++i)
	{
		derecha += zona[i];
		izquierda += zona[sectores - i];
	}
	if (derecha <= izquierda)
	{
		anguloRelativo = 90;
	}
	else
	{
		anguloRelativo = -90;
	}
	
	float test = anguloRelativo;
	anguloRelativo = anguloRelativo / 180;	 //graficar la recta para corroborar que angulo 0->1,90->0.5,180->0,270->-0.5,360->-1
	float wi = tita + anguloRelativo;	//angulo actual + angulo relativo deseado
//	cout << "AO EMERGENCIA AbsDes(" << wi << " " << wi*180 << "°) = AbsAct(" << tita << " " << tita*180 << "°) + RelDes("  << anguloRelativo << " " << anguloRelativo*180 << "°) " << test << endl;
	return wi;
}

//verifica que todas las areas de la zona no detecten obstaculos a dist menores que distMax
int ObstacleAvoidance::zonaSafe()
{
	for (int i = 0; i < sectores; ++i)
	{
		if (zona[i]<distMax)
		{
			return 0;
		}
	}
	return 1;
}

// en base al indice del area donde se encuentra el obstaculo mas peligroso, determino el angulo opuesto hacia el cual escapar
float ObstacleAvoidance::escapeTo(int minArea)
{
	float resolucion = haz / sectores;
	float regla = minArea - ( sectores - 1) / 2;
	float anguloRelativo = resolucion * regla;	//angulo en el que se ubica el obstaculo
	float test = anguloRelativo;
	if (anguloRelativo >= 180)
	{
		anguloRelativo -= 180;
	}
	else
	{
		anguloRelativo += 180;
	}
	cout << "Angulo relativo del OBS: " << test << "Angulo relativo de ESC: " << anguloRelativo << endl;
	anguloRelativo = anguloRelativo / 180;	 //graficar la recta para corroborar que angulo 0->1,90->0.5,180->0,270->-0.5,360->-1
	float wi = tita + anguloRelativo;	//angulo actual + angulo relativo deseado
//	cout << "AO AbsDes(" << wi << " " << wi*180 << "°) = AbsAct(" << tita << " " << tita*180 << "°) + RelDes("  << anguloRelativo << " " << anguloRelativo*180 << "°) " << relativeIndex << " -> " << test << endl;
	return wi;
}

void ObstacleAvoidance::printZona()
{
	cout << "Zona: I( ";
	for (int i = 0; i < zona.size(); ++i)
	{
		cout << zona[sectores-1-i] << " ";
	}
	cout << ")D" << endl;
}

float ObstacleAvoidance::getVble1()
{
	return closestObstacle;	//devuelve la dist al obstaculo mas cercano
}
float ObstacleAvoidance::getVble2()
{
	return 0.0;
}
float ObstacleAvoidance::getVble3()
{
	return 0.0;
}
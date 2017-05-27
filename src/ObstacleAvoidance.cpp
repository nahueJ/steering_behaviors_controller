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
	for (int i = 0; i < haz; ++i)
	{
		laser[i]=scan->ranges[i];
	}
}

void ObstacleAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
	tf::Pose pose;
	tf::poseMsgToTF(odom->pose.pose, pose);
	tita = tf::getYaw(pose.getRotation());	//tita: orientación en radianes para el marco coordenadas 2D, transformado a partir del marco de referencia 3D expresado por el quaternion (x,y,z,w) de la estructura orientation.
	//a partir de la posición inicial (0rad) tiene un rango (-PI/2 ; +PI/2] siendo el giro positivo hacia la izquierda del vehículo
}

ObstacleAvoidance::ObstacleAvoidance(unsigned int id, std::string pre, Setting* configurationPtr) : SteeringBehavior(id, pre, configurationPtr)
{
	//Cargar Valores de configuracion
	distMax = (*configurationPtr)["distMax"];
	distMin = (*configurationPtr)["distMin"];
	haz = (*configurationPtr)["haz"];
	prescicion = (*configurationPtr)["prescicion"];
	sectores = (*configurationPtr)["sectores"];

	for (int i = 0; i < sectores; ++i)
	{
		zona.push_back(3.0);
	}
	div = haz/sectores;
	abanico = prescicion*haz;

	//generar el nombre del nodo con el robotId, inicializa el nodo
	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	ros::M_string remappingsArgs;
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;
	remappingsArgs.insert(ros::M_string::value_type( "aoBh", nameMaster.str()));
	std::stringstream name;
	name << "obstacleavoidance_" << robotId;
	ros::init(remappingsArgs, name.str());
	rosNode = new ros::NodeHandle;

	// Subscripcion al topic odom, crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	std::stringstream odomtopicname;
	odomtopicname << pretopicname << "odom" ;
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(odomtopicname.str(), 1000, &ObstacleAvoidance::odomCallback,this);

	//inicializo el puntero con las variables para almacenar los valores de los lasers
	laser = new float[haz];	//almacena base_scan
	for (int i = 0; i < haz; ++i)
	{
		laser[i]=5.000;
	}

	// Subscripcion al topic base_scan, crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	std::stringstream lasertopicname;
	lasertopicname << pretopicname << "base_scan";
	sensorSubscriber = new ros::Subscriber;
	*sensorSubscriber = (*rosNode).subscribe<sensor_msgs::LaserScan>(lasertopicname.str(), 1000, &ObstacleAvoidance::sensorCallback,this);
	}

ObstacleAvoidance::~ObstacleAvoidance()
{
	delete rosNode;
	delete [] laser;
	delete odomSubscriber;
	delete sensorSubscriber;
}

int ObstacleAvoidance::update()
{
	updateState();
	oIdeal();//calcular la orientacion ideal
	int flag = vIdeal();//calcular la velocidad ideal
	return flag;
}

void ObstacleAvoidance::updateState()
{
	//indice de la medida del obstaculo mas cercano
	int minIndex = 0;
	for (int i = 0; i < haz; ++i)
	{
		if (laser[i] < laser[minIndex]) {
			minIndex = i;
		}
	}
	//actualizo valores de estado
	minLaserIndex = minIndex;
	stateContinuous = laser[minLaserIndex];
	cout << stateContinuous << endl;
	//discretizo el valor de estado continuo
	discretizarEstado();
}

void ObstacleAvoidance::oIdeal()
{
	int minIndex = minLaserIndex;
	if (minIndex==269)
	{
		//si el obstaculo esta en el primer laser tomo como minimo el siguiente así laser[minIndex-1] no da error
		minIndex = 268;
	}
	float distUno = laser[minIndex];
	float distDos = laser[minIndex+1];
	float alphaUno = (minIndex - 135) * PI /180;
	float alphaDos = (minIndex + 1 - 135) * PI /180;
	float distUnoX = distUno * cos(alphaUno);
	float distUnoY = distUno * sin(alphaUno);
	float distDosX = distDos * cos(alphaDos);
	float distDosY = distDos * sin(alphaDos);
	float obsAng;
	if (distUnoX == distDosX) {
		obsAng = PI/2;	//Si el obstaculo esta de frente del agente, las distancias en x son iguales, y la inclinacion se hace infinita...equivalente a una linea vertical
	} else {
		obsAng = atan2((distDosY - distUnoY),(distDosX - distUnoX));
	}
	//calcularl el error respecto a la orientacion actual
	float angRespuesta = (obsAng+(PI/2));
	if (angRespuesta > PI) {
		angRespuesta = angRespuesta - 2 * PI;
	}
	setDesiredW(angRespuesta);
}

int ObstacleAvoidance::vIdeal()
{
	float vmax = 0.5;
	if (stateContinuous>distMax) {
		setDesiredV(0.0);
		return 0;
	}
	else if (stateContinuous<distMin) {
		setDesiredV(vmax);
		return (-1);
	}
	else
	{
		float m =(vmax-0)/(distMin-distMax);
		float b = -m*distMax;
		float desiredV = (m*stateContinuous)+b;
		setDesiredV(desiredV);
		return 1;
	}
}

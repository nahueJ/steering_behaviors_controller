/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "Seek.h"

/**
 * Seek implementation
 *
 * The seek steering behavior returns a force that directs an agent toward a
 * target position.
 */
void Seek::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	*myData = *odom;		//almaceno en la variable correspondiente los ultimos valores recibidos
}

/**
 * @param objective
 * @param id
 * @param weight
 */
Seek::Seek(unsigned int id, std::string pre, Setting* configurationPtr) : SteeringBehavior(id, pre, configurationPtr)
{
	//Cargar Valores de configuracion
	target.position.x = (*configurationPtr)["targetX"];
	target.position.y = (*configurationPtr)["targetY"];
	standardVel = (*configurationPtr)["desiredV"];
	toleranceToTarget = (*configurationPtr)["toleranceToTarget"];

	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;

	//generar el nombre del nodo con el robotId
	std::stringstream nameMaster;
	nameMaster << "controller_" << robotId;

	remappingsArgs.insert(ros::M_string::value_type( "__master", nameMaster.str()));

	//generar el nombre del nodo con el robotId
	std::stringstream name;
	name << "seek_" << robotId;

	//inicializa el nodo
	ros::init(remappingsArgs, name.str());

	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	rosNode = new ros::NodeHandle;

	// Subscripcion al topic base_pose_ground_truth de este robot
	//generar el nombre del topic a partir del robotId
	std::stringstream topicname;

	topicname << pretopicname << "odom" ;

	//Crear el suscriptor en la variable de la clase y ejecutar la suscripcion
	odomSubscriber = new ros::Subscriber;
	*odomSubscriber = (*rosNode).subscribe<nav_msgs::Odometry>(topicname.str(), 1000, &Seek::odomCallback,this);

	myData = new nav_msgs::Odometry;

	setDesiredV(standardVel);
}

Seek::~Seek() {
	delete rosNode;
	delete odomSubscriber;
	delete myData;
}

/**
 * gets the last data and actualizes the desiredTwist
 * @param myPose
 */
void Seek::update() {

	errorx = target.position.x - myData->pose.pose.position.x;
	errory = target.position.y - myData->pose.pose.position.y;

	//actualizar estado
	updateState();

	//calcular la orientacion ideal!
	float wi = wIdeal(errorx,errory);

	setDesiredW(wi);

	//verificar distancia al objetivo
	float almost = sqrt(pow(errorx,2)+pow(errory,2));
	// cout << "ALMOST! " << almost << endl;
	if (toleranceToTarget>almost)
	{
		//si es menor que la tolerancia se detiene
		cout << "OBJETIVO ALCANZADO" << endl;
		setDesiredV(0.0);
		setDesiredW(0.0);
		system("killall stageros &");
	}
	else if (toleranceToTarget*3>almost)
	{
		setDesiredV(0.1);
	}
	else
	{
		setDesiredV(standardVel);
	}
}

float Seek::getDesiredW()
{
	update();
	return desiredW;
}

float Seek::wIdeal( float dx, float dy)
{
	float objAng;	//angulo al objetivo
	float wIdeal;	//orientación ideal del vehículo.
	if (dx == 0) {
		objAng = PI/2;	//Si el obstaculo esta enfrente del agente, las distancias en x son iguales, y la inclinacion se hace infinita...equivalente a una linea vertical
	} else {
		objAng = atan2(dy,dx);
	}
	if (objAng > PI) {
		objAng = objAng - 2 * PI;
	}

	cout << "objAngC " << objAng*180/PI << " myAng "<< (acos(myData->pose.pose.orientation.z)*2-PI)*180/PI<< endl;
	objAng = objAng - ((acos(myData->pose.pose.orientation.z)*2) - PI);
	if (objAng > 0) {		//correspondencia a la escala de orientaciones de ROS
		wIdeal = -objAng/PI +1;
	} else {
		wIdeal = -objAng/PI-1;
	}
	cout << " objAng: " << objAng*180/PI << " ROS" << wIdeal << " To:" << target.position.x << " , " << target.position.y << endl;
	return wIdeal;
}

std::vector<float> Seek::getState()
{
//El estado del comportamiento Seek es la distancia al objetivo, discretizada en los valores del vector valoresEstado
	return state;
}

void Seek::updateState()
{
	// cout << "SState/cont: ";
	float continuousValState=sqrt(pow(errorx,2)+pow(errory,2));
	//buscar dentro de los posibles valores aquel mas proximo al valor continuo
	int indexMin = 0;
	float min = continuousValState - valoresEstado[indexMin];
	for (int i = 1; i < valoresEstado.size(); ++i)
	{
		if ((continuousValState - valoresEstado[i]) > 0)
		{
			if ((continuousValState - valoresEstado[i])<min)
			{
				min = continuousValState - valoresEstado[i];
				indexMin=i;
			}
		}
		else{
			break;
		}
	}
	state[0]=valoresEstado[indexMin];
	// cout << continuousValState << "/" << state[0] << endl;
}
void Seek::setGoal(float xg, float yg){
	target.position.x = xg;
	target.position.y = yg;
}

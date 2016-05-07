/**
 * Project SteeringBehaviorsController
 * @author JOSE Nahuel
 * @version 1.0.0
 */


#include "../include/Controller.h"

/**
 * Controller implementation
 */

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
Controller::Controller(unsigned int id)
{
	cout << "Controller " << id << "° responde : Instanciando" << endl ;
	
	robotId = id;

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


	//generar el nombre del topic a partir del robotId
	std::stringstream pubtopic;
	pubtopic << "/robot_" << robotId << "/cmd_vel";

	//Crear el publicador y apuntarlo con la variable de la clase
	ctrlPublisher = new ros::Publisher;
	*ctrlPublisher = rosNode->advertise<geometry_msgs::Twist>(pubtopic.str(), 100000);

	//INSTANCIAR LOS BEHAVIORS --->>> FACTORY
	
	unsigned int laserId = 0;

	cout << "instanciando WallAvoidance de robot " << robotId << " laserId " << laserId << endl; 

	behavior = new WallAvoidance(robotId, laserId);
}

Controller::~Controller() {
	delete rosNode;
	delete ctrlPublisher;

	//LIBERAR LOS BEHAVIORS

}

/*--------------------------- Update -----------------------------------
 *	Hace update a los behavior q corresponden, luego sumar los twists de
 *	cada uno de ellos ponderadamente y comunicar eso a los actuadores del
 *	robot
 ------------------------------------------------------------------------*/

void Controller::update() 
{


	//Valor por default para mostrar
    myTwist.linear.x = 0.5;
    myTwist.angular.z = -0.5;

    ctrlPublisher->publish(myTwist);
}

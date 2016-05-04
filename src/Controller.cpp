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
	
	//Inicializacion del publisher en el topic cmd_vel del robot correspondiente
	ros::M_string remappingsArgs;
	remappingsArgs.insert(ros::M_string::value_type( "__master", "controllerHandler"));

//	TODO generar el nombre del nodo con el id!!

	ros::init(remappingsArgs, "controller_0");

	//crear el manejador del nodo y apuntarlo desde la variable de la clase
	ros::NodeHandle c;
//	rosNode = &c;

//	TODO ...generar el nombre del topic a partir del id

	//Crear el publicador y apuntarlo con la variable de la clase
//	ros::Publisher ctrlr_pub_0 = c.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1000);
//	ctrlPublisher = &ctrlr_pub_0;

}

Controller::~Controller() {

}

void Controller::update() 
{
	cout << "spining" << endl;
    myTwist.linear.x = 0.5;
    myTwist.angular.z = 0.5;
    ctrlPublisher->publish(myTwist);

}

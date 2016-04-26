//--------------------------- Constructor -----------------------------------
//	segun la cantidad de comportamientos solicitados en la variable behaviors
//	inicializa el vector de comportamientos steering_behavior* behaviors[] 
//	luego instancia un objeto de cada uno de los comportamientos, enviandoles 
//	una ponderacion por defecto
//	A continuacion inicializa la conexion por el topic del robot_id
//------------------------------------------------------------------------
Controller::Controller(	int 	id,				//identificador del robot
						int 	behaviors);		//comportamientos que se van a activar
{

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);
}

//--------------------------- Destructor -----------------------------------
//------------------------------------------------------------------------
Controller::~Controller()
{

}

//----------------------------- Update -----------------------------------
//	Recupera y suma cada uno de los aportes de los comportamientos y lo
//	publica por el topic del robot controlado
//------------------------------------------------------------------------
void Controller::update()
{
	linear=0;
	angular=0;
	for (int i = 0; i < numBehaviors; ++i)
	{
		behaviors[i].update();
		linear += behaviors[i].getLinear();		//Verificar la suma de velocidades
		angular += behaviors[i].getAngular();	//Verificar la suma de angulos
	}
	chatter_pub.publish(msg);
}
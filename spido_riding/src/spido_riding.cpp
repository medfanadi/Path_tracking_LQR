	#include "ros/ros.h"
	#include "geometry_msgs/Twist.h"
	#include <sstream>
	//#include "nav_msgs/Odometry.h"
	//#include "sensor_msgs/Imu.h"
		    #include <iostream>
#include <fstream>
#include <string.h>
#include "spido_pure_interface/cmd_drive.h"
using namespace std;

//**************************************
	// Déclaration du noeud
	ros::Publisher emetteur;
//***************************************
	// programme principal
	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "spido_riding");
		ros::NodeHandle noeud;
		ros::Time last_ros_time_;
	  	bool wait = true;
		double vx ;
		double beta_front;
		double beta_rear;
		double simul_time;

		//déclaration des paramètres d"entrées
		const std ::string PARAM_NAME_ls = "~linear_speed" ;
		const std ::string PARAM_NAME_fs = "~steering_angle_front" ;
		const std ::string PARAM_NAME_rs = "~steering_angle_rear" ;
		const std ::string PARAM_NAME_st = "~simulation_time" ;
		while (wait)
  		{
    			last_ros_time_ = ros::Time::now();
			if (last_ros_time_.toSec() > 0)
				wait = false;
		}
		//publier dans le topic /cmd_car
		emetteur=noeud.advertise<spido_pure_interface::cmd_drive>("/cmd_drive",1000);
		ros::Rate loop_rate(200);
		//déclaration d'un message "input" de type "spido_pure_interface::cmd_car"
		spido_pure_interface::cmd_drive input;
		//lecture des paramètres d'entrées
	        ros::param::get(PARAM_NAME_ls,  vx) ;
		ros::param::get(PARAM_NAME_fs,  beta_front) ;
		ros::param::get(PARAM_NAME_rs,  beta_rear) ;
		ros::param::get(PARAM_NAME_st,  simul_time) ;
		//attribuer les valeurs des paramètres au message
		input.linear_speed=vx;
		input.steering_angle_front=beta_front*3.14159265358979323846/180;	
		input.steering_angle_rear =beta_rear*3.14159265358979323846/180;	
		double t;
	  	double  t0= ros::Time::now().toSec();
		do {
			t=ros::Time::now().toSec()-t0;
			//envoyer le message		
			emetteur.publish(input);
			ros::spinOnce();
			loop_rate.sleep();
	


	    
		}while(t<simul_time);
		//arrêter le robot après le temps de simulation "simul_time"
		input.linear_speed=0;
		input.steering_angle_front=0;
		input.steering_angle_rear=0;
		emetteur.publish(input);

	}

	  

      
      
 

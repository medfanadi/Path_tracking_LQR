	#include "ros/ros.h"
	#include "geometry_msgs/Twist.h"
	#include <sstream>
	#include "nav_msgs/Odometry.h"
	//#include "sensor_msgs/Imu.h"
	#include <iostream>
	#include <fstream>
	#include <string.h>
	#include "spido_pure_interface/cmd_drive.h"
	#include "geometry_msgs/Pose2D.h"
#include <sensor_msgs/NavSatFix.h>
using namespace std;
double longitude , latitude;
double kkk= 0.017453293;
double Rt=6371000;
//double latitude_init=48.80551814; //sur le terrain réel
//double longitude_init=2.07650929;
double latitude_init=39.5080322117; //sur gazebo
double longitude_init=-0.46198057533;
/*const double INF = 1.e100;

		vector<pair<double, double> > matxref;
		vector<pair<double, double> > matyref;
		vector<pair<double, double> > matpsiref;
		vector<pair<double, double> > matxpref;
		vector<pair<double, double> > matypref;
		vector<pair<double, double> > matpsipref;	
		vector<pair<double, double> > matrhoref;	*/
double masse=880 , a=0.85 ,  b=0.85,  d=0.5 ,   Cf=15000  ,  vit  ,   Cr=15000   ,  moment=86.5 , k ;	

	/*double interpolate(double xx,vector<pair<double, double> > mat)
	{
    // Assumes that "mat" is sorted by .first
    // Check if x is out of bound
	if (xx > mat.back().first) return INF;
    	if (xx < mat[0].first) return -INF;
	vector<pair<double, double> >::iterator it, it2;
    // INFINITY is defined in math.h in the glibc implementation
	it = lower_bound(mat.begin(), mat.end(), make_pair(xx, -INF));
    // Corner case
	if (it == mat.begin()) return it->second;
	it2 = it;
	--it2;
	return it2->second + (it->second - it2->second)*(xx - it2->first)/(it->first - it2->first);

	}*/



//**************Ce noeud est un "publisher" et un "subscriber"*********************
	// Déclaration du noeud
	ros::Publisher cmd_drive_publisher;
	ros::Subscriber IMU_subscriber;
	ros::Subscriber sub_;
	//ros::Publisher pose_publisher;
//***************************************
	// programme principal
		double tref,xref,yref,Psiref,xpref,ypref,Psipref;
	double X ,Y,Qx, Qy, Qz, Qw, Xp, Yp, Psip;//données de l'odométrie. X,Y,XP,Yp sont exprimés dans le repère absolu. LesQ(x,y,z,w) sont 		les quaternions.
	void ImuCallback(const nav_msgs::Odometry::ConstPtr & odom_message);	
void retour_gps (const sensor_msgs::NavSatFix::ConstPtr& msg);
	int main(int argc, char **argv)
	{
		
		ros::init(argc, argv, "spido_LQR_droite_gnrl");//initiation du noeud.
		ros::NodeHandle noeud;
		ros::Time last_ros_time_;
	  	bool wait = true;
		double simul_time;


		const std ::string PARAM_NAME_st = "~simulation_time" ;
		//déclaration des paramètres d"entrées

		ros::param::get(PARAM_NAME_st,  simul_time) ;

		ifstream fichier("/home/bachir/spido_ws/src/spido_riding/pose.txt", ios::in);  //  ouvre en lecture
		//if(fichier)  // si l'ouverture a fonctionné
        	//{
		double tref,xref,yref,psiref,xpref,ypref,psipref,rhoref;
		//int:dim=0;
		//vector<double>  rho,dy,ddy;		
		/*while(fichier >> tref>>xref>>yref>>psiref>>xpref>>ypref>>psipref>>rhoref)
		{	matxref.push_back(make_pair(tref, xref));
			matyref.push_back(make_pair(tref, yref));
			matpsiref.push_back(make_pair(tref, psiref));
			matxpref.push_back(make_pair(tref, xpref));
			matypref.push_back(make_pair(tref, ypref));
			matpsipref.push_back(make_pair(tref, psipref));
			matrhoref.push_back(make_pair(tref, rhoref));
			
		
		}
		fichier.close();
		sort(matxref.begin(), matxref.end());
		sort(matyref.begin(), matyref.end());
		sort(matpsiref.begin(), matpsiref.end());
		sort(matxpref.begin(), matxpref.end());
		sort(matypref.begin(), matypref.end());
		sort(matpsipref.begin(), matpsipref.end());
		sort(matrhoref.begin(), matrhoref.end());
		}
		else
		cout<<"impossible d'ouvrir le fichier";	*/
		while (wait)
  		{
    			last_ros_time_ = ros::Time::now();
			if (last_ros_time_.toSec() > 0)
				wait = false;
		}
		IMU_subscriber = noeud.subscribe("/IMU", 10, ImuCallback);
		sub_=noeud.subscribe("/GPS/fix",10,retour_gps);
		cmd_drive_publisher=noeud.advertise<spido_pure_interface::cmd_drive>("/cmd_drive",10);
		//pose_publisher=noeud.advertise<geometry_msgs::Pose2D>("/Pose",1000);
		ros::Rate loop_rate(10);
		spido_pure_interface::cmd_drive   input;
		//geometry_msgs::Pose2D   pose_msg;

	       
		//attribuer les valeurs des paramètres au message
		//			input.linear_speed=2;
		//input.steering_angle_front=0;	
		//input.steering_angle_rear =0;
		//cmd_drive_publisher.publish(input);//envoyer (publier) le message input dans le topic cmd_drive
		//vit=input.linear_speed;	
		double t,vy,Psi,ey,epsi,vyref,xxpref,yypref,alphap,rho;
	  	double  t0= ros::Time::now().toSec();
		double alpha;
		//double xb,yb;//coordonnée d'un point par lequel passe la droite;
		//xb=3;yb=2;
		vit=2;
		input.linear_speed=vit;	
		Psiref=kkk*45;
		Psipref=0;
				
		do {
		//fichier >> tref>>xref>>yref>>psiref>>xpref>>ypref>>psipref;

			t=ros::Time::now().toSec()-t0;
			Psi=atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz));//transformation des quaternion vers l'angle de lacet.
			vy=-Xp*sin(Psi)+Yp*cos(Psi); // vy:vitesse latérale expriméé dans le repère véhicule
			/*xxpref=interpolate(t,matxpref);
			yypref=interpolate(t,matypref);
			alphap=interpolate(t,matpsipref);
			alpha=interpolate(t,matpsiref);
			vyref=-xxpref*sin(alpha)+yypref*cos(alpha); // vy:vitesse latérale expriméé dans le repère véhicule*/
			vyref=0;//-xpref*sin(psiref)+ypref*cos(psiref); // vy:vitesse latérale expriméé dans le repère véhicule
			//yyref=interpolate(t,matyref);
			
			ey=-(X-0)*sin(Psiref)+(Y-0)*cos(Psiref);
			//ey=((xref-X)*sin(psiref)-(yref-Y)*cos(psiref));
			//ey=Y;
			epsi=(Psi-Psiref);
			//vit=sqrt(xxpref*xxpref+yypref*yypref);

			//psipref=alphap;
			//k=interpolate(t,matrhoref);
			/*pose_msg.x=X;
			pose_msg.y=Y;
			pose_msg.theta=Psi;
			pose_publisher.publish(pose_msg);*///envoyer (publier) le message input dans le topic cmd_drive
			/// cas d'une droite verticale (là le ey=-(xrobot-xdroite_référence))

			input.steering_angle_front=  -(8800*Cf*vyref*a   + 8800*Cf*vyref*b  + 5*Cf*Psip*a*vit  + 3*Cf*vy*a*vit + 3*Cf*vyref*a*vit + 5*Cf*Psip*b*vit  + 3*Cf*vy*b*vit + 					3*Cf*vyref*b*vit + 1200*Cf*a*epsi*vit + 82*Cf*a*ey*vit + 1200*Cf*b*epsi*vit + 82*Cf*b*ey*vit)/(10000*Cf*vit*(a + b)) ;

			input.steering_angle_rear=(- 11071*Cr*vyref*a - 11071*Cr*vyref*b  + 5*Cr*Psip*a*vit  + 2*Cr*vy*a*vit + 2*Cr*vyref*a*vit + 5*Cr*Psip*b*vit  + 2*Cr*vy*b*vit +	2*Cr*vyref*b*vit + 1071*Cr*a*epsi*vit + 69*Cr*a*ey*vit + 1071*Cr*b*epsi*vit + 69*Cr*b*ey*vit)/(10000*Cr*vit*(a + b));
			cmd_drive_publisher.publish(input);
			ros::spinOnce();
			loop_rate.sleep();
                        //cout<<t<<"	"<<X<<"	"<<input.linear_speed<<endl;
			
			
	    	}while(t<simul_time);
		//arrêter le robot après le temps de simulation.
		input.linear_speed=0;
		input.steering_angle_front=0;
		input.steering_angle_rear=0;
		cmd_drive_publisher.publish(input);
		}
//subroutine qui enregistre les donnnées de l'odom.
void ImuCallback(const nav_msgs::Odometry::ConstPtr & odom_message){
	//X=odom_message->pose.pose.position.x;
	//Y=odom_message->pose.pose.position.y;
	Qx=odom_message->pose.pose.orientation.x;
	Qy=odom_message->pose.pose.orientation.y;
	Qz=odom_message->pose.pose.orientation.z;
	Qw=odom_message->pose.pose.orientation.w;
	Xp=odom_message->twist.twist.linear.x;
	Yp=odom_message->twist.twist.linear.y;
   	Psip=odom_message->twist.twist.angular.z;

	
}	  

 void transform (){
	X=Rt*(longitude - longitude_init)*kkk*cos(kkk*latitude_init);
	Y=Rt*(latitude - latitude_init)*kkk;
		
}


void retour_gps (const sensor_msgs::NavSatFix::ConstPtr& msg){
	ROS_INFO("reading ...");
	longitude= msg->longitude;
	latitude= msg->latitude;
	transform();
	//ROS_INFO("longitude= %f latitude= %f ",longitude, latitude);
	//ROS_INFO("x= %f y= %f ",x, y);
}


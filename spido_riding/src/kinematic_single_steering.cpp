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
const double INF = 1.e100;

		/*vector<pair<double, double> > matxref;
		vector<pair<double, double> > matyref;
		vector<pair<double, double> > matpsiref;*/
		vector<pair<double, double> > matxpref;
		vector<pair<double, double> > matypref;
		vector<pair<double, double> > matbfref;
		/*vector<pair<double, double> > matpsipref;	
		vector<pair<double, double> > matrhoref;*/	
double masse=880 , a=0.85 ,  b=0.85,  d=0.5 ,   Cf=15000  ,  vit  ,   Cr=15000   ,  moment=86.5 , k ;	

	double interpolate(double xx,vector<pair<double, double> > mat)
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

	}




//**************Ce noeud est un "publisher" et un "subscriber"*********************
	// Déclaration du noeud
	ros::Publisher cmd_drive_publisher;
	ros::Subscriber IMU_subscriber;
	ros::Subscriber sub_;
	//ros::Publisher pose_publisher;
//***************************************
	// programme principal
	double tref,xref,yref,psiref,xpref,ypref,psipref,rhoref,bfref,brref;
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
		ifstream fichier2("/home/bachir/spido_ws/src/spido_riding/steer.txt", ios::in);  //  ouvre en lecture
		if(fichier)  // si l'ouverture a fonctionné
        	{

		//int:dim=0;
		//vector<double>  rho,dy,ddy;		
		while(fichier >> tref>>xref>>yref>>psiref>>xpref>>ypref>>psipref>>rhoref)
		{	matxpref.push_back(make_pair(tref, xpref));
			matypref.push_back(make_pair(tref, ypref));
			
		}
		fichier.close();
		while(fichier2 >> tref>>bfref>>brref) matbfref.push_back(make_pair(tref, bfref));			
		fichier2.close();	
		
		
		sort(matxpref.begin(), matxpref.end());
		sort(matypref.begin(), matypref.end());
		sort(matbfref.begin(), matbfref.end());
		}
		else
		cout<<"impossible d'ouvrir le fichier";	
		while (wait)
  		{
    			last_ros_time_ = ros::Time::now();
			if (last_ros_time_.toSec() > 0)
				wait = false;
		}
		IMU_subscriber = noeud.subscribe("/IMU", 1000, ImuCallback);
		sub_=noeud.subscribe("/GPS/fix",1000,retour_gps);
		cmd_drive_publisher=noeud.advertise<spido_pure_interface::cmd_drive>("/cmd_drive",1000);
		//pose_publisher=noeud.advertise<geometry_msgs::Pose2D>("/Pose",1000);
		ros::Rate loop_rate(1000);
		spido_pure_interface::cmd_drive   input;
		//geometry_msgs::Pose2D   pose_msg;

	       
		//attribuer les valeurs des paramètres au message
		//			input.linear_speed=2;
		//input.steering_angle_front=0;	
		//input.steering_angle_rear =0;
		//cmd_drive_publisher.publish(input);//envoyer (publier) le message input dans le topic cmd_drive
		//vit=input.linear_speed;	
		double bf0,Psi,t,a=0.85,b=0.85,vit,tref,ex0,x0=0,ey0,y0=0,eteta0,teta0=0,z1,z2,z3,z4,dz1,dz2,dz3,dz4,w1,w2,beta0=0,k1=1,k2=1,k3=3,k4=3,L;
		double  t0= ros::Time::now().toSec();
		//double xb,yb;//coordonnée d'un point par lequel passe la droite;
		//xb=3;yb=2;
		input.steering_angle_rear=0;
		ex0=0;//(0-x0);
		ey0=0;//(3-y0);
		Psi=atan2(2.0 * (Qw * Qz + Qx * Qy),1.0 - 2.0 * (Qy * Qy + Qz * Qz));//transformation des quaternion vers l'angle de lacet.
		eteta0=0.7;//(Psi-teta0);
		z1=ex0;
		z2=ey0;
		z3=tan(eteta0);
		bf0=0;
		z4=(tan(beta0)-cos(eteta0)*tan(interpolate(ros::Time::now().toSec()-t0,matbfref)))/(L*cos(eteta0)*cos(eteta0)*cos(eteta0))+k2*z2;
		L=a+b;
		do {

			t=ros::Time::now().toSec()-t0;
			bfref=interpolate(t,matbfref);
			xpref=interpolate(t,matxpref);
			ypref=interpolate(t,matypref);
			vit=sqrt(xpref*xpref+ypref*ypref);   
input.linear_speed=vit;
w1=-k1*abs(vit)*(z1+z3/k2*(z4+(1+z3*z3)*tan(bfref)/L));
w2=-k3*vit*z3-k4*abs(vit)*z4;

dz1=(vit/L*tan(bfref))*z2+w1;
dz2=-(vit/L*tan(bfref))*z1+vit*z3+w1*z3;
dz3=-k2*vit*z2+vit*z4+w1*(z4-k2*z2+(1+z3*z3)*tan(bfref)/L);
dz4=w2;
z1=z1+0.001*dz1;
z2=z2+0.001*dz2;
z3=z3+0.001*dz3;
z4=z4+0.001*dz4;

input.steering_angle_front=atan((z4-k2*z2)*(L*cos(atan(z3))*cos(atan(z3))*cos(atan(z3)))+cos(atan(z3))*tan(bfref));

			cmd_drive_publisher.publish(input);
			ros::spinOnce();
			loop_rate.sleep();
                        //cout<<t<<"	"<<X<<"	"<<input.linear_speed<<endl;
		   // cout<< t<<"	"<<input.linear_speed<<"	"<<input.steering_angle_front<<"	"<<input.steering_angle_rear<<endl;	
		    cout<< t<<"	"<<tref<<xref<<yref<<vit<<endl;	
	   } while(t<simul_time);
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
	//ROS_INFO("reading ...");
	longitude= msg->longitude;
	latitude= msg->latitude;
	transform();
	//ROS_INFO("longitude= %f latitude= %f ",longitude, latitude);
	//ROS_INFO("x= %f y= %f ",x, y);
}


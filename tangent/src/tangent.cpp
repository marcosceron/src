#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cfloat>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;
bool laser_pronto = false;

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	current_laser = *laser;
	laser_pronto = true;
	return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr& pose)
{
        current_pose = *pose;

	return;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "Path");  // Inicializacao do Nodo

    ros::NodeHandle n; 		     

    // Configuracao do topico a ser publicado
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10); 

    // Configuracao dos topicos a serem lidos
    ros::Subscriber sub = n.subscribe("/base_scan", 10, lasercallback);	
    ros::Subscriber sub1 = n.subscribe("/base_pose_ground_truth", 10, posecallback);
   
    // Define a frequencia do no
    ros::Rate loop_rate(2);

    // Declaracoes
    geometry_msgs::Twist speed_create;  // Comando de Velocidade
    double v1=2.0, v2=2.0;              // Velocidades
    double goalx,goaly;                 // Alvos

    //Faz Leitura dos Parametros para o GOAL
    if(argc == 1){
        printf("Definido Padrao Aleatorio\n");
        srand(time(NULL));
        goalx=double(rand()%200)/10.0;
        srand(time(NULL));
        goaly=double(rand()%150)/10.0;
    }else{
        if(argc == 3){
            goalx=atof(argv[1]);
            goaly=atof(argv[2]);
            printf("Definido GOAL %lf %lf\n",goalx,goaly);
        }else{
            printf("Error in Goal\n");
            return -1;
        }
    }
    double orientation;    

    while(ros::ok() && !laser_pronto) 
    {
	ros::spinOnce();
        loop_rate.sleep();
    }
    while(ros::ok()) 
    {
	// Diferença X e Y do objetivo para o robô
	double difX = goalx - current_pose.pose.pose.position.x;
	double difY = goaly - current_pose.pose.pose.position.y;
	// Ângulo de orientação do objetivo	
	double goalOrientation = atan2(difY, difX);	
	
	/**	
	 Orientação - tipo Quarternion
	 yaw = ângulo de orientação do robô
	**/
	orientation = tf::getYaw(current_pose.pose.pose.orientation);	
	// Ângulo de radiano para grau	
	orientation = orientation*180/M_PI;	
	goalOrientation = goalOrientation*180/M_PI;	
	
	//ROS_INFO("Goal %lg", goalOrientation);
	//ROS_INFO("Robo %lg", orientation);

	double diferencaAngulos = goalOrientation - orientation;
	
	//ROS_INFO("Dif %lg", diferencaAngulos);
	double xObstaculo;
	double yObstaculo;
	
	/**	
	xObstaculo = range * cos(current_laser.angle_min + current_laser.angle_increment * i);
	yObstaculo = range * sin(current_laser.angle_min + current_laser.angle_increment * i);	
	**/
	//ROS_INFO("%lg", current_laser.ranges[current_laser.ranges.size()/2]);
	//ROS_INFO_STREAM(current_laser.ranges.size());	
	
	//ROS_INFO_STREAM(diferencaAngulos);

	double laserCentral = current_laser.ranges[540];
	double laserDireita = current_laser.ranges[405];
	double laserEsquerda = current_laser.ranges[675];

	//ROS_INFO("Central: %lg\nEsquerda: %lg\nDireita: %lg\n", laserCentral, laserEsquerda, laserDireita);	
	ROS_INFO("Dif: %lg",diferencaAngulos);	

		

	v2=0;
	v1=2.0;

	
	if(laserCentral<2 && laserDireita>laserCentral) {
		
		if(diferencaAngulos > 100) {
			
			v1=0;
			v2=2.0;
		}
		else if(diferencaAngulos>35 && diferencaAngulos<75) {
			//ROS_INFO("ang45");
			v1=0;
			v2=-2.0;		
		}
		
		// dif=63 lá em cima
		//break;
	
	}
	
	if(laserDireita<2 && laserCentral<2 && diferencaAngulos>0) {
		//ROS_INFO("IF1");

		v1=0;
		v2=2.0;	
	}
	
	
	if (laserEsquerda<1 && laserDireita<2 && laserCentral<8) {
		//ROS_INFO("IF2");
		v1=0;
		v2=1.0;	
	}
	if(laserCentral<1 && laserDireita<1 && diferencaAngulos<-200) {
		//ROS_INFO("Aqui");
		v1=0;
		v2=1.0;	
	}
	if(laserDireita<0.5) {
		v1=0;
		v2=1.0;	
	}

	
	if((current_pose.pose.pose.position.y>=12 && current_pose.pose.pose.position.y<=14)
	&& (current_pose.pose.pose.position.x>=6 && current_pose.pose.pose.position.x<=8)) {
		//ROS_INFO("Objetivo");
		v1=0;
		v2=0;	
	}
	
	
	if(diferencaAngulos>-15 && diferencaAngulos<15) {
		v2=0;
		v1=4.0;	
	}
	// Comentário que só deve aparecer no segundo trabalho
	//ROS_INFO("X: %lg\n, Y:%lg", current_pose.pose.pose.position.x, current_pose.pose.pose.position.y);
	
        // Envia Sinal de Velocidade
        speed_create.linear.x=v1;
        speed_create.angular.z=v2;
        vel_pub.publish(speed_create);
        ros::spinOnce();
        loop_rate.sleep();
}
    return 0;
}

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

using namespace std;

// Constantes
#define MAPPING_WIDTH 25.0
#define MAPPING_HEIGHT 20.0
#define GRID_RES_X 2 // cells per meters (cell density)
#define GRID_RES_Y 2
#define GRID_OFFSET_X -2
#define GRID_OFFSET_Y -2

const int GRID_WIDTH=MAPPING_WIDTH*GRID_RES_X,
    GRID_HEIGHT=MAPPING_HEIGHT*GRID_RES_Y;

char gMap[GRID_HEIGHT][GRID_WIDTH];

// Variaveis para enviar msgs
ros::Publisher marker_pub;
std_msgs::ColorRGBA txtColor;

double path_length=0.0;

//Variaveis Globais Para Leitura de Dados
nav_msgs::Odometry current_pose;
sensor_msgs::LaserScan current_laser;

bool laserReady=false,
     odomReady=false;

std_msgs::ColorRGBA makeColor(float r, float g, float b, float a=1.f)
{
    std_msgs::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

void resetMap()
{
    for(unsigned i = 0 ; i < GRID_HEIGHT;i++)
    {
        for(unsigned j = 0 ; j < GRID_WIDTH; j++)
            gMap[i][j] =0;
    }
}

void loadMap(string fileName)
{
    FILE *f = fopen(fileName.c_str(), "r");
    int v;
    for(unsigned i = 0 ; i < GRID_HEIGHT;i++)
    {
        for(unsigned j = 0 ; j < GRID_WIDTH; j++)
        {
            fscanf(f,"%d,",&v);
            gMap[i][j] = v;
        }
    }
    fclose(f);
}

void saveMap(string fileName)
{
    FILE *f = fopen(fileName.c_str(), "w");
    for(unsigned i = 0 ; i < GRID_HEIGHT;i++)
    {
        for(unsigned j = 0 ; j < GRID_WIDTH; j++)
        {
            fprintf(f,"%d,",int(gMap[i][j]));
        }
        fprintf(f,"\n");
    }
    fclose(f);
}

double mapStatistic()
{
    int notVisitedCount=0,
        visitedCount=0;

    for(unsigned i = 0 ; i < GRID_HEIGHT;i++)
    {
        for(unsigned j = 0 ; j < GRID_WIDTH; j++)
        {
            switch(gMap[i][j])
            {
                case 0: notVisitedCount++; break;
                case 1: visitedCount++; break;
            }
        }
    }
    return visitedCount/double(notVisitedCount+visitedCount);
}

void setMapV(const geometry_msgs::Point &p, char v)
{
    int j = floor((p.x-GRID_OFFSET_X) * GRID_RES_X),
        i = floor((p.y-GRID_OFFSET_Y) * GRID_RES_Y);
    if(i>=0 && i < GRID_HEIGHT &&
       j>=0 && j < GRID_WIDTH)
    {
        if(gMap[i][j] < 2)
        {
            gMap[i][j] = v;
            txtColor = makeColor(0,0,0);
        }
        else if(gMap[i][j] == 2)
        {
            txtColor = makeColor(1,0,0);
        }else
        {
            txtColor = makeColor(1,0,1);
        }
    }
}


void drawTxt(string ptsName, string txt,geometry_msgs::Point pt,
                   std_msgs::ColorRGBA c)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = ptsName;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = txt;

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = pt;

    marker.scale.z = 1.0;
    marker.color = c;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}

void desenhaPontos(string ptsName, vector<geometry_msgs::Point> points,
                   std_msgs::ColorRGBA c)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = ptsName;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE_LIST;

    marker.action = visualization_msgs::Marker::ADD;

    marker.points = points;

    marker.scale.x = 1.0/GRID_RES_X;
    marker.scale.y = 1.0/GRID_RES_Y;
    marker.scale.z = 0.05;

    marker.color = c;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}

geometry_msgs::Point getLaserPoint(int id)
{
    geometry_msgs::Point p;
    double ang = current_laser.angle_min +
            id* current_laser.angle_increment;

    p.x = current_laser.ranges[id] * cos(ang);
    p.y = current_laser.ranges[id] * sin(ang);
    return p;
}

void global2LocalFrame(geometry_msgs::Point gl,geometry_msgs::Point &loc)
{
    float rYaw = tf::getYaw(current_pose.pose.pose.orientation),
          ct = cos(rYaw), st = sin(rYaw);

    geometry_msgs::Point rP = current_pose.pose.pose.position;

    gl.x -= rP.x;
    gl.y -= rP.y;

    loc.x = gl.x * ct + gl.y * st;
    loc.y =-gl.x * st + gl.y * ct;
}

void local2GlobalFrame(geometry_msgs::Point loc,geometry_msgs::Point &gl)
{
    float rYaw = tf::getYaw(current_pose.pose.pose.orientation),
          ct = cos(rYaw), st = sin(rYaw);

    geometry_msgs::Point rP = current_pose.pose.pose.position;

    gl.x = rP.x + loc.x * ct - loc.y * st;
    gl.y = rP.y + loc.x * st + loc.y * ct;
}
double dist(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
    double dx = b.x - a.x,
           dy = b.y - a.y;
    return sqrt(dx*dx + dy*dy);
}

double mod(const geometry_msgs::Vector3 &v)
{
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

//Funcao Callback do Laser
void lasercallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
    current_laser = *laser;

    /*
    if(fabs(current_pose.twist.twist.angular.z) < 0.01 &&
       mod(current_pose.twist.twist.linear) < 0.01)
    {
        ROS_INFO("Speed %lg %lg",
                 fabs(current_pose.twist.twist.angular.z),
                 mod(current_pose.twist.twist.linear));

        geometry_msgs::Point p;
        for(unsigned i = 0; i < laser->ranges.size();i++)
        {
            if(laser->ranges[i] <30)
            {
                p = getLaserPoint(i);
                local2GlobalFrame(p,p);
                setMapV(p,2);
            }
        }
    }
    */
    laserReady=true;
    return;
}

//Funcao Callback da Pose
void posecallback(const nav_msgs::Odometry::ConstPtr& pose)
{
    geometry_msgs::Point p=  pose->pose.pose.position;
    geometry_msgs::Quaternion o= pose->pose.pose.orientation;

    if(odomReady)
        path_length+= dist(current_pose.pose.pose.position,p);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(p.x, p.y, 0.2) );
    tf::Quaternion q;
    q.setX(o.x);
    q.setY(o.y);
    q.setZ(o.z);
    q.setW(o.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, pose->header.stamp, "world", "ground_truth_pose"));

    transform.setOrigin( tf::Vector3(8, 8, 0.2) );
    q.setEuler(0,0,0);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, pose->header.stamp, "world", "odom"));

    current_pose = *pose;
    setMapV(pose->pose.pose.position,1);
    odomReady=true;
    return;
}

void drawMap()
{
    geometry_msgs::Point p;
    vector<geometry_msgs::Point> obj, notVis, vis,
            notMonitored;

    for(unsigned i = 0; i < GRID_HEIGHT; i++)
    {
        for(unsigned j = 0; j < GRID_WIDTH; j++)
        {
            p.x = GRID_OFFSET_X + j/double(GRID_RES_X) + (1.0/GRID_RES_X)*0.5; // Because the cube origen is in the center
            p.y = GRID_OFFSET_Y + i/double(GRID_RES_Y) + (1.0/GRID_RES_Y)*0.5;
            switch(gMap[i][j])
            {
                case 0: notVis.push_back(p); break;
                case 1: vis.push_back(p); break;
                case 2: obj.push_back(p); break;
                case 3: notMonitored.push_back(p); break;
            }
        }
    }

    desenhaPontos("not_visited",notVis,
                  makeColor(1,1,1));
    desenhaPontos("visited",vis,
                  makeColor(0,1,0));
    desenhaPontos("walls",obj,
                  makeColor(0,0,1));
    desenhaPontos("not_monitored",notMonitored,
                  makeColor(0,1,1));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Path");  // Inicializacao do Nodo
    ros::NodeHandle n;

    // Configuracao do topico a ser publicado
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Configuracao dos topicos a serem lidos
    ros::Subscriber sub = n.subscribe("/base_scan", 10, lasercallback);	
    ros::Subscriber sub1 = n.subscribe("/base_pose_ground_truth", 10, posecallback);
   
    // Define a frequencia do no
    ros::Rate loop_rate(3);


    if(argc >= 2)
    {
        loadMap(argv[1]);
    }
    else
    {
        resetMap();
    }

    // Wait sensors be ready
    while(!laserReady || !odomReady)
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(!ros::ok()) break;
    }

    char exploreStatistic[100];
    geometry_msgs::Point cp;
    cp.x = 11;
    cp.y = 6;
    cp.z = 2;
    txtColor = makeColor(0,0,0);

    //Loop Principal
    while(ros::ok())
    {
        drawMap();
        sprintf(exploreStatistic,"%.2lfm,%.2lf%% explored",
                path_length, mapStatistic()*100.0);
        drawTxt("explore_statistic",exploreStatistic,
                cp,txtColor);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

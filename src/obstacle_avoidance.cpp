#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <math.h>

//Define publisher e subscriber
ros::Subscriber sub;
ros::Subscriber sub_laser;
ros::Publisher pub;


//METODOS-------------------------------------------------------------------------

/*********************************************************************************
 * Recebe informacoes dos sensores e de direcao. Publica comandos de velocidade 
 * evitando bater em obstaculos.
**/
void obstacleAvoidanceControl(){

}

/*********************************************************************************
 * Callback da escuta de informacoes de velocidade
**/
void teleopCallback(geometry_msgs::Twist twist_teleop)
{
  //Imprime informacoes de twist
//  ROS_INFO("I heard:");
//  ROS_INFO("Lin X: [%f]", twist_teleop.linear.x);
//  ROS_INFO("Lin Y: [%f]", twist_teleop.linear.y);
//  ROS_INFO("Ang Z: [%f]", twist_teleop.angular.z);
    

  //Altera o twist devido a obstaculos TODO
  obstacleAvoidanceControl();

  //Publica twist para o cmd_vel robo
  pub.publish(twist_teleop);
  
}

/*********************************************************************************
 * Callback da escuta de informacoes do laser
 **/
void laserCallback(sensor_msgs::LaserScan scan)
{
    
    //Calcula Numero de Amostras
    int numero_amostras = (int) floor((scan.angle_max - scan.angle_min) / scan.angle_increment);
    
    //Imprime informacoes do laser
    int amostra = 0;
    ROS_INFO("I heard:");
    ROS_INFO("MIN[%d]: [%lf]", amostra, scan.ranges[amostra]);
    
    amostra = (int)floor(numero_amostras/8);
    ROS_INFO("---[%d]: [%lf]", amostra,  scan.ranges[amostra]);
    
    amostra = (int)floor(numero_amostras/4);
    ROS_INFO("---[%d]: [%lf]", amostra,  scan.ranges[amostra]);
    
    amostra = (int)floor(numero_amostras/2);
    ROS_INFO("MED[%d]: [%lf]", amostra,  scan.ranges[amostra]);
    
    amostra = (int)floor(2 * numero_amostras/3);
    ROS_INFO("+++[%d]: [%lf]", amostra,  scan.ranges[amostra]);
    
    amostra = (int)floor(5 * numero_amostras/6);
    ROS_INFO("+++[%d]: [%lf]", amostra,  scan.ranges[amostra]);
    
    amostra = numero_amostras;
    ROS_INFO("MAX[%d]: [%lf]", amostra,  scan.ranges[amostra]);
    
    
}

/*********************************************************************************
**/
int main(int argc, char **argv)
{

  //Inicializa o noh
  ros::init(argc, argv, "obstacle_avoidance");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  //Define as publicacoes de twist para o cmd_vel do robo
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //Define a escuta do topico com as velocidades fornecidas pelo turtlebot
  sub = n.subscribe("/cmd_vel_mux/input/teleop", 100, teleopCallback);
    
  //Define a escuta do topico com as medicoes do laser
  sub_laser = n.subscribe("/hokuyo_scan", 100, laserCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

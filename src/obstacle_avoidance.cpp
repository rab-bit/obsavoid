#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include <math.h>


//VARIAVEIS GLOBAIS---------------------------------------------------------------

//Define publishers e subscribers
ros::Subscriber sub;
ros::Subscriber sub_laser;
ros::Publisher pub;

//Variavel que armazena a ultima leitura do laser
sensor_msgs::LaserScan scan_mem;

//Variavel que funciona como um semaforo que indica se scan_mem foi inicializada
int scan_mem_active = 0;


//METODOS-------------------------------------------------------------------------

/*********************************************************************************
 * Recebe o angulo central de uma amostra, a mensagem do laser e o numero de amostras vizinhas ao angulo recebido que serao utilizadas para o calculo de uma distancia media
 **/
float getDistanceAverage(float angulo, sensor_msgs::LaserScan msg, int num_amostras){
    
    //Obtem o numero total de amostras
    int numero_amostras = (int) floor((msg.angle_max - msg.angle_min) / msg.angle_increment);
    
    int posicao_array = (int) floor((angulo - msg.angle_min)/msg.angle_increment);
    
    float sum = msg.ranges[posicao_array]; //Central
    
    for(int i = 1; i <= num_amostras; i++){
        sum += msg.ranges[posicao_array + i];
        sum += msg.ranges[posicao_array - i];
    }
    
    return sum/(2*num_amostras);
}

/*********************************************************************************
 * Recebe informacoes dos sensores e de direcao. Publica comandos de velocidade 
 * evitando bater em obstaculos.
**/
void obstacleAvoidanceControl(geometry_msgs::Twist twist_teleop){
    
    if (scan_mem_active == 0) return; //Verifica se a variavel scan_mem ja foi inicializada
    
    int numero_amostras = (int) floor((scan_mem.angle_max - scan_mem.angle_min) / scan_mem.angle_increment);
    
    //Se estiver andando no sentido positivo de x e houver obstaculo frontal,
    // reduz a velocidade exponencialmente
    if (twist_teleop.linear.x > 0){
        
        if(scan_mem.ranges[(int)floor(numero_amostras/2)] < 1) // 0 rad a uma distancia de 1m
            twist_teleop.linear.x *=
            fmin(1, exp(scan_mem.ranges[(int)floor(numero_amostras/2)] - 0.5) - 1);
        
        if(scan_mem.ranges[(int)floor(numero_amostras/3)] < .36) // -0.785 rad a uma distancia de 20cm
            twist_teleop.linear.x *=
            fmin(1, exp(scan_mem.ranges[(int)floor(numero_amostras/3)] - 0.10) - 1);
        
        if(scan_mem.ranges[(int)floor(2*numero_amostras/3)] < .36) // +0.785 rad a uma distancia de 20cm
            twist_teleop.linear.x *=
            fmin(1, exp(scan_mem.ranges[(int)floor(2*numero_amostras/3)] - 0.10) - 1);
        
    }

    //Se estiver andando no sentido negativo de x e houver obstaculos trazeiros
    else if (
             twist_teleop.linear.x < 0 &&
             (scan_mem.ranges[numero_amostras] < 1 // +2.355 rad
              || scan_mem.ranges[0] < 1) // -2.355 rad
             )
        twist_teleop.linear.x *=
        fmin(1, exp(fmin(scan_mem.ranges[numero_amostras], scan_mem.ranges[0]) -.8) - 1);
    
//    //Limita a velocidade angular
//    twist_teleop.angular.z = twist_teleop.linear.x;
    
    //Publica twist para o cmd_vel robo
    pub.publish(twist_teleop);

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
  obstacleAvoidanceControl(twist_teleop);
  
  
}

/*********************************************************************************
 * Callback da escuta de informacoes do laser
 **/
void laserCallback(sensor_msgs::LaserScan scan)
{
    
    //Calcula Numero de Amostras
    int numero_amostras = (int) floor((scan.angle_max - scan.angle_min) / scan.angle_increment);
    
    //Imprime informacoes do laser
    // 0 se refere ao menor angulo: -2.355
    // numero_amostras se refere ao maior angulo: +2.355
    int amostra = 0;
    ROS_INFO("I heard:");
    ROS_INFO("MIN[%d]: [%lf]", amostra, scan.ranges[amostra]); // -2.355 rad
    
    amostra = (int)floor(numero_amostras/6);
    ROS_INFO("---[%d]: [%lf]", amostra,  scan.ranges[amostra]); // -1.57 rad
    
    amostra = (int)floor(numero_amostras/3);
    ROS_INFO("---[%d]: [%lf]", amostra,  scan.ranges[amostra]); // -0.785 rad
    
    amostra = (int)floor(numero_amostras/2);
    ROS_INFO("MED[%d]: [%lf]", amostra,  scan.ranges[amostra]); // 0 rad
    
    amostra = (int)floor(2 * numero_amostras/3);
    ROS_INFO("+++[%d]: [%lf]", amostra,  scan.ranges[amostra]); // +0.785 rad
    
    amostra = (int)floor(5 * numero_amostras/6);
    ROS_INFO("+++[%d]: [%lf]", amostra,  scan.ranges[amostra]); // +1.57 rad
    
    amostra = numero_amostras;
    ROS_INFO("MAX[%d]: [%lf]", amostra,  scan.ranges[amostra]); // +2.355 rad
    
    //Armazena a leitura atual
    scan_mem = scan;
    scan_mem_active = 1;
    
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
    
    
  //Define a escuta do topico com as medicoes do laser
  sub_laser = n.subscribe("/hokuyo_scan", 100, laserCallback);

  //Define a escuta do topico com as velocidades fornecidas pelo turtlebot
  sub = n.subscribe("/cmd_vel_mux/input/teleop", 100, teleopCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

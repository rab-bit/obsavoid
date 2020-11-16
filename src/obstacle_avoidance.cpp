#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "VFH.h"
#include <sstream>
#include <math.h>
#include <vector>

#define BUBLE_SIZE .7 //< Tamanho da bolha
#define MAX_SPEED 1

//--------------------------------------------------------------------------------------------------
//VARIAVEIS GLOBAIS
//--------------------------------------------------------------------------------------------------

//Define publishers e subscribers
ros::Subscriber sub;
ros::Subscriber sub_laser;
ros::Publisher pub;

VFH *vfh;

//Variavel que armazena a ultima leitura do laser
sensor_msgs::LaserScan scan_mem;

// Circular buffer that stores last 40 scan informations
std::vector<sensor_msgs::LaserScan> laserScanBuffer;

//Variavel que funciona como um semaforo que indica se scan_mem foi inicializada
int scan_mem_active = 0;

//--------------------------------------------------------------------------------------------------
//METODOS
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
/*!
 * Deteccao de obstaculos para um vetor de ranges de laser
 *
 * \param msg Mensagem de leitura do laser
 * \param limite Limite a partir do qual eh detecatado um obstaculoa
 *
 * \return Se algum range for menor que o valor da bolha que envolve o robo, retorna 1
 */
bool checkForObstacles(sensor_msgs::LaserScan msg, float limite){
    
    int numero_amostras = (int) floor((msg.angle_max - msg.angle_min) / msg.angle_increment);
    
    //Le as amostras a cada 10 unidades
    for (int i = 0; i < numero_amostras; i+= 10){
        if(i > numero_amostras/6 && i < 5*numero_amostras/6) //90 graus
            if (msg.ranges[i] < limite) return true;
    }
    return false;
}

//--------------------------------------------------------------------------------------------------
/*!
 * Recebe informacoes dos sensores e de direcao. Publica comandos de velocidade
 * evitando bater em obstaculos.
 *
 * \param twist_teleop
 */
void obstacleAvoidanceControl(geometry_msgs::Twist twist_teleop){
    
    if (scan_mem_active == 0) return; //Verifica se a variavel scan_mem ja foi inicializada
    
    sensor_msgs::LaserScan msg = scan_mem; //Armazena dados de leitura do laser
    
    //Filtra velocidade maxima (e impede de andar no sentido negativo)
    twist_teleop.linear.x = fmax(0, fmin(MAX_SPEED, twist_teleop.linear.x));
    
    //Calcula raio da bolha de acordo com a velocidade do robo
    float tam_bolha = fmax(BUBLE_SIZE , BUBLE_SIZE * twist_teleop.linear.x);
    
    //Inicializa o algoritmo de desvio de obstaculos
    vfh->setLimite(60 - tam_bolha*2);
    
    //Se houver obstaculos, recalcula a tragetoria de acordo com o vfh
    if (checkForObstacles(msg, tam_bolha))
        twist_teleop = vfh->calculate(twist_teleop, msg);
    
    //Publica twist para o cmd_vel robo
    pub.publish(twist_teleop);
}


//--------------------------------------------------------------------------------------------------
/*!
 * Callback da escuta de informacoes de velocidade
 */
void teleopCallback(geometry_msgs::Twist twist_teleop)
{
    //Altera o twist devido a obstaculos TODO
    obstacleAvoidanceControl(twist_teleop);
}


//--------------------------------------------------------------------------------------------------
/*!
 * Callback da escuta de informacoes do laser
 */
void laserCallback(sensor_msgs::LaserScan scan)
{
    
    laserScanBuffer.push_back(scan);
    if(laserScanBuffer.size() > 40)
    {
        // Erases oldest element
        laserScanBuffer.erase(laserScanBuffer.begin());
    }
    
    scan_mem = scan;
    //ROS_INFO("%f\t%f", scan.range_min, scan.range_max);
    for(auto laserScan : laserScanBuffer)
    {
        for(auto range = scan.range_min ; range < scan.range_max ; range++)
        {
            scan_mem.ranges[range] += laserScan.ranges[range];
        }
    }
    
    for(auto range = scan.range_min ; range < scan.range_max ; range++)
    {
        scan_mem.ranges[range] /= laserScanBuffer.size();
    }
    
    //Informa inicializacao da scan_mem
    scan_mem_active = 1;
    
}


//--------------------------------------------------------------------------------------------------
//
int main(int argc, char **argv)
{
    //Inicializa o noh
    ros::init(argc, argv, "obstacle_avoidance");
    
    ros::NodeHandle n;
    
    //Inicializa o VFH
    vfh = new VFH();
    
    //Define as publicacoes de twist para o cmd_vel do robo
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    
    //Define a escuta do topico com as medicoes do laser
    sub_laser = n.subscribe("/hokuyo_scan", 100, laserCallback);
    //Define a escuta do topico com as velocidades fornecidas pelo turtlebot
    sub = n.subscribe("/cmd_vel_mux/input/teleop", 100, teleopCallback);
    
    ros::spin();
    
    return 0;
}


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <math.h>
#include <vector>

#define BUBLE_SIZE 1 //< Tamanho da bolha

//VARIAVEIS GLOBAIS---------------------------------------------------------------
//Define publishers e subscribers
ros::Subscriber sub;
ros::Subscriber sub_laser;
ros::Publisher pub;
//Variavel que armazena a ultima leitura do laser
sensor_msgs::LaserScan scan_mem;

// Circular buffer that stores last 40 scan informations
std::vector<sensor_msgs::LaserScan> laserScanBuffer;

//Variavel que funciona como um semaforo que indica se scan_mem foi inicializada
int scan_mem_active = 0;

//METODOS-------------------------------------------------------------------------

/*********************************************************************************
 * Recebe o angulo central de uma amostra, a mensagem do laser e o numero de 
 * amostras vizinhas ao angulo recebido que serao utilizadas para o calculo de uma
 * distancia media
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
    
    //Calcula media dos ranges
    return sum/(2*num_amostras);
}

/*********************************************************************************
 * Deteccao de obstaculos
 *
 * Se algum range for menor que o valor da bolha que envolve o robo, retorna 1
 **/
bool checkForObstacles(sensor_msgs::LaserScan msg){
    
    int numero_amostras = (int) floor((msg.angle_max - msg.angle_min) / msg.angle_increment);
    
    //Le as amostras a cada 10 unidades
    for (int i = 0; i < numero_amostras; i+= 10){
        if(i > numero_amostras/6 && i < 5*numero_amostras/6) //90 graus
            if (msg.ranges[i] < BUBLE_SIZE) return true;
    }
    return false;
}


/*********************************************************************************
 * Recebe informacoes dos sensores e de direcao. Publica comandos de velocidade
 * evitando bater em obstaculos.
 **/
void obstacleAvoidanceControl(geometry_msgs::Twist twist_teleop){
    
    if (scan_mem_active == 0) return; //Verifica se a variavel scan_mem ja foi inicializada
    
    
    //Enquanto houver obstaculos, recalcula a tragetoria de forma ir para o angulo com o maior range
    if (true){ //(checkForObstacles(scan_mem)) {
        ROS_INFO("-------------------------------");
        ROS_INFO("-------------------------------");
        //Computa novo angulo (desviar)
        sensor_msgs::LaserScan msg = scan_mem;
        float angulo_setor = 0.0872665; //< Angulo por setor (5graus)
        float angulo_abertura = 1.5708/2; //< Angulo maximo de abertura do laser para cada lado (45graus)
        float a = 60; //Magnitude maxima
        float b = 2; // 0 = a - 30b
        float limite = 56; //Limite para detectar vales
        int s_max = 3; //Numero de setores livres consecutivos para o robo passar
        int val_counter = 0; //Contador de setores por vale
        float best_val_ang = 1.6; //Vetor central do melhor vale escolhido
        
        //A cada setor de -90 a 90
        for (float alpha = (-1)*angulo_abertura; alpha < angulo_abertura; alpha += angulo_setor){
            
            float c = 1; //Probabilidade de haver um obstaculo no setor
            float obstacle_prox = getDistanceAverage(alpha, msg, 5); //Range medio de 11 medidas
            
            //Calcula magnitude
            float m = pow(c,2) * (a - b*obstacle_prox);

	    ROS_INFO("[%f] rad:\t %f", alpha, m);
            
            //Verifica vales
            if (m > limite || alpha+angulo_setor >= angulo_abertura){ //Se o valor atual for maior que o limite
                //Se o numero de setores for maior que o determinado e este for mais proximo do angulo atual que o anterior	
		ROS_INFO("-M:\t %f", m);
		ROS_INFO("-C:\t %d", val_counter);	
		if (val_counter >= s_max){
           	 float val_ang = alpha - (val_counter * angulo_setor)/2; //Calcula angulo central resultante
            		//Se o angulo for mais proximo que o atual
	      		if (abs(val_ang - twist_teleop.angular.z) < abs(best_val_ang - twist_teleop.angular.z))
    				best_val_ang = val_ang;
		}
                val_counter = 0; //Reseta o contador
            }
                
            else val_counter++;
	    ROS_INFO("C:\t %d", val_counter);
        }
        
        //O melhor angulo eh o escolhido pelo loop
        twist_teleop.angular.z = best_val_ang;

	ROS_INFO("Best rad:\t %f", best_val_ang);
	ROS_INFO("Twis rad:\t %f", twist_teleop.angular.z);
        
        //Controle da velocidade
        float best_val_mag = getDistanceAverage(best_val_ang, msg, 5); //Magnitude da direcao escolhida
	best_val_mag = pow(1,2) * (a - b*best_val_mag);

	ROS_INFO("Best mag:\t %f", best_val_mag);

        float hm = 58; //Constante que determina o nivel da perda de velocidade
        twist_teleop.linear.x *= (1 - fmin(best_val_mag, hm)/hm); //Controla a velocidade do robo
        ROS_INFO("X:\t %f", twist_teleop.linear.x);
    }
    
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
    //    int amostra = 0;
    //    ROS_INFO("I heard:");
    //    ROS_INFO("MIN[%d]: [%lf]", amostra, scan.ranges[amostra]); // -2.355 rad
    //
    //    amostra = (int)floor(numero_amostras/6);
    //    ROS_INFO("---[%d]: [%lf]", amostra,  scan.ranges[amostra]); // -1.57 rad
    //
    //    amostra = (int)floor(numero_amostras/3);
    //    ROS_INFO("---[%d]: [%lf]", amostra,  scan.ranges[amostra]); // -0.785 rad
    //
    //    amostra = (int)floor(numero_amostras/2);
    //    ROS_INFO("MED[%d]: [%lf]", amostra,  scan.ranges[amostra]); // 0 rad
    //
    //    amostra = (int)floor(2 * numero_amostras/3);
    //    ROS_INFO("+++[%d]: [%lf]", amostra,  scan.ranges[amostra]); // +0.785 rad
    //
    //    amostra = (int)floor(5 * numero_amostras/6);
    //    ROS_INFO("+++[%d]: [%lf]", amostra,  scan.ranges[amostra]); // +1.57 rad
    //
    //    amostra = numero_amostras;
    //    ROS_INFO("MAX[%d]: [%lf]", amostra,  scan.ranges[amostra]); // +2.355 rad
    
    laserScanBuffer.push_back(scan);
    if(laserScanBuffer.size() >= 40)
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

    //Armazena a leitura atual
    // scan_mem = scan;
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


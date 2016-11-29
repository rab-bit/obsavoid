#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <chrono>

#define BUBLE_SIZE_FRONT .5 //< Tamanho da bolha na parte frontal
#define BUBLE_SIZE_SIDE .20 //< Tamanho da bolha na parte lateral

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

double angulo_global = 0;
double velocidade = 0;
unsigned long time_g = 0;
bool rotina_retorno;

bool hasSpace (double angulo, sensor_msgs::LaserScan msg);
int getDirection(double angulo_gb);


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
        if(i > numero_amostras/3 && i < 2*numero_amostras/3){ //45 graus - Deteccao frontal
            if (msg.ranges[i] < BUBLE_SIZE_FRONT) return true;
        }
        else if(i > numero_amostras/6 && i < 5*numero_amostras/6) //90 graus - Deteccao lateral
            if (msg.ranges[i] < BUBLE_SIZE_SIDE) return true;
    }
    return false;
}
/*********************************************************************************
 * Recebe informacoes dos sensores e de direcao. Publica comandos de velocidade
 * evitando bater em obstaculos.
 **/
void obstacleAvoidanceControl(geometry_msgs::Twist twist_teleop){

	unsigned long now = static_cast<long int>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
	
    
    if (scan_mem_active == 0) return; //Verifica se a variavel scan_mem ja foi inicializada
    
    
    //Enquanto houver obstaculos, recalcula a tragetoria de forma ir para o angulo com o maior range
    if (checkForObstacles(scan_mem)) {
        ROS_INFO("Obstaculo!!");
        //Computa novo angulo (desviar)
        float soma_ang = 0.0;
        float soma_rang = 0.0;
        float nearest = 40.0;
        sensor_msgs::LaserScan msg = scan_mem;
        
        //A cada 10 amostras
        for (float alpha = scan_mem.angle_min; alpha < msg.angle_max; alpha += msg.angle_increment*10){
            
            float obstacle_prox = getDistanceAverage(alpha, msg, 2); //Quanto maior o range, mais perto o obstaculo
            
            soma_ang += alpha * obstacle_prox; //Calcula a soma dos angulos multiplicados aos valores de range
            soma_rang += obstacle_prox; //Calcula soma dos valores de range
            
            if (obstacle_prox < nearest) nearest = obstacle_prox;
        }
        
        float rebound_angle = soma_ang/soma_rang; //Angulo de desvio
        ROS_INFO("R Angle: [%f]", rebound_angle);
        ROS_INFO("Nearest: [%f]", nearest);

	//now e igual ao valor em microssegundos do computador
	angulo_global += velocidade * (double)(now - time_g); //acertar as grandezas escalares
	velocidade = rebound_angle *2;
	time_g = now;
	
	 ROS_INFO("Angulo: [%lf]", angulo_global);
        twist_teleop.linear.x = 0; //Para o robo
        twist_teleop.angular.z = rebound_angle * 2; //Ajusta o angulo

        //pub.publish(twist_teleop);//
    } else{
	//now e igual ao valor em microssegundos do computador
	angulo_global += velocidade * (now - time_g); //acertar as grandezas escalares
	velocidade = 0;
	time_g = now;
	
	 if(abs(angulo_global) > 0){
		ROS_INFO("Tento retornar!!");
		
		if(hasSpace(angulo_global, scan_mem)){
			twist_teleop.angular.z = getDirection(angulo_global); //Ajusta o angulo	
			velocidade = twist_teleop.angular.z;		
		}
		
	}
    }    
    //Publica twist para o cmd_vel robo
    pub.publish(twist_teleop);
}
bool hasSpace (double angulo, sensor_msgs::LaserScan msg) {
    
    int posicao_array = (int) floor((angulo - msg.angle_min)/msg.angle_increment);
        if(posicao_array < 3)
	posicao_array = 3;

    float sum = msg.ranges[posicao_array]; //Central

    for(int i = 1; i <= 3; i++){
        sum += msg.ranges[posicao_array + i];
        sum += msg.ranges[posicao_array - i];
    }
    
    //Calcula media dos ranges
    if ((sum/7) > BUBLE_SIZE_SIDE) 
	return true;
    return false;

} 
int getDirection(double angulo_gb) {
	if(angulo_gb > 0) 
		return -0.2;
	return 0.2;
	
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
    ROS_INFO("%f\t%f", scan.range_min, scan.range_max);
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

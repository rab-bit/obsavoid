#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <math.h>
#include <vector>

#define BUBLE_SIZE .7 //< Tamanho da bolha
#define MAX_SPEED 1
#define MIN_SPEED 0

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
bool checkForObstacles(sensor_msgs::LaserScan msg, float limite){
    
    int numero_amostras = (int) floor((msg.angle_max - msg.angle_min) / msg.angle_increment);
    
    //Le as amostras a cada 10 unidades
    for (int i = 0; i < numero_amostras; i+= 10){
        if(i > numero_amostras/6 && i < 5*numero_amostras/6) //90 graus
            if (msg.ranges[i] < limite) return true;
    }
    return false;
}


/*********************************************************************************
 * Calcula em um conjunto de angulos, o mais proximo de um angulo determinado.
 *
 **/
float nearestAngle(float angle, float* angles, int num_angles){
    
    float nearest = 999;

    for (int i = 0; i < num_angles; i++)
        if(fabs(angles[i] - angle) < fabs(nearest - angle))
            nearest = angles[i];

    return nearest;
}

/*********************************************************************************
 * Recebe informacoes dos sensores e de direcao. Publica comandos de velocidade
 * evitando bater em obstaculos.
 **/
void obstacleAvoidanceControl(geometry_msgs::Twist twist_teleop){
    
    if (scan_mem_active == 0) return; //Verifica se a variavel scan_mem ja foi inicializada
    
    //Filtra velocidade maxima (e impede de andar no sentido negativo)
    twist_teleop.linear.x = fmax(0, fmin(MAX_SPEED, twist_teleop.linear.x));
    
    //Armazena dados de leitura
    sensor_msgs::LaserScan msg = scan_mem;
    
    //Calcula raio da bolha de acordo com a velocidade do robo
    float tam_bolha = fmax(BUBLE_SIZE , BUBLE_SIZE * twist_teleop.linear.x);
    
    
    //Enquanto houver obstaculos, recalcula a tragetoria de forma ir para o angulo com o maior range
    if (checkForObstacles(msg, tam_bolha)){
        
        //Computa novo angulo (desviar)
        float angulo_setor = 0.0872665; //< Angulo por setor (5graus)
        float angulo_abertura = 1.5708; //< Angulo maximo de abertura do laser para cada lado (90 graus)
        float a = 60; //Magnitude maxima
        float b = 2; // 0 = a - 30b
        float limite = 60 - tam_bolha*2; //Limite para detectar vales
        int s_max = 8; //Numero de setores livres consecutivos para o robo passar
        int sec_counter = 0; //Contador de setores por vale
        float best_val_ang = (-1)*angulo_abertura; //Vetor central do melhor vale escolhido
        float best_val_mag = 0; //Magnitude do melhor vale
        
        float vales[(int)ceil(2*angulo_abertura/angulo_setor)]; //Resultado dos vales
        int num_vales = 0;
        
        //---------------------------------------------------------------------
        //A cada setor de -angulo_abertura a angulo_abertura, verifica os possiveis vales
        for (float alpha = (-1)*angulo_abertura; alpha < angulo_abertura; alpha += angulo_setor){
            
            float c = 1; //Probabilidade de haver um obstaculo no setor
            float dist_setor = getDistanceAverage(alpha, msg, 5); //Range do setor atual
            float m = pow(c,2) * (a - b*dist_setor); //Calcula magnitude
            
            //Verifica possiveis vales
            //Se o valor atual for maior que o limite (ou percorreu todas as amostras)
            if (m > limite || alpha+angulo_setor >= angulo_abertura){
                if (sec_counter >= s_max) //Se o vale tiver o numero minimo de setores
                    vales[num_vales++] = alpha - (sec_counter * angulo_setor)/2; //Calcula angulo central resultante do vale
                
                //Reseta o contador
                sec_counter = 0;
            }
            else sec_counter++;
        }
        
        //---------------------------------------------------------------------
        //Se nao encontrou nenhum vale
        if (num_vales == 0){
            twist_teleop.linear.x = 0; //Fica parado
            twist_teleop.angular.z = -1; //E girando em torno do proprio eixo
        }
        
        //---------------------------------------------------------------------
        //Encontrou vale(s)
        else {
            //Escolhe o angulo referente ao vale mais proximo
            twist_teleop.angular.z = nearestAngle(twist_teleop.angular.z, vales, num_vales);
            
            //Controle da velocidade
            float c = 1; //Probabilidade de haver um obstaculo no setor
            float dist_vale = getDistanceAverage(twist_teleop.angular.z, msg, 5); //Range do vale escolhido
            float m = pow(c,2) * (a - b*dist_vale); //Calcula magnitude
            
            float hm = 158; //Constante que determina o nivel da perda de velocidade
            twist_teleop.linear.x *= (1 - fmin(m, hm)/hm); //Controla a velocidade do robo pela magnitude do vale
            
            //Controla a velocidade pela diferenca angular
            twist_teleop.linear.x = twist_teleop.linear.x * (1 - fabs(1.5*twist_teleop.angular.z)/angulo_abertura) + MIN_SPEED;
        }
    }
    //Publica twist para o cmd_vel robo
    pub.publish(twist_teleop);
}


/*********************************************************************************
 * Callback da escuta de informacoes de velocidade
 **/
void teleopCallback(geometry_msgs::Twist twist_teleop)
{
    //Altera o twist devido a obstaculos TODO
    obstacleAvoidanceControl(twist_teleop);
}


/*********************************************************************************
 * Callback da escuta de informacoes do laser
 **/
void laserCallback(sensor_msgs::LaserScan scan)
{
    
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
    
    //Informa inicializacao da scan_mem
    scan_mem_active = 1;
    
}


/*********************************************************************************
 **/
int main(int argc, char **argv)
{
    //Inicializa o noh
    ros::init(argc, argv, "obstacle_avoidance");
    
    ros::NodeHandle n;
    
    //Define as publicacoes de twist para o cmd_vel do robo
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    
    //Define a escuta do topico com as medicoes do laser
    sub_laser = n.subscribe("/hokuyo_scan", 100, laserCallback);
    //Define a escuta do topico com as velocidades fornecidas pelo turtlebot
    sub = n.subscribe("/cmd_vel_mux/input/teleop", 100, teleopCallback);
    
    ros::spin();
    
    return 0;
}


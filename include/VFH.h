#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>

#define MIN_SPEED 0

//--------------------------------------------------------------------------------------------------
/*!
 *
 * Classe responsavel pela implentacao dos metodos e atributos responsaveis pelo calculo de desvio
 * de obstaculos utilizando o algoritmo VFH.
 *
 */
class VFH {
    
private:
    
    float angulo_setor = 0.0872665; //< Angulo por setor (5graus)
    float angulo_abertura = 1.5708; //< Angulo maximo de abertura do laser para cada lado (90 graus)
    float a = 60; //Magnitude maxima
    float b = 2; // 0 = a - 30b
    float limite = 59; //Limite para detectar vales
    int s_max = 8; //Numero de setores livres consecutivos para o robo passar
    float hm = 158; //Constante que determina o nivel da perda de velocidade
    
public:
    
    //--------------------------------------------------------------------------------------------------
    /*!
     * Metodo construtor da classe VFH. Recebe os parametros de configuracao do algoritmo
     *
     * \param angulo_setor Angulo de cada setor
     * \param angulo_abertura Angulo maximo de leitura do laser
     * \param limite Magnitude limite para o setor ser considerado um vale
     * \param a Valor da magnitude maxima
     * \param b 0 = a - 30*(range_max)
     * \param s_max Numero de setores livres consecutivos ser considerado um vale "largo"
     */
    VFH(float angulo_setor, float angulo_abertura, float limite, float a, float b, int s_max);
    
    //--------------------------------------------------------------------------------------------------
    /*!
     * Metodo construtor da classe VFH. Utiliza parametros padroes.
     *
    */
    VFH(){}
    
    //--------------------------------------------------------------------------------------------------
    /*!
     * Metodo responsavel por calcular o vetor resultante para as velocidades linear e angular, dado um
     * vetor de ranges de leitura do laser e o vetor de velocidade de entrada.
     *
     * \param twist_teleop Vetor de velocidade de entrada
     * \param msg Mensagem de leitura do laser
     *
     * \return Vetor resultante
     */
    geometry_msgs::Twist calculate(geometry_msgs::Twist twist_teleop, sensor_msgs::LaserScan msg);
    
    
    //--------------------------------------------------------------------------------------------------
    //
    inline void setLimite(float limite){
        this->limite = limite;
    }

private:
    
    //--------------------------------------------------------------------------------------------------
    /*!
     * Recebe o angulo central de uma amostra, a mensagem do laser e o numero de
     * amostras vizinhas ao angulo recebido que serao utilizadas para o calculo de um
     * range medio.
     *
     * \param angulo Angulo central
     * \param msg Mensagem de leitura do laser
     * \param num_amostras Numero de amostras vizinhas
     * 
     * \return Media dos ranges das amostras
     */
    float getDistanceAverage(float angulo, sensor_msgs::LaserScan msg, int num_amostras);
    
    //--------------------------------------------------------------------------------------------------
    /*!
     * Calcula em um conjunto de angulos, qual o mais proximo de um angulo determinado.
     *
     * \param angle Angulo principal
     * \param angles Vetor de angulos
     * \param num_angles Numero de elemtos no vetor de angulos
     *
     * \return Media dos ranges das amostras
     */
    float nearestAngle(float angle, float* angles, int num_angles);
    
    //--------------------------------------------------------------------------------------------------
    /*!
     * Busca um conjunto de vales para uma determinada quantidade de amostras de range
     *
     * \param angles Vetor a receber os valores dos angulos centrais dos vales
     * \param msg Mensagem de leitura do laser
     *
     * \return Numero de vales
     */
    int getVales(float* vales, sensor_msgs::LaserScan msg);
    

};

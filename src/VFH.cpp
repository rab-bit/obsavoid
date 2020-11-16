#include "VFH.h"
#include <math.h>

//--------------------------------------------------------------------------------------------------
//
VFH::VFH(float angulo_setor, float angulo_abertura, float limite, float a, float b, int s_max){
    this->angulo_setor = angulo_setor;
    this->angulo_abertura = angulo_abertura;
    this->limite = limite;
    this->a = a;
    this->b = b;
    this->s_max = s_max;
}


//--------------------------------------------------------------------------------------------------
//
float VFH::getDistanceAverage(float angulo, sensor_msgs::LaserScan msg, int num_amostras){
    
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


//--------------------------------------------------------------------------------------------------
//
float VFH::nearestAngle(float angle, float* angles, int num_angles){
    
    float nearest = 999;
    
    for (int i = 0; i < num_angles; i++)
        if(fabs(angles[i] - angle) < fabs(nearest - angle))
            nearest = angles[i];
    
    return nearest;
}


//--------------------------------------------------------------------------------------------------
//
int VFH::getVales(float* vales, sensor_msgs::LaserScan msg){
    
    int sec_counter = 0; //Contador de setores por vale
    int num_vales = 0; //Contador de setores por vale
    
    //A cada setor de -angulo_abertura a angulo_abertura, verifica os possiveis vales
    for (float alpha = (-1)*angulo_abertura; alpha < angulo_abertura; alpha += angulo_setor){
        
        float c = 1; //Probabilidade de haver um obstaculo no setor
        float dist_setor = getDistanceAverage(alpha, msg, 5); //Range do setor atual
        float m = pow(c,2) * (a - b * dist_setor); //Calcula magnitude
        
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
    return num_vales;
}


//--------------------------------------------------------------------------------------------------
//
geometry_msgs::Twist VFH::calculate(geometry_msgs::Twist twist_teleop, sensor_msgs::LaserScan msg){
    
    float best_val_ang = (-1)*angulo_abertura; //Vetor central do melhor vale escolhido
    float vales[(int)ceil(2*angulo_abertura/angulo_setor)]; //Resultado dos vales
    int num_vales;
    
    //Obtem os possiveis vales
    num_vales = getVales(vales, msg);
    
    //Se nao encontrou nenhum vale
    if (num_vales == 0){
        twist_teleop.linear.x = 0; //Fica parado
        twist_teleop.angular.z = -1; //E girando em torno do proprio eixo
    }
    
    //Encontrou vale(s)
    else {
        //Escolhe o angulo referente ao vale mais proximo
        twist_teleop.angular.z = nearestAngle(twist_teleop.angular.z, vales, num_vales);
        
        //Controle da velocidade
        float c = 1; //Probabilidade de haver um obstaculo no setor
        float dist_vale = getDistanceAverage(twist_teleop.angular.z, msg, 5); //Range do vale escolhido
        float m = pow(c,2) * (a - b*dist_vale); //Calcula magnitude
        
        twist_teleop.linear.x *= (1 - fmin(m, hm)/hm); //Controla a velocidade do robo pela magnitude do vale
        
        //Controla a velocidade pela diferenca angular
        twist_teleop.linear.x = twist_teleop.linear.x *
            (1 - fabs(1.5 * twist_teleop.angular.z)/angulo_abertura) + MIN_SPEED;
    }
    
    //Retorna o vetor resultante
    return twist_teleop;
}

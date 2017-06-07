#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <signal.h>
#include "sensor_msgs/LaserScan.h"
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <iostream>

#include "controller.h"
#include "main.h"

//#include "neuralPosition.h"



static const int amountOfPositions = 10;

//double descriptor[sizeOfDescript][numberOfspot][numberOfScales];

//double descriptor[sizeOfDescript][numberOfspot][numberOfScales] = {0};
///normal distributions:
///{0.0001, 0.0044, 0.0540, 0.2420, 0.3989, 0.2420, 0.0540, 0.0044, 0.0001};
/// {0.111, 0.111, 0.111, 0.111, 0.112, 0.111, 0.111, 0.111, 0.111};

double NeuralPosition::gaussDistribSensor[gaussDisbSensLength] = {1}; //{0.111, 0.111, 0.111, 0.111, 0.112, 0.111, 0.111, 0.111, 0.111};
double NeuralPosition::gaussDistribPlaCells[gaussDisbPlaCelLength] = {1}; /*{0.25,0.5,0.25};/*{0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 0.0579, 0.0666, 0.0737,
                                                                      0.0782, 0.0798, 0.0782, 0.0737, 0.0666, 0.0579, 0.0484, 0.0388, 0.0299,
                                                                      0.0222, 0.0158, 0.0108 };
/*
{0.0108, 0.0158, 0.0222, 0.0299, 0.0388, 0.0484, 0.0579, 0.0666, 0.0737,
                                                                      0.0782, 0.0798, 0.0782, 0.0737, 0.0666, 0.0579, 0.0484, 0.0388, 0.0299,
                                                                      0.0222, 0.0158, 0.0108 };
 */



NeuralPosition::NeuralPosition(){

    std::fill_n(gaussDistribSensor, gaussDisbSensLength, 1);
    std::fill_n(gaussDistribPlaCells, gaussDisbPlaCelLength, 1);

    for(int i = 0; i < sizeOfDescript; i++){
        for(int j = 0; j < numberOfspot; j++){
            for(int k = 0; k++; k < numberOfScales){
                descriptor[i][j][k] = 0;
            }
        }
    }
    highestScaleBuild = 0;



/*    for(int i = 0; i < sizeOfDescript; i++){
        descriptor[i][sample(ranges[i])][0] = 1;
    }*/
}

void NeuralPosition::buildDescri(double ranges[]){
    //std::cout << descriptor << std::endl;

    for(int i = 0; i < sizeOfDescript; i++){
        for(int j = 0; j < numberOfspot; j++){
            for(int k = 0; k++; k < numberOfScales){
                descriptor[i][j][k] = 0;
            }
        }
    }

    for(int i = 0; i < sizeOfDescript; i++){
        for(int j = 0; j < numberOfspot; j++){
            for(int k = 0; k++; k < numberOfScales){
                descriptor[i][j][k] = 0;
            }
        }
    }
    highestScaleBuild = 0;

    int sampled = 0;
    for(int i = 0; i < sizeOfDescript; i++){
        sampled = sample(ranges[i]);
        //descriptor[i][sample(ranges[i])][0] = 1;
        int k = - gaussDisbSensLength / 2;
        if(sampled > -k){                   // catch the whole thing
            for(int j = 0; j < gaussDisbSensLength; j++){
                descriptor[i][sampled+k][0] = gaussDistribSensor[j];
                k++;
            }
        }

    }
}


int NeuralPosition::sample(double a){
    int place = a / precision;

    if(numberOfspot < place){
        place = 0;
        std::cout << place << "error place set to 0";
    }
    std::cout << place << ", ";

    highestScaleBuild = 0;
    return place;
}


//* just print, as there are many many zero it is a bit strange but schould be fine
void NeuralPosition::print(int scale){

    for(int i = 0; i < sizeOfDescript; i++){
        std::cout << "[";

        for(int j = 0; j < numberOfspot; j++){
            if(descriptor[i][j][scale] != 0){
                std::cout << descriptor[i][j][scale] <<"(" << i << "," << j <<"),";
            }
        }
        std::cout << "] \n";
    }
    std::cout << std::endl;
}



///build higher scales:


void NeuralPosition::buildNextScale(){
/*
    for(int i = 0; i < sizeOfDescript; i++){
        for(int j = 1; j < numberOfspot-1; j++){
            descriptor[i][j][highestScaleBuild+1] = 0;
        }
    }
/*
 * for each value multiply [1/4 1/2 1/4] (for ex.) and add it to the result array
 *
 *
 */
   // std::cout << highestScaleBuild << "->" << highestScaleBuild+1 << " \n";

    for(int i = 0; i < sizeOfDescript; i++){
        for(int j = 1; j < numberOfspot-1; j++){

            int k = - gaussDisbPlaCelLength / 2;
            if(j > -k){                   // catch the whole thing
                for(int z = 0; z < gaussDisbPlaCelLength; z++){
                    descriptor[i][j+k][highestScaleBuild+1] +=(double) gaussDistribPlaCells[z] * (double)descriptor[i][j][highestScaleBuild];
                    k++;
                }
             //   std::cout << (double) gaussDistribPlaCells[5] * (double)descriptor[i][j][highestScaleBuild] << " \n";
            }
        }
    }

    highestScaleBuild++;
}


/*  descriptor[i][j-1][highestScaleBuild+1] += 0.25 * descriptor[i][j][highestScaleBuild];
  descriptor[i][j][highestScaleBuild+1] += 0.5 * descriptor[i][j][highestScaleBuild];
  descriptor[i][j+1][highestScaleBuild+1] += 0.25 * descriptor[i][j][highestScaleBuild];*/


void NeuralPosition::buildAllScales(){
    for(int i = highestScaleBuild; i < numberOfScales - 1; i++){
        buildNextScale();
    }
}






/// compare to descriptors
double NeuralPosition::computeSimilarity(NeuralPosition desrciA, int scaleComp){
    double similarity = 0;
    double similarityTmp = 0;
    double total = 0;

    double a, b;

    int similarityOnThisRay = 0;
    int numberOfSimilarRays = 0;

    for(int i = 0; i < sizeOfDescript; i++){
        for(int j = 0; j < numberOfspot; j++){
            a = desrciA.descriptor[i][j][scaleComp];
            b = descriptor[i][j][scaleComp];

            if(a != 0 && b != 0){
                similarityTmp += a;
                similarityTmp += b;
                similarityOnThisRay++;
            }
            //similarity += (double)desrciA.descriptor[i][j][scaleComp];
            //similarity += (double)descriptor[i][j][scaleComp];
            total += a;
            total += b;
        }
        similarityTmp = similarityTmp / total;
        similarity += similarityTmp;

        if(similarityTmp >= 0.5){numberOfSimilarRays++;}

        similarityTmp = 0;
        total = 0;

        //if(similarityOnThisRay){numberOfSimilarRays++;}
       // if(similarityOnThisRay >= gaussDisbSensLength / 2){numberOfSimilarRays++;}
        similarityOnThisRay = 0;
    }

    similarity = similarity / (double)sizeOfDescript;

    //return similarity;
    return numberOfSimilarRays / (double)sizeOfDescript;
}


















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

    highestScaleBuild = 0;

    int sampled = 0;

    for(int i = 0; i < sizeOfDescript; i++){
        rangesP[i] = ranges[i];
        sampled = sample(ranges[i]);
        int k = - gaussDisbSensLength / 2;

        if(sampled > -k){                   // catch the whole thing
#ifdef simplified
    descriptor[i][0][0] = sampled + k;
    descriptor[i][1][0] = sampled - k;
#else
            for(int j = 0; j < gaussDisbSensLength; j++){
                descriptor[i][sampled+k][0] = gaussDistribSensor[j];
                k++;
            }
#endif
        }

    }

    extractAnomalies();

}


int NeuralPosition::sample(double a){
    int place = a / precision;

#ifdef simplified
#else
    if(numberOfspot < place){
        place = 0;
        std::cout << place << "error place set to 0";
    }
#endif
    std::cout << place << ", ";

    highestScaleBuild = 0;
    return place;
}


//* just print, as there are many many zero it is a bit strange but schould be fine
void NeuralPosition::print(int scale){

    for(int i = 0; i < sizeOfDescript; i++){

        std::cout << "[";
#ifdef simplified

        std::cout << descriptor[i][0][0] << "," << descriptor[i][1][0];
#else
        for(int j = 0; j < numberOfspot; j++){
            if(descriptor[i][j][scale] != 0){
                std::cout << descriptor[i][j][scale] <<"(" << i << "," << j <<"),";
            }
        }
 #endif
        std::cout << "] \n";

}
    std::cout << std::endl;
}



///build higher scales:


void NeuralPosition::buildNextScale(){

    int k = - gaussDisbPlaCelLength / 2;

    for(int i = 0; i < sizeOfDescript; i++){

#ifdef simplified
    descriptor[i][0][highestScaleBuild+1] = descriptor[i][0][highestScaleBuild] + k;
    descriptor[i][1][highestScaleBuild+1] = descriptor[i][1][highestScaleBuild] - k;

#else
        for(int j = 1; j < numberOfspot-1; j++){

            if(j > -k){                   // catch the whole thing
                for(int z = 0; z < gaussDisbPlaCelLength; z++){
                    descriptor[i][j+k][highestScaleBuild+1] +=(double) gaussDistribPlaCells[z] * (double)descriptor[i][j][highestScaleBuild];
                    k++;
                }
             //   std::cout << (double) gaussDistribPlaCells[5] * (double)descriptor[i][j][highestScaleBuild] << " \n";
            }
        }
 #endif

}
    highestScaleBuild++;
}



void NeuralPosition::buildAllScales(){
    for(int i = highestScaleBuild; i < numberOfScales - 1; i++){
        buildNextScale();
    }
}





/// compare two descriptors
double NeuralPosition::computeSimilarity(NeuralPosition desrciA, int scaleComp){
    double similarity = 0;
    double similarityTmp = 0;
    double total = 0;

    double a, b;

    int similarityOnThisRay = 0;
    int numberOfSimilarRays = 0;


    double deriveTreshold = 3;                              ///TODO : nearest neighbor mechanism
                                                            ///-> not optimal now


    std::vector<int> raysToignor(sizeOfDescript);
    double minimum_dist = 100;
    int placeOfBestMach = 0;

    std::vector<int> takenPlaces(3,sizeOfDescript+1);
    int taken = 12;

    //find similar anomalies:
    int i = 0;
    std::list<AnomalyZone>::iterator it, it2 = desrciA.anomalyList.begin();
    for(it = anomalyList.begin(); it!=anomalyList.end(); ++it, ++it2){
        AnomalyZone anom1 = *it;

        minimum_dist = 10 * sizeOfDescript;    i = 0;

        for(it2 = desrciA.anomalyList.begin(); it2!=desrciA.anomalyList.end(); ++it2){
             AnomalyZone anom2 = *it2;

             double dist = sqrt(pow(fabs(anom1.highestDerivate - anom2.highestDerivate),2)+pow(computeEcart(anom1.startIndex, anom2.startIndex),2));

             if(dist < minimum_dist && taken != i && !(std::find(takenPlaces.begin(), takenPlaces.end(), i) != takenPlaces.end())){
                std::cout << taken << "  " << i << "  "<< dist << "  "<< minimum_dist << "  \n";
                minimum_dist = dist;   placeOfBestMach = i;
             }
             i++;
        }


        takenPlaces.push_back(placeOfBestMach);
        taken = placeOfBestMach;

       // if(minimum_dist < 2+scaleComp){                  //TODO: could get ride of this if always truepositiv in anomaly detection
            it2 = desrciA.anomalyList.begin();
            for(i = 0; i < placeOfBestMach; i++, ++it2){}

            AnomalyZone anom2 = *(it2);           // +1 ??

            std::cout   << "\n match! ";
            for(int i = 0; i < anom1.length; i++){
             //   if(!(std::find(raysToignor.begin(), raysToignor.end(), anom1.startIndex + i) != raysToignor.end())){
                    raysToignor.push_back(anom1.startIndex + i);
                    std::cout   << ", " << anom1.startIndex + i;
               // }
            }
            for(int i = 0; i < anom2.length; i++){
                //if(!(std::find(raysToignor.begin(), raysToignor.end(), anom2.startIndex + i) != raysToignor.end())){
                    raysToignor.push_back(anom2.startIndex + i);
                    std::cout   << "; " << anom2.startIndex + i;
                    if(anom2.startIndex + i > sizeOfDescript){ break; }
                //}
            }
            std::cout   << "" << std::endl;

        //}

        /*    if(fabs(anom1.highestDerivate - anom2.highestDerivate) < deriveTreshold){
                if(computeEcart(anom1.startIndex, anom2.startIndex)  < scaleComp+1){     //abs(anom1.startIndex - anom2.startIndex)
                    std::cout   << "\n match! ";
                    for(int i = 0; i < anom1.length; i++){
                        if(!(std::find(raysToignor.begin(), raysToignor.end(), anom1.startIndex + i) != raysToignor.end())){
                            raysToignor.push_back(anom1.startIndex + i);
                            std::cout   << ", " << anom1.startIndex + i;
                        }
                    }
                    for(int i = 0; i < anom2.length; i++){
                        if(!(std::find(raysToignor.begin(), raysToignor.end(), anom2.startIndex + i) != raysToignor.end())){
                            raysToignor.push_back(anom2.startIndex + i);
                            std::cout   << ", " << anom2.startIndex + i;
                        }
                    }

                    std::cout   << "\n end of the adding " << std::endl;
                }
            }

        }*/

    }

    /// this is most definitly not the best way to do it        */
    /// This will be corected soon, but it is not that bad since two matching anomalies should be
    /// aproximatly at the same place




    for(int i = 0; i < sizeOfDescript; i++){

#ifdef simplified

        int untGrenzOber, obGrenzOber, untGrenzUnter, obGrenzUnter;

        if(desrciA.descriptor[i][0][scaleComp] < descriptor[i][0][scaleComp]){
            untGrenzUnter = desrciA.descriptor[i][0][scaleComp];
            obGrenzUnter = desrciA.descriptor[i][1][scaleComp];

            untGrenzOber  = descriptor[i][0][scaleComp];
            obGrenzOber  = descriptor[i][1][scaleComp];
        } else {
            untGrenzUnter = descriptor[i][0][scaleComp];
            obGrenzUnter = descriptor[i][1][scaleComp];

            untGrenzOber  = desrciA.descriptor[i][0][scaleComp];
            obGrenzOber  = desrciA.descriptor[i][1][scaleComp];
        }


    int mitteU = (obGrenzUnter + untGrenzUnter) / 2;

    if(untGrenzOber < mitteU){
        numberOfSimilarRays++;
    } else if(std::find(raysToignor.begin(), raysToignor.end(), i) != raysToignor.end() || i == 0){
        numberOfSimilarRays++;
    }



#else
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
#endif
}
    similarity = similarity / (double)sizeOfDescript;

    //return similarity;
    return numberOfSimilarRays / (double)sizeOfDescript;
}






void NeuralPosition::extractAnomalies(){
    double averageDerivate = 0;
    double derivate = 0;
    int second = 0;
    double threshold = 3; //gaussDisbSensLength / 200;

    for(int i = 0; i < sizeOfDescript; i++){
        if(i != sizeOfDescript-1){
            second = i+1;
        } else {
            second = 0;
        }

        derivate = fabs(rangesP[i] - rangesP[second]);

        if(derivate > threshold){
            double rays[2] = {rangesP[i], rangesP[second]};
            AnomalyZone newAn = AnomalyZone(rays, i, 2);
            anomalyList.push_back(newAn);


            std::cout   << "\n deviate: " << derivate << "anomaly added!!" << rangesP[second] << " at: " << i << std::endl;
        }
    }

}










int NeuralPosition::computeEcart(int a, int b){
    if(a > b){
        int tmp = a;
        a = b;
        b = tmp;
    }

    int firstPos = abs(sizeOfDescript +1 - b) + abs(a - 0);

    int secondPos = abs(a - b);


    if(firstPos > secondPos){
      //  std::cout << "                                      " << secondPos << " from:" << a <<" " << b << std::endl;
        return secondPos;
    } else {
      //  std::cout << "                                      " << firstPos << "from: " << a << " " << b << std::endl;
        return firstPos;
    }

}































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


#include "main.h"
#include "anomalyZone.h"


AnomalyZone::AnomalyZone(double rays[], int begi, int leng){
    highestDerivate = 0;
    int j = 0;

    for(int i = 0; i < leng; i++){
        j= i+begi;
        rayDist[i] = rays[j%sizeOfDescript];
    }

    startIndex = begi;
    endIndex = begi + leng;
    length = leng;

    for(int i = 0; i < leng - 1; i++){
        derivates[i] = abs(rayDist[i] - rayDist[i+1]);

        if(derivates[i] > highestDerivate){highestDerivate = derivates[i];}

    }



}


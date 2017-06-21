#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <signal.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/ModelStates.h"
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <gazebo_msgs/SetModelState.h>

#include "controller.h"


//#include "neuralPosition.h"

using namespace std;

//Controller contr;
//const int sizeOfDescript = contr.sizeOfDescript;


/*
static const int amountOfPositions = 10;
double static ranges[sizeOfDescript];

double static positions[amountOfPositions][sizeOfDescript];
*/

double Controller::ranges[sizeOfDescript] = {0};
double Controller::positions[amountOfPositions][sizeOfDescript] = {0};

double Controller::truePositions[amountOfPositions][2] = {0};


int positionIndex = 0;

double orientationWcos = 0;
double orientationZsin = 0;

double orientGlobalWcos = 0;
double orientGlobalZsin = 0;
double orientGlobalX = 0;
double orientGlobalY = 0;

double trueGlobPosX = 0;
double trueGlobPosY = 0;

int numberOfLoop = 0;



NeuralPosition Controller::neuralPos[amountOfPositions];

//NeuralPosition theTestPos = NeuralPosition();

Controller::Controller():
  ph_("~"),
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("my_robot/cmd_vel", 1);

  client = nhmod.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
}



void Controller::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && 
      (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    publish(0, 0);
}


void Controller::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;

    vel_pub_.publish(vel);    

  return;
}

/*
 *	sensor managment
 */

void Controller::startSensors(void){
    std::cout << "starting hokuyo!\n";

      ros::Subscriber subHokuyo = nSub.subscribe("/my_robot/laser/scan", 1000, getHokuyoVal);
      ros::Subscriber subOrientation = nSub.subscribe("/imu", 1000, getOrientation);
      ros::Subscriber subGlobOrientation = nSub.subscribe("/gazebo/model_states", 1000, getGlobalState);
      ros::spin();
}
void Controller::getHokuyoVal(const sensor_msgs::LaserScan laser){
    //ROS_INFO("size[%d]: ", laser.intensities.size());
    //std::cout << "size[%d]: " << laser.ranges.size();
    for (unsigned int i=0; i<sizeOfDescript;i++) //laser.ranges.size()
    {
        ranges[i] = laser.ranges[i];
        //ROS_INFO("intens[%f]: ", laser.intensities[i]);
        //std::cout << "intens[" << i << "]: " << ranges[i] << "\n";
    }
}
void Controller::printHokuyoRanges(void){

    for (unsigned int i=0; i<sizeOfDescript;i++)
    {
        std::cout << "range[" << i << "]: " << ranges[i] << "\n";
        //std::cout << "$"<< ranges[i] << "$ ";
    }

}


void Controller::getOrientation(const sensor_msgs::Imu imu){
    //std::cout << "value " << imu.orientation.w;
    orientationWcos = imu.orientation.w;
    orientationZsin = imu.orientation.z;
}
void Controller::getGlobalState(const gazebo_msgs::ModelStates state){
    //std::cout << "value " << imu.orientation.w;
    orientGlobalWcos = state.pose.at(1).orientation.w;
    orientGlobalZsin = state.pose.at(1).orientation.w;
    orientGlobalX = state.pose.at(1).orientation.x;
    orientGlobalY = state.pose.at(1).orientation.y;
    trueGlobPosX = state.pose.at(1).position.x;
    trueGlobPosY = state.pose.at(1).position.y;
}
void Controller::printOrientation(void){
    std::cout << "z: " << orientationZsin << "\n";
    std::cout << "w: " << orientationWcos << "\n";
}


/*
 *	mouvments managment
 */

void Controller::neunanteDegRot(void){
    ros::Rate r(10);
    int i = 0;
    while(i < 30){
        publish(-0.475, 0);     //0.475
        r.sleep();
        i++;
    }
    std::cout << "done rot!\n";

    while(i < 40){
        publish(0, 0);
        r.sleep();
        i++;
    }
    //std::cout << "stoped!\n";
}



void Controller::forwardOne(void){
    ros::Rate r(10);
    int i = 0;
    while(i < 30){
        publish(0, 0.475);
        r.sleep();
        i++;
    }
    std::cout << "done for!\n";

    while(i < 40){
        publish(0, 0);
        r.sleep();
        i++;
    }
    //std::cout << "stoped!\n";
}


// almost working
void Controller::forwardFor(double distance)
{
    double error = 1000;
    double actualRange = ranges[0];
    double goalRange = actualRange + distance;


    error = goalRange - actualRange;

    while(abs(error) > 0.005){
        publish(0,error/5); // /7

        //wait to see
        usleep(50000); //0.2 sec

        actualRange = ranges[0];

        error = goalRange - actualRange;

    }

    publish(0 , 0);
    publish(0 , 0);
    std::cout << "move done!\n";
}

// almost working
double Controller::turnTill(double goalAngle)
{
    double error = 1000;
    double actualAngle = 0;

    goalAngle = goalAngle * (PI / 180);
    //if(goalAngle >= PI){ goalAngle = goalAngle - 2 * PI; }

    actualAngle = acos(orientGlobalWcos);           //orientationWcos      or    orientGlobalWcos
    actualAngle = fmod(actualAngle, PI);

    actualAngle *= 2;
    //if(actualAngle >= PI){ actualAngle = actualAngle - 2 * PI; }
    //else if(actualAngle <= -PI){ actualAngle = actualAngle + 2 * PI; }

    error = goalAngle - actualAngle;
    if(error >= PI){ error = error - 2 * PI; }
    else if(error <= -PI){ error = error + 2 * PI; }

    while(abs(error) > 0.005){
        // compute where to go

      //  std::cout << "goalAngle: " << goalAngle << "\n";
        //std::cout << "actualAngle: " << actualAngle << "\n";
        //std::cout << "error: " << error << "\n";

        error = error; //abs(error);

        publish(error/5 , 0); // /7


        //wait to see
        usleep(50000); //0.2 sec
        //compute error
        actualAngle = acos(orientGlobalWcos);
        actualAngle = fmod(actualAngle, PI);

        actualAngle *= 2;

        error = goalAngle - actualAngle;
        /*if(actualAngle >= PI){ actualAngle = actualAngle - 2 * PI; }
        else if(actualAngle <= -PI){ actualAngle = actualAngle + 2 * PI; }

        if(actualAngle >= 0){ error = goalAngle - actualAngle;}
        else if(actualAngle <= -0){ error = actualAngle - goalAngle;  }

        // vielleicht nuetzlos
        if(error >= PI){ error = error - 2 * PI; }
        else if(error <= -PI){ error = error + 2 * PI; }*/


    }

    publish(0 , 0);
    publish(0 , 0);
    std::cout << "rot done!\n";
    return error;
}






/*
 *	positions managment
 */

void Controller::savePosition(void){

    double tmp[sizeOfDescript];
    double actualAngle = acos(orientGlobalWcos);
    int place = (actualAngle / (PI)) * sizeOfDescript;
    int k = place;
    
    for(int i = 0; i < sizeOfDescript; i++){
        positions[positionIndex][i] = ranges[i];  //ranges[sizeOfDescript-k];
        tmp[i] = ranges[i];  //ranges[sizeOfDescript-k];
        k++;
        if(k == sizeOfDescript){ k=0;}
    }

    //neuralPos[positionIndex] = NeuralPosition();
    neuralPos[positionIndex].buildDescri(tmp);

    //theTestPos = NeuralPosition();
    //theTestPos.buildDescri(tmp);

    truePositions[positionIndex][0] = trueGlobPosX;
    truePositions[positionIndex][1] = trueGlobPosY;

    std::cout << "begun at: " << place << " saved!\n";
    std::cout << "position " << positionIndex << " saved!\n";
    positionIndex++;
}

void Controller::printPosition(int pos)
{
    std::cout << "[";
    for(int i = 0; i < sizeOfDescript; i++){
        int a = positions[pos][i];
        if(a < 10){ std::cout << " ";}
        std::cout << a << ",";
    }
    std::cout  << "] \n";
}

double Controller::computeError(int posA, int posB){
    
    double error = 0;
    
    for(int i = 0; i < sizeOfDescript; i++){
	error += abs(positions[posA][i] - positions[posB][i]);
    }

    error = error / sizeOfDescript;

    std::cout << "error (" << posA << "," << posB << ") : " << error << " !\n";
    return error;
}

double Controller::computeRealDistance(int posA, int posB){
    double dist = 0;

    dist = sqrt(pow((truePositions[posA][0] - truePositions[posB][0]),2) + pow((truePositions[posA][1] - truePositions[posB][1]),2));

    std::cout << "dist (" << posA << "," << posB << ") : " << dist << " !\n";
    return dist;
}


 void Controller::newNeuronalPos(void){
     savePosition();
     //neuralPos[positionIndex-1].print(0);
     neuralPos[positionIndex-1].buildNextScale();
     //neuralPos[positionIndex-1].print(1);
     //neuralPos[positionIndex-1].print(2);
     neuralPos[positionIndex-1].buildNextScale();
     neuralPos[positionIndex-1].print(0);
     neuralPos[positionIndex-1].print(1);
     neuralPos[positionIndex-1].print(2);



     char nameFile[40] = "/home/franz/chabadabada#.txt";
     nameFile[6] = 3 + 48;
     ofstream myfile(nameFile);

     if (myfile.is_open()) {


              myfile << "Baam!.\n";


              myfile.close();
      } else cout << "Unable to open file";
 }



 void Controller::setNewPos(double x, double y){

     start_pose.position.x = x;
     start_pose.position.y = y;
     start_pose.position.z = 0.0;
     start_pose.orientation.x = 0.0;
     start_pose.orientation.y = 0.0;
     start_pose.orientation.z = 0.0;
     start_pose.orientation.w = 0.0;
     start_twist.linear.x = 0.0;
     start_twist.linear.y = 0.0;
     start_twist.linear.z = 0.0;
     start_twist.angular.x = 0.0;
     start_twist.angular.y = 0.0;
     start_twist.angular.z = 0.0;

     modelstate.model_name = (std::string) "my_robot";
     modelstate.reference_frame = (std::string) "world";
     modelstate.pose = start_pose;
     modelstate.twist = start_twist;

     setmodelstate.request.model_state = modelstate;
     client.call(setmodelstate);

 }




/*
 *  More complicated behaviour
 */

void Controller::oneLoop()
{
    std::cout << "lets do a loop!\n" << std::endl;
    int wating = 2000000;
    int startingPos = positionIndex;
    double distance = 0.25 + (double)positionIndex/(double)20;

    double losangeAngle = 0+(double)positionIndex;


    turnTill(0);
    savePosition();


    /// 1. movement
    turnTill(180 + losangeAngle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    /// 2. movement
    turnTill(270 - losangeAngle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    /// 3. movement
    turnTill(0 + losangeAngle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    /// 4. movement
    turnTill(90 - losangeAngle);
    forwardFor(distance);

    turnTill(0);
    savePosition();



    computeError(startingPos,startingPos );
    computeError(startingPos,1 + startingPos);
    computeError(startingPos,2 + startingPos);
    computeError(startingPos,3 + startingPos);
    computeError(startingPos,4 + startingPos);

/*  /*   printPosition(0 + numberOfLoop * 5);
    printPosition(1 + numberOfLoop * 5);
    printPosition(2 + numberOfLoop * 5);
    printPosition(3 + numberOfLoop * 5);
    printPosition(4 + numberOfLoop * 5);*/

 //   neuralPos[0].print(0);
 //   neuralPos[1].print(0);



   // std::cout << posa.computeSimilarity(posb,0) << std::endl;

    double tmpa = 0;

    std::cout << "\n distances:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        std::cout << computeRealDistance(0,i) << std::endl;
    }

    std::cout << "\n scale 0:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        tmpa = neuralPos[startingPos].computeSimilarity(neuralPos[i],0);
        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
    }

    for(int i = startingPos; i < 5 + startingPos; i++){
        neuralPos[i].buildNextScale();
    }

    std::cout << "\n scale 1:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        tmpa = neuralPos[startingPos].computeSimilarity(neuralPos[i],1);
        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
    }

    for(int i = startingPos; i < 5 + startingPos; i++){
        neuralPos[i].buildNextScale();
    }
    std::cout << "\n scale 2:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        tmpa = neuralPos[startingPos].computeSimilarity(neuralPos[i],2);
        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
    }


    /*neunanteDegRot();
    usleep(wating);
    neunanteDegRot();
    usleep(wating);
    neunanteDegRot();
    usleep(wating);

old:
error (0,0) : 0 !
error (0,1) : 0.59394 !
error (0,2) : 0.944974 !
error (0,3) : 0.558499 !
error (0,4) : 0.0723938 !


error (0,0) : 0 !
error (0,1) : 0.598049 !
error (0,2) : 0.949082 !
error (0,3) : 0.554391 !
error (0,4) : 0.0682854 !


new:

1
0.34375
0
0
0.3125

0.375
0.154297
0
0.0351562
0.185547

0.273438
0.119873
0ranges
0.0352783
0.153442


 scale 0:
1
0.28125
0
0.03125
0.25

 scale 1:
0.375
0.154297
0
0.015625
0.183594

 scale 2:
0.273438
0.122314
0.00012207
0.0185547
0.153442

*/
    //forwardOne();
   // usleep(wating);
}



void Controller::goForwardWithPoints(int angleI, double dist)
{
    std::cout << "lets move and measure!\n" << std::endl;
    int wating = 2000000;
    int startingPos = positionIndex;

    double distance = 0.25;
    distance = dist;

    double angle = angleI;

    turnTill(0);
    savePosition();


    /// 1. movement
    turnTill(angle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    /// 2. movement
    turnTill(angle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    /// 3. movement
    turnTill(angle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    /// 4. movement
    turnTill(angle);
    forwardFor(distance);

    turnTill(0);
    savePosition();


    computeError(startingPos,startingPos );
    computeError(startingPos,1 + startingPos);
    computeError(startingPos,2 + startingPos);
    computeError(startingPos,3 + startingPos);
    computeError(startingPos,4 + startingPos);

    double tmpa = 0;

    std::cout << "\n distances:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
       computeRealDistance(0,i); // << std::endl;        std::cout <<
    }

    std::cout << "\n scale 0:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        tmpa = neuralPos[startingPos].computeSimilarity(neuralPos[i],0);
        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
    }

    for(int i = startingPos; i < 5 + startingPos; i++){
        neuralPos[i].buildNextScale();
    }

    std::cout << "\n scale 1:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        tmpa = neuralPos[startingPos].computeSimilarity(neuralPos[i],1);
        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
    }

    for(int i = startingPos; i < 5 + startingPos; i++){
        neuralPos[i].buildNextScale();
    }
    std::cout << "\n scale 2:" << std::endl;
    for(int i = startingPos; i < 5 + startingPos; i++){
        tmpa = neuralPos[startingPos].computeSimilarity(neuralPos[i],2);
        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
    }

}






void Controller::testingTheAlg(int numberOfPoints){
    int numberOfRecorded = 0;
    int numberOfRecords = 0;
    int desI = 0;
    double desD = 0;

    int waitingActualTime = 500000;     //one half second

    double tmpa;

    //open file
    char nameFile[30] = "/home/franz/thetest# .txt";
    nameFile[20] = numberOfRecords + 48;
    std::ofstream myfile("/home/franz/thetest#3.3(0.95).txt", ios::app);

    //myfile.open("/home/franz/thetest#0.txt", ios::app);

    if (myfile.is_open()) {


    setNewPos(0,0);
    usleep(waitingActualTime);
    positionIndex = 0;
    savePosition();
    neuralPos[0].buildAllScales();

    for(double i = -2; i < 2; i += 0.1){
         for(double j = -2; j < 2; j += 0.1){
                  setNewPos(i,j);
                  usleep(waitingActualTime);
                  savePosition();

                  int placeOfNew = positionIndex-1;

                  neuralPos[placeOfNew].buildAllScales();     //positionIndex-1


                  //theTestPos.buildAllScales();
                  //NeuralPosition static tmpPositio;
                  //tmpPositio.buildDescri(ranges);
                  //tmpPositio.buildAllScales();

                  int bestMatch;
                  for(bestMatch = 0; bestMatch < numberOfScales; bestMatch++){

                      //tmpa = neuralPos[0].computeSimilarity(tmpPositio,bestMatch);
                      tmpa = neuralPos[0].computeSimilarity(neuralPos[placeOfNew],bestMatch);
                      //tmpa = neuralPos[0].computeSimilarity(tmpPositio,bestMatch);
                        std::cout << tmpa * (double)sizeOfDescript << ", " << tmpa << std::endl;
                      if(tmpa >= 0.935){
                          break;
                      }
                  }
                    std::cout << "-->> " << bestMatch <<"\n" << std::endl;

                   myfile << truePositions[placeOfNew][0] << "," << truePositions[placeOfNew][1] << "," << bestMatch << "\n";


                   //positionIndex = 1;
                  //if(positionIndex >= 2){positionIndex = 1;}         //amountOfPositions - 2
         }
    }

        myfile.close();
     } else {std::cout << "Unable to open file" << std::endl;}


    std::cout << "done!!! pff" << std::endl;
/*
       //build all scales:
       for(int i = 0; i < positionIndex; i++){
           // for(int j = 0; j < numberOfScales; j++){
                        neuralPos[i].buildAllScales();
            //}
       }



        //then compare all the position and find the best maching scale
        // after having fund it we save it in a file
        double tmpa;

        //open file
        char nameFile[30] = "/home/franz/thetest# .txt";
        nameFile[20] = numberOfRecords + 48;
        ofstream myfile(nameFile);

        if (myfile.is_open()) {

            for(int i = 0; i < positionIndex; i++){
                int bestMatch;
                for(bestMatch = 0; bestMatch < numberOfScales; bestMatch++){

                    tmpa = neuralPos[0].computeSimilarity(neuralPos[i],bestMatch);

                    if(tmpa >= 0.999){
                        break;
                    } else {
                       // std::cout << tmpa << std::endl;
                       // neuralPos[i].buildNextScale();
                    }
                }


                 myfile << truePositions[i][0] << "," << truePositions[i][1] << "," << bestMatch << "\n";


             }



            myfile.close();
         } else cout << "Unable to open file";
*/

}

/*   while(numberOfRecorded < numberOfPoints){
  //     positionIndex = 0;

       // record points until you reach the limit amount of positions
       while(positionIndex < amountOfPositions - 5){
           desI = std::rand(); desI = desI%2;

           //desI = 0;

           if(desI == 0){
               desI = std::rand(); desI = desI%360;
               desD = std::rand(); desD = fmod(desD,200); desD = desD-200; desD = desD/150;
               //desI = 0;
               //goForwardWithPoints(desI,desD);
           } else {
               oneLoop();
           }

 //      }

   }*/

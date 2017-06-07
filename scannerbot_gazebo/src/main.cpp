/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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





int main(int argc, char** argv)
{
  ros::init(argc, argv, "theControlleNode");
  Controller theMaster;
  ros::NodeHandle n;

  char str[10];
  char a;



  //signal(SIGINT,quit);

 //theMaster.startHokuyo();

  std::cout << "starting the controller!" << std::endl;

  boost::thread my_threadH(boost::bind(&Controller::startSensors, &theMaster));
  
  
  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&Controller::watchdog, &theMaster));


  //ros::Subscriber subHokuyo = theMaster.nSub.subscribe("/my_robot/laser/scan", 1000, theMaster.getHokuyoVal);

  while(true){
      scanf(" %c", &a);


      switch(a) {
      case 'f': {
          theMaster.forwardOne();
          break;
      }
      case 'q':{
         return(0);
          break;
      }
      case 't':{
          //theMaster.turnTill(270); //
          theMaster.neunanteDegRot();
          break;
      }
      case 'p':{
          theMaster.printHokuyoRanges();
          break;
      }
      case 's':{
          theMaster.savePosition();
          break;
      }
      case 'l': {
          theMaster.oneLoop();
          break;
      }
      case 'o': {
          theMaster.printOrientation();
          break;
      }
      case 'a': {
          theMaster.turnTill(0);  //double x =
          break;
      }
      case 'm': {
          theMaster.newNeuronalPos();  //double x =
          break;
      }
      case 'x': {
          theMaster.testingTheAlg(20);
          break;
      }
      case 'k': {
          int x;
          std::cout << "angle?  ";
          scanf(" %i", &x);
          double d;
          std::cout << "distance?  ";
          scanf(" %lf", &d);
          theMaster.goForwardWithPoints(x,d);  //double x =
          break;
      }
      }

  }


  //ros::spin();

  my_threadH.interrupt() ;
  my_threadH.join() ;

  return(0);
}








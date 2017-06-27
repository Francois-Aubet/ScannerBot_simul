#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"

#include "main.h"
#include "neuralPosition.h"

class Controller
{



public:
  Controller();
  void keyLoop();
  void watchdog();

  double linear_, angular_;
  double l_scale_, a_scale_;



  ros::NodeHandle nh_,ph_, nSub, nhmod;
  ros::Time first_publish_;
  ros::Time last_publish_;
  ros::Publisher vel_pub_;
  boost::mutex publish_mutex_;

  ros::ServiceClient client;

  gazebo_msgs::SetModelState setmodelstate;

  geometry_msgs::Pose start_pose;
  geometry_msgs::Twist start_twist;
  gazebo_msgs::ModelState modelstate;



  void publish(double, double);

  void neunanteDegRot(void);

  void forwardOne(void);

  void printHokuyoRanges(void);

  void startSensors(void);
  void savePosition(void);

  double computeError(int posA, int posB);

  void oneLoop(void);

  void goForwardWithPoints(int angle, double dist);

  void findNorthB(void);
  void printOrientation(void);

  double turnTill(double goalAngle);
  void forwardFor(double distance);

  void printPosition(int pos);

  void newNeuronalPos(void);

  double computeRealDistance(int posA, int posB);


  void testingTheAlg(int numberOfPoints);


  //NeuralPosition tmpPositio;

 private:
  void static getHokuyoVal(const sensor_msgs::LaserScan laser);
  void static getOrientation(const sensor_msgs::Imu imu);
  void static getGlobalState(const gazebo_msgs::ModelStates state);



  static const int amountOfPositions = 500;
  static double ranges[sizeOfDescript];

  double static positions[amountOfPositions][sizeOfDescript];

  NeuralPosition static neuralPos[amountOfPositions];

  //NeuralPosition static theTestPos;

  double static truePositions[amountOfPositions][2];

  void setNewPos(double x, double y);


};

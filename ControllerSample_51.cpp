#include <string>    
#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/python.hpp>
#include <Python.h>
#include <dlfcn.h>
#include <typeinfo>
#include <ctime>
#include <unistd.h>
#include <sys/time.h>

#include "StopMotion.h"
#include "ControlPosition.h"
#include "ApplyAction.h"
#include "MeasureDisplacement.h"
#include "EntityMotionStatus.h"
#include "ResetEntity.h"


#define PI 3.141592      
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

using namespace std;
ofstream myfile ("example_51.csv", ios::out | ios::app );

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);  
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt); 

  void applyTapAction();

private:

  const char* linkName;
  CParts *link;

  Vector3d initTargetPos;
  Rotation initTargetRotation;

  Vector3d initToolPos;
  Rotation initToolRotation;
  Rotation newToolRotation;

  Vector3d currentToolPos;
  Vector3d currentTargetPos;

  Vector3d  linkInitPos, linkPos;
  Rotation  linkInitRotation, linkCurrentRotation;


  Vector3d toolVelAtHit;
  Vector3d targetVelAtHit;

  bool positionData;
 
  bool isTargetAtRest;   // to check if target is not moving
  bool isToolAtRest;     // to check if tool is not moving

  bool isNewPoseObtained; // to set tool in a new pose
  bool isToolAtDesiredPosition; 


  bool isXForceVarianceAllowed ;
  bool isZForceVarianceAllowed ;
  double f_x, f_z;
  bool flag;
  bool Colli; 
  int counter;
  
};  
  
Vector3d forceOnTool;
double forceOnTool_x, forceOnTool_z;

void MyController::onInit(InitEvent &evt) {  


  SimObj *target = getObj("box_013");
  SimObj *toolName = getObj("TShapeTool_013");

  toolName->setMass(10.0);
  
  target->getPosition(initTargetPos);
  target->getRotation(initTargetRotation);

  toolName->getPosition(initToolPos);
  toolName->getRotation(initToolRotation);

  isTargetAtRest = true;   
  isToolAtRest = true;  
  f_x = 0 ;
  f_z = 0 ;

  int xForceVariance = 2000;
  int zForceVariance = 2000;


  // Reset the forces applied
  double r;
  r = ((double) rand() / (RAND_MAX)) ;
  f_x = r * int(xForceVariance);
  f_z = r * int(zForceVariance);



  flag = true;
  Colli = false; 
  counter = 0; 

  forceOnTool_x = 5000 + f_z;
  forceOnTool_z = 5000 + f_z;


  // cout << "The applied force " << forceOnTool_x <<  " , " << forceOnTool_z << endl;


  forceOnTool.set(forceOnTool_x, 0 , forceOnTool_z);

  // toolName->addForce(forceOnTool.x(), forceOnTool.y(), forceOnTool.z());

  tapWithTool(toolName, initToolRotation, forceOnTool);

  myfile.flush(); // I have uncommented the file, because I want to overwrite the file. 


//   if (myfile.is_open())
//   {
//      myfile << "Action" << " , " << "FunctionalFeature" << " , " ;
//      myfile << "forceOnTool_X" << " , " << "forceOnTool_Z " <<  " , ";
//      myfile << "toolVelAtHit_X" << " , " << "toolVelAtHit_Z" << " , ";  
//      myfile << "InitialToolPos_X" << " , " << "InitialToolPos_Z" << " , ";
//      myfile << "InitialTargetPos_X" << " , " << "InitialTargetPos_Z" << " , ";
//      myfile <<  "targetFinalPos_X" << " , " << "targetFinalPos_Z"  << " , ";
//      myfile << "targetDisplacement_X" << " , " << "targetDisplacement_Z";
//      myfile <<  "\n" ;

// }


}  



  
double MyController::onAction(ActionEvent &evt) {  


  int actionNumber = 5;
  int functionalFeature = 1;
  myfile << setprecision(2) << std::fixed;

  SimObj *target = getObj("box_013");
  SimObj *toolName = getObj("TShapeTool_013");
  maintainOrientationOfTool(toolName, initToolRotation);
  

  isToolAtRest   = checkEntityMotionStatus(toolName);  // checks whether the tool is moving by calculating its velocity
  isTargetAtRest = checkEntityMotionStatus(target);    // checks whether the object is moving by calculating its velocity

  if(isToolAtRest)
  {
     toolName->getPosition(currentToolPos);
  }

      
 if (isTargetAtRest && isToolAtRest)
 {

    target->getPosition(currentTargetPos);        

 }

if (isTargetAtRest && isToolAtRest && flag )
       {
          
           myfile << actionNumber  << " , " << functionalFeature << " , " ;
           myfile << forceOnTool_x  << " , " << forceOnTool_z << " , " ;
           myfile << initToolPos.x() << " , " << initToolPos.z() << " , " ;
           myfile << initTargetPos.x() << " , " << initTargetPos.z() << " , " ;
           myfile << currentTargetPos.x() << " , " << currentTargetPos.z() << " , " ;
           myfile << currentTargetPos.x() -  initTargetPos.x() << " , " << currentTargetPos.z() - initTargetPos.z();
           myfile << "\n"; 
          
           cout << "The simulation for " << actionNumber << " , " << functionalFeature << " has been recorded" << endl;
           // exit(0);
           flag = false;  

        
  }

  

      
  return 0.01;    

 }

        
std::string msg = " ";

void MyController::onRecvMsg(RecvMsgEvent &evt) {  

  }  

void MyController::onCollision(CollisionEvent &evt) { 

  Colli = true; 
  counter ++ ; 

  SimObj *target = getObj("box_001");
  SimObj *toolName = getObj("TShapeTool_001");


  if(Colli && counter == 1 )
  {
      toolName->getLinearVelocity(toolVelAtHit);
      target->getLinearVelocity(targetVelAtHit);
      cout << "The tool velocity is " << toolVelAtHit.x() << " , " << toolVelAtHit.z() << endl;
      cout << "The target velocity is " << targetVelAtHit.x() << " , " << targetVelAtHit.z() << endl;
      myfile << toolVelAtHit.x() << " , " << toolVelAtHit.z() << " , " ;
      myfile << targetVelAtHit.x() << " , " << targetVelAtHit.z() << " , " ;
      Colli = false;

  }


}



  
extern "C" Controller * createController() {  
  return new MyController;  
}  


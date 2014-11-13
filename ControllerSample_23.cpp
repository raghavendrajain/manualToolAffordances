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
ofstream myfile ("example_23.csv", ios::out | ios::app );

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

  bool positionData;
 
  bool isTargetAtRest;   // to check if target is not moving
  bool isToolAtRest;     // to check if tool is not moving

  bool isNewPoseObtained; // to set tool in a new pose
  bool isToolAtDesiredPosition; 


  bool isXForceVarianceAllowed ;
  bool isZForceVarianceAllowed ;
  double f_x, f_z;
  bool flag;

  
};  
  
Vector3d forceOnTool;
double forceOnTool_x, forceOnTool_z;

void MyController::onInit(InitEvent &evt) {  


  SimObj *target = getObj("box_006");
  SimObj *toolName = getObj("TShapeTool_006");

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

  forceOnTool_x = -10000 - f_x;
  // forceOnTool_z = 10000 + f_z;
  // forceOnTool_x = 0;
  forceOnTool_z = 0;

  // cout << "The applied force " << forceOnTool_x <<  " , " << forceOnTool_z << endl;


  forceOnTool.set(forceOnTool_x, 0 , forceOnTool_z);
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


  int actionNumber = 2;
  int functionalFeature = 3;
  myfile << setprecision(2) << std::fixed;

  SimObj *target = getObj("box_006");
  SimObj *toolName = getObj("TShapeTool_006");
  
  
  
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

if (isTargetAtRest && isToolAtRest)
       {
          
           myfile << actionNumber  << " , " << functionalFeature << " , " ;
           myfile << forceOnTool_x  << " , " << forceOnTool_z << " , " ;
           myfile << toolVelAtHit.x() << " , " << toolVelAtHit.z() << " , " ;
           myfile << initToolPos.x() << " , " << initToolPos.z() << " , " ;
           myfile << initTargetPos.x() << " , " << initTargetPos.z() << " , " ;
           myfile << currentTargetPos.x() << " , " << currentTargetPos.z() << " , " ;
           myfile << currentTargetPos.x() -  initTargetPos.x() << " , " << currentTargetPos.z() - initTargetPos.z();
           myfile << "\n"; 
           

           cout << "The simulation for " << actionNumber << " , " << functionalFeature << " has been recorded" << endl;
           exit(0);

        
  }

  

      
  return 0.01;    

 }

        
std::string msg = " ";

void MyController::onRecvMsg(RecvMsgEvent &evt) {  

  }  

void MyController::onCollision(CollisionEvent &evt) { 


}



  
extern "C" Controller * createController() {  
  return new MyController;  
}  


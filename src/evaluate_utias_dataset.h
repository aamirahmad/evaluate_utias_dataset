#include <cstdio>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <read_omni_dataset/LRMGTData.h>
#include <read_omni_dataset/RobotState.h>
// for receiving the odometry of the robots 
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>

// for robot to landmark and robot to robot (and target=robot) measurements
#include <utiasdata_to_rosbags/MCLAMMeasurementData.h>

// for once receiving the info of the landmarks GT
#include <utiasdata_to_rosbags/MCLAM_landmark_GTData.h>

// for publishing the estimatated and ground truth states of the robots
#include <utiasdata_to_rosbags/MCLAM_RobotTeamState.h>

using namespace cv;
using namespace std;
using namespace ros;

// Dataset related constants

///@INFO In truth we have only 4 robots in the dataset but since the robot IDs go upto 5, we will keep a phantom robot: OMNI2
const std::size_t NUM_ROBOTS = 5;  
///@INFO In truth we have 2 targets in the raw dataset but the 'pre-processed' data does not contain the measurements of the blue ball from the robots (we have that only for the orange ball). However, extend this to 2 in case you do your own pre-processing for the blue ball measurements using the raw images from the robot's camera. Check raw dataset details for other related params to perform this measurement
const std::size_t NUM_TARGETS = 1;

/******** Intrinsic Parameters ********/
/******** !!!!DO NOT MODIFY!!! ********/

///@INFO The params below are of the individual cameras of the GT camera system.
///@INFO Extrinsic stere calibration is performed on the fly (to avoid carrying a lot of xml files and make this code more portable) and called at the beginning of this node. Check method initializeCamParams()

//Left camera
double M1[] = {952.364130338909490,0,638.552089454614702, 0,953.190083002521192,478.055570057081638, 0,0,1};
double D1[] = {-0.239034777360406 , 0.099854524375872 , -0.000493227284393 , 0.000055426659564 , 0.000000000000000 };  

//Right Camera
double M2[] = {949.936867662258351,0,635.037661574667936,0,951.155555785186380,482.586148439616579 ,0,0,1};
double D2[] = { -0.243312251484708 , 0.107447328223015 , -0.000324393194744 , -0.000372030056928 , 0.000000000000000 };

// Some constants for drawing funcions... change these values if you want more points on the overlaid circle... an so on
static const int circlePointsPerPosition = 50;
static const int targetPntsPos = 1;
static const int arrowPointsPerPosition = 20;
static const int totPntsPerPos = 1 + circlePointsPerPosition + arrowPointsPerPosition;
static const int totPntsPerPosGT = 1 + circlePointsPerPosition;

// Some fixed omni robot-specific constants 
static const float robRadius = 0.20; //in meters
static const float robHeight = 0.0; //in meters  
  
  
class ImageOverlayer
{
  //ROS node specific private stuff
  NodeHandle n; 
  Subscriber sRobotsGT_and_Estimates_,sTargetRobotGT_;
  Publisher orangeBallError;
  //Main image
  IplImage img;   
  char imagePathNameBaseName[100];
  
  //Estimated states
  geometry_msgs::PoseWithCovarianceStamped robot_state[NUM_ROBOTS];
  geometry_msgs::PoseWithCovarianceStamped target_state[NUM_TARGETS];
  bool robotActive[NUM_ROBOTS];
  bool targetActive[NUM_TARGETS];
  
  // GT Stereo system calibration params
  CvMat _M1;
  CvMat _D1;  
  CvMat _M2;
  CvMat _D2;
  CvMat* Tvec_right;
  CvMat* Rvec_right_n;
  CvMat* Tvec_left;
  CvMat* Rvec_left_n;  
  
  //Groud Truth (GT) poses and positions using custom ROS msgs
  CvScalar color_est[10];
  geometry_msgs::PoseWithCovariance omniGTPose[5];
  bool foundOMNI_GT[5];
  read_omni_dataset::BallData targetGTPosition[1];
  float targetGTdata[2];//x,y,No theta because we are not detecting the target's orientation
  
  
  public:
    ImageOverlayer(char *camImageFolderPath)
    {
     
     strcpy(imagePathNameBaseName,camImageFolderPath); 
      
     cvNamedWindow("Overlaid Estimates on the Grount Truth Images of Right GT Camera"); 
     
      sRobotsGT_and_Estimates_ = n.subscribe<utiasdata_to_rosbags::MCLAM_RobotTeamState>("/utias_mhlspose_estimated", 1000, boost::bind(&ImageOverlayer::utiasMHLSCallback,this, _1,&img));
           
      sTargetRobotGT_ = n.subscribe<geometry_msgs::PoseStamped>("/Robot_5/GTpose", 1000, boost::bind(&ImageOverlayer::UpdateTargetGT,this, _1));   
     
     // Stereo cam calibration. Done only once in the beginning
     initializeCamParams();
     
     //change these to alter colors of the robots estimated state representation   
     color_est[0] = cvScalar(255.0, 0.0, 0.0);
     color_est[1] = cvScalar(0.0, 0.0, 255.0);
     color_est[2] = cvScalar(0.0, 255.0, 0.0);
     color_est[3] = cvScalar(255.0, 255.0, 0.0);
     color_est[4] = cvScalar(255.0, 0.0, 255.0);
     
     color_est[5] = cvScalar(128.0, 0.0, 0.0);   
     color_est[6] = cvScalar(0.0, 0.0, 128.0);  
     color_est[7] = cvScalar(0.0, 128.0, 0.0);    
     color_est[8] = cvScalar(128.0, 128.0, 0.0);  
     color_est[9] = cvScalar(0.0, 0.0, 0.0);     
     
     //some bool initialization
     foundOMNI_GT[0] = false;foundOMNI_GT[1] = false;foundOMNI_GT[2] = false;foundOMNI_GT[3] = false;foundOMNI_GT[4] = false;     
     for(int i = 0; i<NUM_ROBOTS; i++)
	robotActive[i] = false;
    }
    
    void UpdateTargetGT(const geometry_msgs::PoseStamped::ConstPtr&);
    
    // main callback for the utias MHLS estimates overlay
    void utiasMHLSCallback(const utiasdata_to_rosbags::MCLAM_RobotTeamState::ConstPtr&, IplImage*);
    
    ///Important initializer: computes the reprojection matrices based on the stereo calibration. Do not change this method for the LRM GT images!!! Port it to other stereosystems if and when necessary
    void initializeCamParams(void);
    
    ///Overlay the colored circle, and arrow for the updated robot pose
    void OverlayEstimatedRobotPose(double, double, double, double, CvScalar, IplImage*);
    
    ///Overlay the colored circle, and arrow for the updated robot pose
    void OverlayTargetGTPose(double, double, double, double, CvScalar, IplImage*);    
    
    
    void MakeGroundPlaneX(CvScalar, IplImage* baseImage);
    void MakeGroundPlaneY(CvScalar, IplImage* baseImage);
    void MakeGroundPlaneZ(CvScalar, IplImage* baseImage);
};
#include "evaluate_utias_dataset.h"

inline void getActualShift(int i, double* shift_a_, double* shift_b_)
{
     
    if(i==0)
    {
      *shift_a_ = 0.098947; *shift_b_ = 0.02391;
    }
    if(i==2)
    {
      *shift_a_ = 0.153636; *shift_b_ = 0.025022;
    }
    if(i==3)
    {
      *shift_a_ = 0.061141; *shift_b_ = 0.0272;
    }
    if(i==4)
    {
      *shift_a_ = 0.102613; *shift_b_ = 0.01631;
    }    
   
}


void ImageOverlayer::initializeCamParams()
{
  int nframes=1;
  int n = 9; //Number of points used to process
  int N=nframes*n;
  
  vector<CvPoint2D32f> temp(n);
  vector<int> npoints;
  vector<CvPoint3D32f> objectPoints;
  vector<CvPoint2D32f> points[2];
  points[0].resize(n);
  points[1].resize(n);
  
  double R[3][3], T[3], E[3][3], F[3][3];
  double Q[4][4];
    

/*************************************************************/    
    
   _M1 = cvMat(3, 3, CV_64F, M1 );
   _M2 = cvMat(3, 3, CV_64F, M2 );
   _D1 = cvMat(1, 5, CV_64F, D1 );
   _D2 = cvMat(1, 5, CV_64F, D2 );
  CvMat _R = cvMat(3, 3, CV_64F, R );
  CvMat _T = cvMat(3, 1, CV_64F, T );
  CvMat _E = cvMat(3, 3, CV_64F, E );
  CvMat _F = cvMat(3, 3, CV_64F, F );
  CvMat _Q = cvMat(4,4, CV_64F, Q);
  
  vector<CvPoint2D32f>& pts = points[0];

  
  //Pontos XY pixels left
  points[0][0].x=34.0;
  points[0][0].y=336.0;
  points[0][1].x=502.0;
  points[0][1].y=156.0;
  points[0][2].x=1280.0;
  points[0][2].y=279.0;
  points[0][3].x=664.0;
  points[0][3].y=174.0;
  points[0][4].x=914.0;
  points[0][4].y=209.0;
  points[0][5].x=248.0;
  points[0][5].y=300.0;
  points[0][6].x=663.0;
  points[0][6].y=482.0;
  points[0][7].x=185.0;
  points[0][7].y=364.0;
  points[0][8].x=400.0;
  points[0][8].y=507.0;  

  
  //Pontos XY pixels right
  points[1][0].x=866.0;
  points[1][0].y=942.0;
  points[1][1].x=98.0;
  points[1][1].y=376.0;
  points[1][2].x=856.0;
  points[1][2].y=72.0;
  points[1][3].x=445.0;
  points[1][3].y=222.0;
  points[1][4].x=690.0;
  points[1][4].y=128.0;
  points[1][5].x=779.0;
  points[1][5].y=442.0;
  points[1][6].x=1162.0;
  points[1][6].y=161.0;
  points[1][7].x=1061.0;
  points[1][7].y=413.0;
  points[1][8].x=1244.0;
  points[1][8].y=215.0;
  
  
  npoints.resize(nframes,n);
  objectPoints.resize(nframes*n);
  
  /* 3D points (x,y,z) related to the 2D points from left and right image - minimum: 8 points*/
  
  objectPoints[0] = cvPoint3D32f(6.0,-4.5,0.0);
  objectPoints[1] = cvPoint3D32f(0.0,-4.5,0.0);
  objectPoints[2] = cvPoint3D32f(0.0,+4.5,0.0);
  objectPoints[3] = cvPoint3D32f(0.0,-1.5,0.0);
  objectPoints[4] = cvPoint3D32f(0.0,+1.5,0.0);
  objectPoints[5] = cvPoint3D32f(4.3,-2.7,0.0);
  objectPoints[6] = cvPoint3D32f(4.3,+2.7,0.0);
  objectPoints[7] = cvPoint3D32f(5.25,-1.75,0.0);
  objectPoints[8] = cvPoint3D32f(5.25,+1.75,0.0);  
  
  for( int i = 1; i < nframes; i++ )
      copy( objectPoints.begin(), objectPoints.begin() + n,
      objectPoints.begin() + i*n );
  
  CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
  CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
  CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
  CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
  
   
   /**************************************************************************/
  double R1[3][3], R2[3][3], P1[3][4], P2[3][4];

  CvMat _R1 = cvMat(3, 3, CV_64F, R1);
  CvMat _R2 = cvMat(3, 3, CV_64F, R2);
  CvMat _P1 = cvMat(3, 4, CV_64F, P1);
  CvMat _P2 = cvMat(3, 4, CV_64F, P2);

/************************************** StereoCalibration - returns R T R1 R2 T1 T2 **************************************/   
  CvSize imageSize = cvSize(1294,964);

  cvStereoCalibrate( &_objectPoints, &_imagePoints1,
      &_imagePoints2, &_npoints,
      &_M1, &_D1, &_M2, &_D2,
      imageSize, &_R, &_T, &_E, &_F,
      cvTermCriteria(CV_TERMCRIT_ITER+
      CV_TERMCRIT_EPS, 30, 1e-5),
      CV_CALIB_USE_INTRINSIC_GUESS+ CV_CALIB_FIX_ASPECT_RATIO);

     
/*************************************************************************************************************************/ 

/***************************************** Extrinsic Parameters **********************************************************/
  CvMat* Rvec_left = cvCreateMat( 3, 1, CV_64F );
  Tvec_left = cvCreateMat( 3, 1, CV_64F );
  CvMat* Rvec_right = cvCreateMat( 3, 1, CV_64F );
  Tvec_right = cvCreateMat( 3, 1, CV_64F ); 
  
  cvFindExtrinsicCameraParams2(&_objectPoints, &_imagePoints1,&_M1,&_D1,Rvec_left,Tvec_left);
  
  Rvec_left_n = cvCreateMat( 3, 3, CV_64F );
  cvRodrigues2(Rvec_left,Rvec_left_n,0);
    

  cvFindExtrinsicCameraParams2(&_objectPoints, &_imagePoints2,&_M2,&_D2,Rvec_right,Tvec_right);
  
  Rvec_right_n = cvCreateMat( 3, 3, CV_64F );
  cvRodrigues2(Rvec_right,Rvec_right_n,0);
    
/*************************************************************************************************************************/
}

void ImageOverlayer::MakeGroundPlaneX(CvScalar color, IplImage* baseImage)
{
  
   int xlinePoints = 500; 
  
   vector<CvPoint3D32f> Robot_PositionsToReProject;
   Robot_PositionsToReProject.resize(xlinePoints);
   
   CvMat _Robot_PositionsToReProject = cvMat(1, xlinePoints, CV_32FC3, &Robot_PositionsToReProject[0]);    
   
    

    for(int pts = 0; pts < xlinePoints; pts++) //circlePointsPerPosition points out of xlinePoints for circle
      {
	float xValue = -7 + ((float)(14.0/xlinePoints))*(float)pts;
	//std::cout<<"xvalue = "<<xValue<<std::endl;
	Robot_PositionsToReProject[pts] = cvPoint3D32f(xValue, 0, 0);
      }
     
   
    vector<CvPoint2D32f> reprojectedPoints_Robot;
    reprojectedPoints_Robot.resize(xlinePoints);
    
    CvMat _imageReprojectedPoints_RobotRight = cvMat(1, xlinePoints, CV_32FC2, &reprojectedPoints_Robot[0]);
    
    cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2, &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL, NULL, NULL); 
   
    for(int pts = 0; pts < xlinePoints; pts++)
      {    
	CvPoint robot_PointToBeShownRight = cvPoint(reprojectedPoints_Robot[pts].x,reprojectedPoints_Robot[pts].y);
	cvCircle(baseImage, robot_PointToBeShownRight, 0, color, 2, 8, 0);       	
      }
   
}

void ImageOverlayer::MakeGroundPlaneY(CvScalar color, IplImage* baseImage)
{
  
   int linePoints = 500; 
  
   vector<CvPoint3D32f> Robot_PositionsToReProject;
   Robot_PositionsToReProject.resize(linePoints);
   
   CvMat _Robot_PositionsToReProject = cvMat(1, linePoints, CV_32FC3, &Robot_PositionsToReProject[0]);    
   
    

    for(int pts = 0; pts < linePoints; pts++) //circlePointsPerPosition points out of linePoints for circle
      {
	float yValue = -7 + ((float)(14.0/linePoints))*(float)pts;
	//std::cout<<"xvalue = "<<xValue<<std::endl;
	Robot_PositionsToReProject[pts] = cvPoint3D32f(0, yValue, 0);
      }
     
   
    vector<CvPoint2D32f> reprojectedPoints_Robot;
    reprojectedPoints_Robot.resize(linePoints);
    
    CvMat _imageReprojectedPoints_RobotRight = cvMat(1, linePoints, CV_32FC2, &reprojectedPoints_Robot[0]);
    
    cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2, &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL, NULL, NULL); 
   
    for(int pts = 0; pts < linePoints; pts++)
      {    
	CvPoint robot_PointToBeShownRight = cvPoint(reprojectedPoints_Robot[pts].x,reprojectedPoints_Robot[pts].y);
	cvCircle(baseImage, robot_PointToBeShownRight, 0, color, 2, 8, 0);       	
      }
   
}

void ImageOverlayer::MakeGroundPlaneZ(CvScalar color, IplImage* baseImage)
{
  
   int linePoints = 500; 
  
   vector<CvPoint3D32f> Robot_PositionsToReProject;
   Robot_PositionsToReProject.resize(linePoints);
   
   CvMat _Robot_PositionsToReProject = cvMat(1, linePoints, CV_32FC3, &Robot_PositionsToReProject[0]);    
   
    

    for(int pts = 0; pts < linePoints; pts++) //circlePointsPerPosition points out of linePoints for circle
      {
	float zValue = 0 + ((float)(5.0/linePoints))*(float)pts;
	//std::cout<<"xvalue = "<<xValue<<std::endl;
	Robot_PositionsToReProject[pts] = cvPoint3D32f(0, 0, zValue);
      }
     
   
    vector<CvPoint2D32f> reprojectedPoints_Robot;
    reprojectedPoints_Robot.resize(linePoints);
    
    CvMat _imageReprojectedPoints_RobotRight = cvMat(1, linePoints, CV_32FC2, &reprojectedPoints_Robot[0]);
    
    cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2, &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL, NULL, NULL); 
   
    for(int pts = 0; pts < linePoints; pts++)
      {    
	CvPoint robot_PointToBeShownRight = cvPoint(reprojectedPoints_Robot[pts].x,reprojectedPoints_Robot[pts].y);
	cvCircle(baseImage, robot_PointToBeShownRight, 0, color, 2, 8, 0);       	
      }
   
}


void ImageOverlayer::UpdateTargetGT(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  targetGTdata[0] = msg->pose.position.x;
  targetGTdata[1] = msg->pose.position.y;
}


void ImageOverlayer::OverlayTargetGTPose(double x, double y, double z, double thetaRob, CvScalar color, IplImage* baseImage)
{
   
   vector<CvPoint3D32f> Robot_PositionsToReProject;
   Robot_PositionsToReProject.resize(totPntsPerPos);
   
   CvMat _Robot_PositionsToReProject = cvMat(1, totPntsPerPos, CV_32FC3, &Robot_PositionsToReProject[0]);    
  
   
  for(float j=0; j<=0.03; j+=0.01)
   {
    
    Robot_PositionsToReProject[0] = cvPoint3D32f(x,y,j);
    for(int pts = 0; pts < circlePointsPerPosition; pts++) //circlePointsPerPosition points out of totPntsPerPos for circle
      {
	float theta = -M_PI + (float)pts*2*M_PI/(circlePointsPerPosition);  
	Robot_PositionsToReProject[1 + pts] = cvPoint3D32f( x + robRadius*cosf(theta), y + robRadius*sinf(theta), j);
      }
     
   for(int pts = 0; pts < arrowPointsPerPosition /*&& j==robHeight*/; pts++) //arrowPointsPerPosition points out of totPntsPerPos for th arrow
    {
      Robot_PositionsToReProject[1 + circlePointsPerPosition + pts] = cvPoint3D32f( x + (float)pts*(robRadius/(float)arrowPointsPerPosition)*cosf(thetaRob), y + (float)pts*(robRadius/(float)arrowPointsPerPosition)*sinf(thetaRob), 0.03);
    }
   
    vector<CvPoint2D32f> reprojectedPoints_Robot;
    reprojectedPoints_Robot.resize(totPntsPerPos);
    
    CvMat _imageReprojectedPoints_RobotRight = cvMat(1, totPntsPerPos, CV_32FC2, &reprojectedPoints_Robot[0]);
    
    cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2, &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL, NULL, NULL); 
   
    for(int pts = 0; pts < totPntsPerPos; pts++)
      {    
	CvPoint robot_PointToBeShownRight = cvPoint(reprojectedPoints_Robot[pts].x,reprojectedPoints_Robot[pts].y);
	cvCircle(baseImage, robot_PointToBeShownRight, 0, color, 2, 8, 0);       	
      }
   }
}



void ImageOverlayer::OverlayEstimatedRobotPose(double x, double y, double z, double thetaRob, CvScalar color, IplImage* baseImage)
{
   
   vector<CvPoint3D32f> Robot_PositionsToReProject;
   Robot_PositionsToReProject.resize(totPntsPerPos);
   
   CvMat _Robot_PositionsToReProject = cvMat(1, totPntsPerPos, CV_32FC3, &Robot_PositionsToReProject[0]);    
  
   
  for(float j=0; j<=robHeight; j+=0.01)
   {
    
    Robot_PositionsToReProject[0] = cvPoint3D32f(x,y,j);
    for(int pts = 0; pts < circlePointsPerPosition; pts++) //circlePointsPerPosition points out of totPntsPerPos for circle
      {
	float theta = -M_PI + (float)pts*2*M_PI/(circlePointsPerPosition);  
	Robot_PositionsToReProject[1 + pts] = cvPoint3D32f( x + robRadius*cosf(theta), y + robRadius*sinf(theta), j);
      }
     
   for(int pts = 0; pts < arrowPointsPerPosition /*&& j==robHeight*/; pts++) //arrowPointsPerPosition points out of totPntsPerPos for th arrow
    {
      Robot_PositionsToReProject[1 + circlePointsPerPosition + pts] = cvPoint3D32f( x + (float)pts*(robRadius/(float)arrowPointsPerPosition)*cosf(thetaRob), y + (float)pts*(robRadius/(float)arrowPointsPerPosition)*sinf(thetaRob), robHeight);
    }
   
    vector<CvPoint2D32f> reprojectedPoints_Robot;
    reprojectedPoints_Robot.resize(totPntsPerPos);
    
    CvMat _imageReprojectedPoints_RobotRight = cvMat(1, totPntsPerPos, CV_32FC2, &reprojectedPoints_Robot[0]);
    
    cvProjectPoints2(&_Robot_PositionsToReProject, Rvec_right_n, Tvec_right, &_M2, &_D2, &_imageReprojectedPoints_RobotRight, NULL, NULL, NULL, NULL, NULL); 
   
    for(int pts = 0; pts < totPntsPerPos; pts++)
      {    
	CvPoint robot_PointToBeShownRight = cvPoint(reprojectedPoints_Robot[pts].x,reprojectedPoints_Robot[pts].y);
	cvCircle(baseImage, robot_PointToBeShownRight, 0, color, 2, 8, 0);       	
      }
   }
}



void ImageOverlayer::utiasMHLSCallback(const utiasdata_to_rosbags::MCLAM_RobotTeamState::ConstPtr& msg, IplImage* img_)
{
   
  char backgroundImageName[500];
  strcpy(backgroundImageName,(ros::package::getPath("evaluate_utias_dataset")).c_str());
  strcat(backgroundImageName,"/resources/background_real.jpg");
  
  img_=cvLoadImage(backgroundImageName);
 
  //MakeGroundPlaneX(color_est[1],img_);
  //MakeGroundPlaneY(color_est[1],img_);
  //MakeGroundPlaneZ(color_est[1],img_);
  
  for(int robot = 0; robot<5;robot++)
  {
    //if(robot!=4)
    OverlayEstimatedRobotPose(msg->robotGTpose[robot].pose.position.x,msg->robotGTpose[robot].pose.position.y,0.5/*Just any height for now*/,tf::getYaw(msg->robotGTpose[robot].pose.orientation),color_est[9],img_);   // Darker shades are GT


    OverlayEstimatedRobotPose(msg->robotEstimatedpose[robot].pose.position.x,msg->robotEstimatedpose[robot].pose.position.y,0.5/*Just any height for now*/,tf::getYaw(msg->robotEstimatedpose[robot].pose.orientation),color_est[robot],img_);   // lighter color shades are estimated robot position
  }

  //OverlayTargetGTPose(targetGTdata[0],targetGTdata[1],0.5/*Just any height for now*/,0,color_est[8],img_);
  
  cvShowImage("Overlaid Estimates on the Grount Truth Images of Right GT Camera", img_);
  cvSaveImage("/home/aamir/catkin_ws/src/abc.jpg",img_);
  cvReleaseImage(&img_);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_overlay_utias_dataset"); 
  
    if (argc != 2)
    {
      ROS_WARN("WARNING: you should specify images folder path (on which GT and estimated poses/positions will be overlaid)! which might be %s\n, respectively",argv[1]);
    }
    else
    {
      printf("INFO: you have set camera images folder path: %s\n",argv[1]);
    }  

  cvStartWindowThread();

  ImageOverlayer node(argv[1]);
  ros::spin();
  return 0;
}
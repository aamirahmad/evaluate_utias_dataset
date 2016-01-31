#include <cstdio>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <time.h>


using namespace std;

int main(int argc, char **argv)
{
    
  //In this program we generate the plots for the mhls method on utias dataet.
  

  cout<<endl<<endl<<endl<<"---------------------Please note that this program should be executed from the directory level where the results file exist otherwise we cannot evaluate---------------------"<<endl<<endl<<endl;

  
  char listFiles_filename[500];
  strcpy(listFiles_filename,"result_filenames.txt");
  
  FILE* tabulatedResultsFile = fopen("tabulatedResults.dat","w");
  fprintf(tabulatedResultsFile,"# roboID\tWindowSize\tDecayCoeff\trmsROB_1\trmsROB_2\trmsROB_3\trmsROB_4\trmsROB_5\n");
  
  char resultsFilename[500];
  
  FILE* nameFiles,*resultsFile;
  nameFiles = fopen(listFiles_filename,"r");
  
  while(!feof(nameFiles))
  {
    fscanf(nameFiles,"%s\n",resultsFilename);
    
    if(resultsFilename!=NULL)
    {
      resultsFile = fopen(resultsFilename,"r");
      int robID=0, Win_Size=0, decayCoeff=0;
      sscanf(resultsFilename,"%*6c_rob_%d_%d_%d.results",&robID,&Win_Size,&decayCoeff);
      cout<<" robID = "<<robID<<" Win_Size = "<<Win_Size<<" decayCoeff = "<<decayCoeff<<endl; 
      
      int buf[5];
      double x_true[5][40000], y_true[5][40000], theta_true;
      double x_est[5][40000], y_est[5][40000], theta_est;
      double error[5][40000];
      unsigned long long int time;
      
      double rms[5]={0,0,0,0,0};
      double rmsDEV[5]={0,0,0,0,0};
      double rmsVAR[5]={0,0,0,0,0};
      
      int counter = 0;
      
      cout<<" file is "<<resultsFilename<<endl;
      
      while(!feof(resultsFile))
      {
	for(int i=0; i<5; i++)
	{
	  fscanf(resultsFile,"%d %lf %lf %lf %lf %lf %lf %llu",&buf[i],&x_est[i][counter],&y_est[i][counter],&theta_est,&x_true[i][counter],&y_true[i][counter],&theta_true,&time);
	  
	  error[i][counter] = pow((pow((x_est[i][counter]-x_true[i][counter]),2)+pow((y_est[i][counter]-y_true[i][counter]),2)),0.5);
	  
	  rms[i] = rms[i]+error[i][counter];
	}
	fscanf(resultsFile,"\n");
	counter++; 
      }
      
      	for(int i=0; i<5; i++)
	{
	  for(int j=0; j<counter; j++)
	  {
	    rmsVAR[i] = rmsVAR[i]+ pow((error[i][counter]-(rms[i]/counter)),2);
	  }
	  rmsVAR[i] = rmsVAR[i]/counter;
	  rmsDEV[i] = pow(rmsVAR[i],0.5);
	}
	
      if((Win_Size==10 && decayCoeff%3==0) || (Win_Size==15 && decayCoeff%2==0))	
	fprintf(tabulatedResultsFile,"%d\t%d\t%d\t%lf %lf\t%lf %lf\t%lf %lf\t%lf %lf\t%lf %lf\n", robID,Win_Size,decayCoeff,rms[0]/counter,rmsVAR[0],rms[1]/counter,rmsVAR[1],rms[2]/counter,rmsVAR[2],rms[3]/counter,rmsVAR[3],rms[4]/counter,rmsVAR[4]);   
      //cout<<"rms of robot 1 = "<<rms[0]/counter<<endl;
      //cout<<"rms of robot 2 = "<<rms[1]/counter<<endl;
      //cout<<"rms of robot 3 = "<<rms[2]/counter<<endl;
      //cout<<"rms of robot 4 = "<<rms[3]/counter<<endl;
      //cout<<"rms of robot 5 = "<<rms[4]/counter<<endl;      
    }

  }
  fcloseall();

  
  
  return 0;
  
}
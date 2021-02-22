/**************************************
 ** Christos N. Mavridis @ CSL, NTUA **
 **************************************

Warning: I have changed stride and ringBufferSize.
I must fix the comments and the readme file sometime !!!

PREREQUISITES:
	bsif.h
	bsif_libusb.o
	labview_dll.ini
	labview_dll.h
	labview_dll shared library
	LDFLAGS=-L. -llabview_dll

 HOW TO RUN:
	  1. sudo su 
	  (2. source initbiosemi.sh)
	  3. roscd biosemi
	  4. rosrun biosemi biosemiNode [arguments] 
	
 ARGUMENTS:
    1. file to write
    2. time duration
    
 PARAMETERS
	  0. paramfile.txt in /catkin_ws/src/biosemi 
          includes parameters for processing and decoding procedures
    1. MACROS 
          READ_FROM_FILE 
          FILE_TO_READ
          WRITE_IN_OR_OUT 
          PA10_CONTROL etc.    		

 PROBLEMS:
    1. iir filter in the begining draws rmean down and produces bias in the features
    2. neural nets decoding 

***************************************  */

#include "ros/ros.h"
#include "biosemi/biosemiMessage.h"
#include <geometry_msgs/Vector3Stamped.h>

#include <stdio.h> 
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>	
#include <errno.h>
#include <signal.h>
#include <sys/wait.h>
#include <string.h>
#include <math.h>

#include <iostream>

#include "biosemi/labview_dll.h"
#include "biosemi/bsif.h"

#define BIO_FREQ 16384
#define NEW_FREQ 500
#define SPEED_MODE 7
#define NUM_CHANNELS 58
#define BIO_CHANS 32 // 40 with 8 EX channels
#define PRINT_STATUS_FREQ 64
#define BIO_SAMPLES 128 // Number of samples per read (stride)

#define BUFFERS_NUMBER 4 // buffers used for filtering
#define BUFFER_ORDER 10
#define SS_SIZE 10
#define NN_SIZE 10

#define READ_FROM_FILE 0
#define FILE_TO_READ "/home/neurorobotics/cmCode/experiments/bb240217/txt/eegnew1"
#define WRITE_IN_OR_OUT 0
#define SS_OR_NN 1 
#define PA10_CONTROL 0

using namespace std;

FILE * eegfile;
FILE * paramfile;

class Eeg_Processing
{

private:
  
  int chIndex[BIO_CHANS];
  int spatialFilterType;
  double pcaMatrixIn[BIO_CHANS][BIO_CHANS];
  double mapminmaxIn[BIO_CHANS][2];
  
  struct meanStruct {
    unsigned long int lastindex; // start from 0
    double lastmean[BIO_CHANS];
  } rmean;
  
  int ssorder;  
  double ssX[SS_SIZE];
  double ssA[SS_SIZE][SS_SIZE];
  double ssB[SS_SIZE][SS_SIZE];
  double ssC[SS_SIZE][SS_SIZE];
  int nnLayers;
  int neuronsPerLayer[NN_SIZE];  
  double w1[NN_SIZE][BIO_CHANS];
  double w2[NN_SIZE][NN_SIZE];
  double w3[NN_SIZE][NN_SIZE];
  double b1[NN_SIZE];  
  double b2[NN_SIZE];
  double b3[NN_SIZE];
  double mapminmaxOut[BIO_CHANS][2];
  double pcaMatrixOutT[BIO_CHANS][BIO_CHANS];
  double meanOut[BIO_CHANS];  
   
public:

  unsigned long int samplesCounter;
  double samplesTime;
	double buffers[BUFFERS_NUMBER][BIO_CHANS][BUFFER_ORDER+1];
	
	int inchans;
	int inchansLowD; 
	double inSample[BIO_CHANS];  
	
	int outchans; 
	int outchansHighD; 
	double outSample[BIO_CHANS];  
      
  Eeg_Processing();
	~Eeg_Processing();
	
	int read_parameters();
	void centerdata();
  void iir_filter(int bufferIn, int bufferOut, 
              int iirOrder, double a_coeff[], double b_coeff[]);
  void push_buffers(int buffer);
  bool resampleTime();
  void spatialFilter();
  void channel_selection();
  void pca();
  void mapminmax();
  
  void ssDecode();
  void nnDecode();
  void mapminmaxInv();
  void pcaInv();
  
};

class Biosemi_Acq
{
private:
	PUCHAR	ringBuffer;
	CHAR 	controlBuffer[64];
	char	infoBuffer[120];
	INT_PTR	ringBufferSize; // 32Mbytes 
	INT_PTR bytes2Use;
	HANDLE	handle;
	float bytesPerMsec;
	int throughPut;
	int nbuffers; // buffer of 128Kbytes (= stride)
  int	nextSync;
	int lastSync;
	INT_PTR	seam;
  INT_PTR	orion;
	INT_PTR	lastSeam;
	
public:
  long int datum[4520][BIO_CHANS]; // 565 samples in 128Kbyte
  // = round [stride / (NUM_CHANNELS * 4 bytes_per_channel)]
  
  Biosemi_Acq();
	~Biosemi_Acq();	
  int Biosemi_Acq_Init(void);
  int Biosemi_Acq_Start(void);
  int Biosemi_Acq_Get(void); 
  int Biosemi_Acq_Close(void); 
  int Biosemi_Acq_Status(void); 
};

/******************************  MAIN  ****************************************/

int 
main (int argc, char **argv) 
{
  /*** VARIABLES ***/
  
  // ros
  ros::init(argc, argv, "biosemiNode"); // Init ROS publisher
  ros::NodeHandle nHandle;
  ros::Publisher eegPublisher = 
              nHandle.advertise<biosemi::biosemiMessage>("eeg", 1000);
  ros::Rate loopRate(BIO_FREQ/BIO_SAMPLES); // loop at 'read' frequency
  biosemi::biosemiMessage msg;
  
  //if(PA10_CONTROL == 1)
  geometry_msgs::Vector3Stamped pa10Cmd;
  ros::Publisher pa10Publisher = 
      nHandle.advertise<geometry_msgs::Vector3Stamped>("/pa10/eeg_pos", 1000);  
  
  // biosemi
	Biosemi_Acq biosemi;
	Eeg_Processing eegsig;
	
	int bioSamplesRead;
	bool writeFile = 0;
	ros::Time startTime, readTime;
	int	time_limit = 0; 
	int chansIn = 0;
	int chansOut = 0;
		
	// filters
	double butter1_44_1000a[2] = {1, -0.755745480048404};
  double butter1_44_1000b[2] = {0.122127259975798, 0.122127259975798};
  double butter2_2_500a[3] = {1, -1.964460580205232, 0.965081173899135};
  double butter2_2_500b[3] = {1.551484234757206e-04, 3.102968469514411e-04, 
                            1.551484234757206e-04};
                            
  // if read from file	
	FILE * readfile;
	double trash;
	if(READ_FROM_FILE)	
	  readfile = 
	    fopen(FILE_TO_READ,"r");	
	
	
	/*** READ PARAMETERS ***/  
	
  paramfile = fopen ("paramfile.txt","r");
  if( eegsig.read_parameters() < 0 )  // run roscd biosemi first
    return -1; 
  fclose(paramfile);
    	
  if(eegsig.inchansLowD == 0)
	  chansIn = eegsig.inchans;
	else
	  chansIn = eegsig.inchansLowD;
    
  if(eegsig.outchansHighD == 0)
	  chansOut = eegsig.outchans;
	else
	  chansOut = eegsig.outchansHighD;
		
	
	/*** READ ARGUMENTS ***/
	
	if (argc<2)
	{
		printf("Usage: No File to Write (1st argument)..\n");
		printf("Try: 'rosrun rqt_plot rqt_plot' for online visualization..\n");
	}
	else	
	{
		/* open and clear file */
		writeFile = 1;
		eegfile = fopen (argv[1],"w");
		printf("Output file: '%s'\n",argv[1]);
	}
	
	if (argc<3)
	{
		printf("Unlimitted acquisition. Press Ctrl-C to end..\n");
		printf("Usage: 2nd argument is time limit (optional)..\n");
	}	
	else
	{
		time_limit = atoi(argv[2]);
		printf("Acquisition duration: %d sec\n", time_limit);
	}

  
  /*** START ACQUISITION ***/  
  
  if(!READ_FROM_FILE)
	{
  	if(biosemi.Biosemi_Acq_Init() < 0)
  		return -1;	
  		
  	if(biosemi.Biosemi_Acq_Start() < 0)
  		return -1;
	}	
  /*** Past this point, the ring buffer is constantly filled ***/
  
  
	/*** LOOP READ CALLS ***/
	
	// get startTime 	
  startTime = ros::Time::now();
	printf("*** BIOSEMI START *** \a\n"); // play beep sound
		 
	while(ros::ok())
	{	
		// get timestamp
    readTime = ros::Time::now();
    eegsig.samplesTime = readTime.toSec();
		
		/*** READ CALL ***/ 
     
    if(READ_FROM_FILE)
    { // read from file
      for(int i=0; i<BIO_SAMPLES; i++)
      {
        for(int j=0; j<BIO_CHANS; j++)
        {
          if(fscanf(readfile, "%ld", &biosemi.datum[i][j]) < 1 )
            return -1;
        }
        fscanf(readfile, "%lf", &trash);
      }
      bioSamplesRead = BIO_SAMPLES;
    }
    else 
    {
      // read ( 29 Kbyte = 128 samples) from device
		  bioSamplesRead = biosemi.Biosemi_Acq_Get(); 
		  if(bioSamplesRead < 0)
			  break;
    }
    			
		/*** LOOP FOR ALL SAMPLES READ ***/ 	
		 
		for(int i=0; i<bioSamplesRead; i++) 
		{	 
			/* @ BIO_FREQ  */
			
			for(int j=0; j<BIO_CHANS; j++)
			  eegsig.inSample[j] = (double)biosemi.datum[i][j] / 10000;
			
			/*** PROCESSING ***/
		      
		  // filter
		  eegsig.push_buffers(0);  
		  eegsig.iir_filter(0, 1, 1, butter1_44_1000a, butter1_44_1000b);  
		  eegsig.push_buffers(1); 
			
			
			/* @ NEW_FREQ */
		      
		  if( eegsig.resampleTime() ) 
		  {
		    // filter
		    eegsig.push_buffers(2);
		    eegsig.iir_filter(2, 3, 2, butter2_2_500a, butter2_2_500b);  
		    eegsig.push_buffers(3); 
		    
		    // todo: centerdata() must start counting after 1st second
		    eegsig.centerdata();
		    eegsig.spatialFilter();
		    eegsig.channel_selection(); 		        
		    eegsig.pca();
		    
		    
		    /*** DECODING ***/
		    
		    eegsig.mapminmax(); 
		    
		    if(SS_OR_NN == 1)
		      eegsig.ssDecode();
		    else  
		      eegsig.nnDecode();
		    
		    eegsig.mapminmaxInv();
		    
		    eegsig.pcaInv();		    
		    
		    /*** WRITE FILE ***/
		    
		    if(writeFile)
		    {
		      if(WRITE_IN_OR_OUT == 1)
		      {
		        for(int j=0; j<chansIn; j++)
		         fprintf(eegfile,"%lf\t", eegsig.inSample[j]); 
		      }
		      else
		      {
		        for(int j=0; j<chansOut; j++)
		         fprintf(eegfile,"%lf\t", eegsig.outSample[j]); 
		      }
		      
		      fprintf(eegfile,"%.6lf\n",eegsig.samplesTime);
		    } 
		    	        
		    /*** PUBLISH MESSAGE ***/ 
		    
		    if(PA10_CONTROL)
		    {
		      pa10Cmd.header.stamp = ros::Time::now();
		      pa10Cmd.vector.x = eegsig.outSample[1]; // pa10.x = shoulder.y
		      pa10Cmd.vector.y = eegsig.outSample[2]; // pa10.y = shoulder.z
		      pa10Cmd.vector.z = eegsig.outSample[0]; // pa10.z = shoulder.x
		    
		      pa10Publisher.publish(pa10Cmd);
		    }
		        
        msg.timeSec = eegsig.samplesTime;
		    for(int j=0; j<chansIn; j++)
		    {
		      msg.raw[j] = eegsig.buffers[0][j][0];	          	          
		      msg.in[j] = eegsig.inSample[j];	          	          
		    }
		    for(int j=0; j<chansOut; j++)
		      msg.out[j] = eegsig.outSample[j];	          	          
		    	                    
		    eegPublisher.publish(msg);
		    
		    eegsig.samplesTime = eegsig.samplesTime + (double)1/NEW_FREQ;
		    
		  } // if( eegsig.resampleTime() ) 
		        
		  eegsig.samplesCounter++;
			
		}	// for(int i=0; i<bioSamplesRead; i++)	
		
		/*** CHECK TIME LIMIT ***/
		/*
		if(!READ_FROM_FILE)
		{
		  if(biosemi.Biosemi_Acq_Status())
		  	printf("@ %.3f\n", eegsig.samplesTime-startTime.toSec());
		}
		*/
		if ((time_limit > 0) && (readTime.toSec() - startTime.toSec() >= time_limit))
		{
			printf("Time limit (%d seconds) reached..\n", time_limit);
			break; 
		}
    
    ros::spinOnce();
    loopRate.sleep();
     
	} // while(ros::ok())

	biosemi.Biosemi_Acq_Close();
	
	if(writeFile)
    fclose(eegfile);
  if(READ_FROM_FILE)  
    fclose(readfile);
    
  printf("\n*** BIOSEMI END *** \a\n");
		
	return 0; 
}

/*****************************  FUNCTIONS  ************************************/

Eeg_Processing::Eeg_Processing() 
{
  samplesCounter = 1;
  samplesTime = 0;
  spatialFilterType = 0;
     
  for(int i=0;i<BUFFERS_NUMBER;i++)
    for(int j=0;j<BIO_CHANS;j++)
      for(int k=0;k<BUFFER_ORDER+1;k++)
  	    buffers[i][j][k] = 0;
  
  rmean.lastindex = 0;
  for(int i=0;i<BIO_CHANS;i++)
    rmean.lastmean[i] = 0;  
  
  inchans = 1;  	     
  for(int i=0;i<BIO_CHANS;i++)
    inSample[i] = 0;
  
  for(int i=0;i<BIO_CHANS;i++)
    chIndex[i] = i;
      
  inchansLowD = 0;
  for(int i=0;i<BIO_CHANS;i++)
    for(int j=0;j<BIO_CHANS;j++)
      {
        if(i == j)
          pcaMatrixIn[i][j] = 1;
        else
          pcaMatrixIn[i][j] = 0;  
      }
  
  for(int i=0;i<BIO_CHANS;i++)
    for(int j=0;j<2;j++)
    {
      if(j == 0)
        mapminmaxIn[i][j] = 1;
      if(j == 1)
        mapminmaxIn[i][j] = -1;   
    }      
   
  outchans = 1;
  for(int i=0;i<BIO_CHANS;i++)
    outSample[i] = 0;
  
  ssorder = 1;
  for(int i=0;i<SS_SIZE;i++)
    ssX[i] = 0;
  for(int i=0;i<SS_SIZE;i++)
    for(int j=0;j<SS_SIZE;j++)
    {
      ssA[i][j] = 0;
      ssB[i][j] = 0;
      ssC[i][j] = 0;
    } 
      
  nnLayers = 2;  
  for(int i=0;i<NN_SIZE;i++)
  {
    neuronsPerLayer[i] = 0;
    b1[i] = 0;
    b2[i] = 0;
    b3[i] = 0;  
  } 
  for(int i=0;i<NN_SIZE;i++)
    for(int j=0;j<NN_SIZE;j++)
    {
      w1[i][j] = 0;
      w2[i][j] = 0;
      w3[i][j] = 0;
    } 
    
  for(int i=0;i<BIO_CHANS;i++)
    for(int j=0;j<2;j++)
    {
      if(j == 0)
        mapminmaxOut[i][j] = 1;
      if(j == 1)
        mapminmaxOut[i][j] = -1;   
    } 
	 
  outchansHighD = 0;
  for(int i=0;i<BIO_CHANS;i++)
    for(int j=0;j<BIO_CHANS;j++)
      {
        if(i == j)
          pcaMatrixOutT[i][j] = 1;
        else
          pcaMatrixOutT[i][j] = 0;  
      }
  
  for(int i=0;i<BIO_CHANS;i++)
    meanOut[i] = 0;
                                 
}

Eeg_Processing::~Eeg_Processing() {}

int 
Eeg_Processing::read_parameters()
{
  char paramname[100];
  int chans = 0;
  int f = 0;
  
  fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "spatialFilter" ) == 0 )  // read spatialFilter
		fscanf(paramfile, "%d", &spatialFilterType);
	else
	{  
	  printf("Parameters: 'spatialFilter' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	
  fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "channelsin" ) == 0 )  // read channelsin
	{
		fscanf(paramfile, "%d", &inchans);
		for(int i=0; i<inchans; i++)
		 if(fscanf(paramfile, "%d", &chIndex[i]) < 1 )
		  {printf("Parameters: 'channelsin' error..\n"); return -1;}
	}
	else
	{  
	  printf("Parameters: 'channelsin' not found in 'paramfile.txt'..\n");
	  return -1;
	}
			  
	fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "pcaIn" ) == 0 ) // read pcaIn
	{
		fscanf(paramfile, "%d", &inchansLowD);
		for(int i=0; i<inchansLowD; i++)
		  for(int j=0; j<inchans; j++)
		    if( fscanf(paramfile, "%lf", &pcaMatrixIn[i][j]) < 1 )
		      {printf("Parameters: 'pcaIn' error..\n"); return -1;}
	}
	else
	{  
	  printf("Parameters: 'pcaIn' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	
	if(inchansLowD == 0)
    chans = inchans;
  else
    chans = inchansLowD; 
	
	fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "mapIn" ) == 0 ) // read mapIn
	{
		for(int i=0; i<chans; i++)
		  for(int j=0; j<2; j++)
		    if( fscanf(paramfile, "%lf", &mapminmaxIn[i][j]) < 1 )
		      {printf("Parameters: 'mapIn' error..\n"); return -1;}
	}
	else
	{  
	  printf("Parameters: 'mapIn' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	
	fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "channelsout" ) == 0 )  // read channelsout
	  fscanf(paramfile, "%d", &outchans); 
	else
	{  
	  printf("Parameters: 'channelsout' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	
	if(SS_OR_NN == 1)
	{
	  fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "ssOrder" ) == 0 )  // read ssOrder
  		fscanf(paramfile, "%d", &ssorder);
  	else
  	{  
  	  printf("Parameters: 'ssOrder' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	  
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "ssA" ) == 0 ) // read ssA
  	{
  		for(int i=0; i<ssorder; i++)
  		  for(int j=0; j<ssorder; j++)
  		    if( fscanf(paramfile, "%lf", &ssA[i][j]) < 1 )
  		      {printf("Parameters: 'ssA' error..\n"); return -1;} 
  	}
  	else
  	{  
  	  printf("Parameters: 'ssA' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "ssB" ) == 0 ) // read ssB
  	{
  		for(int i=0; i<ssorder; i++)
  		  for(int j=0; j<chans; j++)
  		    if(fscanf(paramfile, "%lf", &ssB[i][j]) < 1)
  		      {printf("Parameters: 'ssB' error..\n"); return -1;}
  	}
  	else
  	{  
  	  printf("Parameters: 'ssB' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	  
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "ssC" ) == 0 ) // read ssC
  	{
  		for(int i=0; i<outchans; i++)
  		  for(int j=0; j<ssorder; j++)
  		    if( fscanf(paramfile, "%lf", &ssC[i][j]) < 1 )
  		      {printf("Parameters: 'ssC' error..\n"); return -1;}
  	}
  	else
  	{  
  	  printf("Parameters: 'ssC' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
	}
	else
	{
	  fscanf(paramfile, "%s", paramname);
	  if( strcmp( paramname, "nnLayers" ) == 0 )  // read nnLayers
  	{
  		fscanf(paramfile, "%d", &nnLayers);
  		for(int i=0; i<nnLayers; i++)
  		 if(fscanf(paramfile, "%d", &neuronsPerLayer[i]) < 1 )
  		  {printf("Parameters: 'neuronsPerLayer' error..\n"); return -1;}
  	}
  	else
  	{  
  	  printf("Parameters: 'nnLayers' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	  
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "w1" ) == 0 ) // read w1
  	{
  		for(int i=0; i<neuronsPerLayer[0]; i++)
  		  for(int j=0; j<chans; j++)
  		    if( fscanf(paramfile, "%lf", &w1[i][j]) < 1 )
  		      {printf("Parameters: 'w1' error..\n"); return -1;} 
  	}
  	else
  	{  
  	  printf("Parameters: 'w1' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "w2" ) == 0 ) // read w2
  	{
  		for(int i=0; i<neuronsPerLayer[1]; i++)
  		  for(int j=0; j<neuronsPerLayer[0]; j++)
  		    if( fscanf(paramfile, "%lf", &w2[i][j]) < 1 )
  		      {printf("Parameters: 'w2' error..\n"); return -1;} 
  	}
  	else
  	{  
  	  printf("Parameters: 'w2' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "w3" ) == 0 ) // read w3
  	{
  		for(int i=0; i<outchans; i++)
  		  for(int j=0; j<neuronsPerLayer[1]; j++)
  		    if( fscanf(paramfile, "%lf", &w3[i][j]) < 1 )
  		      {printf("Parameters: 'w3' error..\n"); return -1;} 
  	}
  	else
  	{  
  	  printf("Parameters: 'w3' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	
  fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "b1" ) == 0 )  // read b1
  	{
  		for(int i=0; i<neuronsPerLayer[0]; i++)
  		 if(fscanf(paramfile, "%lf", &b1[i]) < 1 )
  		  {printf("Parameters: 'b1' error..\n"); return -1;}
  	}
  	else
  	{  
  	  printf("Parameters: 'b1' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "b2" ) == 0 )  // read b2
  	{
  		for(int i=0; i<neuronsPerLayer[1]; i++)
  		 if(fscanf(paramfile, "%lf", &b2[i]) < 1 )
  		  {printf("Parameters: 'b2' error..\n"); return -1;}
  	}
  	else
  	{  
  	  printf("Parameters: 'b2' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
  	
  	fscanf(paramfile, "%s", paramname);
  	if( strcmp( paramname, "b3" ) == 0 )  // read b3
  	{
  		for(int i=0; i<outchans; i++)
  		 if(fscanf(paramfile, "%lf", &b3[i]) < 1 )
  		  {printf("Parameters: 'b3' error..\n"); return -1;}
  	}
  	else
  	{  
  	  printf("Parameters: 'b3' not found in 'paramfile.txt'..\n");
  	  return -1;
  	}
 	}  	
  	
fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "mapOut" ) == 0 ) // read mapOut
	{
		for(int i=0; i<outchans; i++)
		  for(int j=0; j<2; j++)
		    if( fscanf(paramfile, "%lf", &mapminmaxOut[i][j]) < 1 )
		      {printf("Parameters: 'mapOut' error..\n"); return -1;}
	}
	else
	{  
	  printf("Parameters: 'mapOut' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	  
	fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "pcaOutT" ) == 0 ) // read pcaOutT
	{
		fscanf(paramfile, "%d", &outchansHighD);
		for(int i=0; i<outchansHighD; i++)
		  for(int j=0; j<outchans; j++)
		    if( fscanf(paramfile, "%lf", &pcaMatrixOutT[i][j]) < 1 )
		      {printf("Parameters: 'pcaOutT' error..\n"); return -1;}
	}
	else
	{  
	  printf("Parameters: 'pcaOutT' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	
	if(outchansHighD == 0)
    chans = outchans;
  else
    chans = outchansHighD;
	  
	fscanf(paramfile, "%s", paramname);
	if( strcmp( paramname, "meanout" ) == 0 ) // read meanout
	{
		for(int i=0; i<chans; i++)
		  if( fscanf(paramfile, "%lf", &meanOut[i]) < 1 )
		    {printf("Parameters: 'meanout' error..\n"); return -1;}
	}
	else
	{  
	  printf("Parameters: 'meanout' not found in 'paramfile.txt'..\n");
	  return -1;
	}
	
	cout<<"Successfully Read Parameters!"<<endl;
	return 0;
}

void
Eeg_Processing::centerdata()
{
  if(samplesCounter>BIO_FREQ)
  {
    for(int j=0; j<BIO_CHANS; j++)
  	{
  
  	 rmean.lastmean[j] = ( rmean.lastmean[j] * rmean.lastindex 
  	          + inSample[j] ) / (rmean.lastindex + 1); 
  	 inSample[j] = inSample[j] - rmean.lastmean[j];
  	  
  	}
  	rmean.lastindex++;
	}
	
	return;
}

void
Eeg_Processing::iir_filter(int bufferIn, int bufferOut, 
                  int iirOrder, double a_coeff[], double b_coeff[])
{ 
  double suma, sumb;
  
  for(int k=0; k<BIO_CHANS; k++)
  {  
    suma = 0;
    sumb = 0;
  
    for(int j=0; j<iirOrder+1; j++)
    {
      sumb = sumb + b_coeff[j]*buffers[bufferIn][k][j];
      if(j>0)
        suma = suma + a_coeff[j]*buffers[bufferOut][k][j-1];
    }
    
    inSample[k] = ( sumb - suma ) / a_coeff[0]; 
    
  }
  
  return; 
}

void 
Eeg_Processing::push_buffers(int buffer)
{
  for(int k=0; k<BIO_CHANS; k++)
  {
    for(int j=BUFFER_ORDER-1; j>-1; j--)
    {
      if(j == 0)
        buffers[buffer][k][j] = inSample[k];
      else
        buffers[buffer][k][j] = buffers[buffer][k][j-1];
    }
  }
  	        
  return;
}

bool 
Eeg_Processing::resampleTime()
{
  long int x;
  double y,modulo;
  
  x = ( floor((double)BIO_FREQ/NEW_FREQ) + samplesCounter );
  y = (double) BIO_FREQ/NEW_FREQ ;
  modulo  = x - floor(x/y) * y;
  
  return ( (int)floor(modulo) == 0 );
}

void 
Eeg_Processing::spatialFilter()
{
  double mean = 0;
  double std = 0;
  double spatialSample[BIO_CHANS];
  
  switch(spatialFilterType)
  {
    case 1: // small Laplacian
    
      spatialSample[0] = inSample[0] -
        ( inSample[2] + inSample[1] + inSample[29]) / 3;
      spatialSample[1] = inSample[1] -
        ( inSample[0] + inSample[2] + inSample[3] + inSample[30] ) / 4;
      spatialSample[2] = inSample[2] -
        ( inSample[1] + inSample[3] + inSample[5] ) / 3;
      spatialSample[3] = inSample[3] -
        ( inSample[1] + inSample[2] + inSample[5] + 
          inSample[4] + inSample[30] ) / 5; 
      spatialSample[4] = inSample[4] -
        ( inSample[3] + inSample[5] + inSample[7] + 
          inSample[31] + inSample[25] + inSample[30] ) / 6;  
      spatialSample[5] = inSample[5] -
        ( inSample[2] + inSample[3] + inSample[4] + 
          inSample[7] + inSample[6]) / 5;
      spatialSample[6] = inSample[6] -
        ( inSample[5] + inSample[7] + inSample[9] ) / 3;
      spatialSample[7] = inSample[7] -
        ( inSample[4] + inSample[5] + inSample[6] + 
          inSample[9] + inSample[8] + inSample[31] ) / 6;    
      spatialSample[8] = inSample[8] -
        ( inSample[7] + inSample[9] + inSample[11] + 
          inSample[12] + inSample[21] + inSample[31] ) / 6;
      spatialSample[9] = inSample[9] -
        ( inSample[6] + inSample[7] + inSample[8] + 
          inSample[10] + inSample[11] ) / 5;
      spatialSample[10] = inSample[10] -
        ( inSample[9] + inSample[11] + inSample[13] ) / 3;
      spatialSample[11] = inSample[11] -
        ( inSample[8] + inSample[9] + inSample[10] + 
          inSample[13] + inSample[12] ) / 5;
      spatialSample[12] = inSample[12] -
        ( inSample[8] + inSample[11] + inSample[13] + 
          inSample[17] + inSample[18] + inSample[21] ) / 6;
      spatialSample[13] = inSample[13] -
        ( inSample[10] + inSample[11] + inSample[12] + inSample[14]  ) / 4;
      spatialSample[14] = inSample[14] -
        ( inSample[13] + inSample[15] ) / 2;
      spatialSample[15] = inSample[15] -
        ( inSample[14] + inSample[16] ) / 2;
      spatialSample[16] = inSample[16] -
        ( inSample[15] + inSample[17] ) / 2;
      spatialSample[17] = inSample[17] -
        ( inSample[12] + inSample[18] + inSample[19] + inSample[16] ) / 4;
      spatialSample[18] = inSample[18] -
        ( inSample[12] + inSample[17] + inSample[19] + 
          inSample[20] + inSample[21] ) / 5;
      spatialSample[19] = inSample[19] -
        ( inSample[17] + inSample[18] + inSample[20] ) / 3;
      spatialSample[20] = inSample[20] -
        ( inSample[19] + inSample[18] + inSample[21] + 
          inSample[22] + inSample[23] ) / 5;
      spatialSample[21] = inSample[21] -
        ( inSample[8] + inSample[12] + inSample[18] + 
          inSample[20] + inSample[22] + inSample[31] ) / 6;
      spatialSample[22] = inSample[22] -
        ( inSample[20] + inSample[23] + inSample[24] + 
          inSample[25] + inSample[31] + inSample[21] ) / 6;
      spatialSample[23] = inSample[23] -
        ( inSample[20] + inSample[22] + inSample[24] ) / 3;
      spatialSample[24] = inSample[24] -
        ( inSample[23] + inSample[22] + inSample[25] + 
          inSample[26] + inSample[27] ) / 5;
      spatialSample[25] = inSample[25] -
        ( inSample[4] + inSample[31] + inSample[22] + 
          inSample[24] + inSample[26] + inSample[30] ) / 6;
      spatialSample[26] = inSample[26] -
        ( inSample[25] + inSample[24] + inSample[27] + 
          inSample[28] + inSample[31] ) / 5;
      spatialSample[27] = inSample[27] -
        ( inSample[28] + inSample[26] + inSample[24] ) / 3;
      spatialSample[28] = inSample[28] -
        ( inSample[29] + inSample[30] + inSample[26] + inSample[27] ) / 4;
      spatialSample[29] = inSample[29] -
        ( inSample[27] + inSample[28] + inSample[0] ) / 3;
      spatialSample[30] = inSample[30] -
        ( inSample[1] + inSample[3] + inSample[4] + 
          inSample[25] + inSample[26] + inSample[28] ) / 6; 
      spatialSample[31] = inSample[31] -
        ( inSample[4] + inSample[7] + inSample[8] + 
          inSample[21] + inSample[22] + inSample[25] ) / 6;
      
      for(int k=0; k<BIO_CHANS; k++)
        inSample[k] = spatialSample[k];    
    
      break;
      
    case 2: // CAR
      
      for(int k=0; k<BIO_CHANS; k++)
        mean = mean + inSample[k];
      mean = mean / BIO_CHANS;    
      //for(int k=0; k<BIO_CHANS; k++)
      //  std = ( inSample[k] - mean ) * ( inSample[k] - mean );
      //std = sqrt( std / BIO_CHANS );
      
      for(int k=0; k<BIO_CHANS; k++)
        inSample[k] = ( inSample[k] - mean ); // / std;   
            
      break;
            
    case 3: // Lobe Dif. and Sum.
      
      // frontal
      spatialSample[0] = inSample[0] - inSample[29];
      spatialSample[1] = inSample[0] + inSample[29];
      
      spatialSample[2] = inSample[1] - inSample[28];
      spatialSample[3] = inSample[1] + inSample[28];
      
      spatialSample[4] = inSample[2] - inSample[27];
      spatialSample[5] = inSample[2] + inSample[27];
      
      spatialSample[6] = inSample[3] - inSample[26];
      spatialSample[7] = inSample[3] + inSample[26];
      
      spatialSample[8] = inSample[30] -
        ( inSample[1] + inSample[28] + inSample[3] + 
          inSample[26] + inSample[4] + inSample[25] ) / 6;
      
      // cerebral
      spatialSample[9] = inSample[4] - inSample[25];
      spatialSample[10] = inSample[4] + inSample[25];
      
      spatialSample[11] = inSample[5] - inSample[24];
      spatialSample[12] = inSample[5] + inSample[24];
      
      spatialSample[13] = inSample[7] - inSample[22];
      spatialSample[14] = inSample[7] + inSample[22];
       
      spatialSample[15] = inSample[31] -
        ( inSample[4] + inSample[7] + inSample[8] + 
          inSample[21] + inSample[22] + inSample[25] ) / 6;
       
      // parietal 
      spatialSample[16] = inSample[8] - inSample[21];
      spatialSample[17] = inSample[8] + inSample[21];
      
      spatialSample[18] = inSample[9] - inSample[20];
      spatialSample[19] = inSample[9] + inSample[20];
      
      spatialSample[20] = inSample[10] - inSample[19];
      spatialSample[21] = inSample[10] + inSample[19];
      
      spatialSample[22] = inSample[11] - inSample[18];
      spatialSample[23] = inSample[11] + inSample[18];
      
      spatialSample[24] = inSample[12] -
        ( inSample[8] + inSample[21] + inSample[11] + 
          inSample[18] + inSample[13] + inSample[17] ) / 6;
      
      // occipital
      spatialSample[25] = inSample[13] - inSample[17];
      spatialSample[26] = inSample[13] + inSample[17];
      
      spatialSample[27] = inSample[14] - inSample[16];
      spatialSample[28] = inSample[14] + inSample[16];
      
      spatialSample[29] = inSample[15] -
        ( inSample[13] + inSample[17] + inSample[14] + inSample[16]  ) / 4;
      
      // temporal
      spatialSample[30] = inSample[6] - inSample[23];
      spatialSample[31] = inSample[6] + inSample[23];
      
      for(int k=0; k<BIO_CHANS; k++)
        inSample[k] = spatialSample[k];
        
      break;
      
    default:
      
      break;
  } 
  
  return;
}
 
void 
Eeg_Processing::channel_selection()
{
  double newSample[BIO_CHANS] = {};
  
  for(int k=0; k<inchans; k++)
	  newSample[k] = inSample[chIndex[k]];
	  
	for(int k=0; k<BIO_CHANS; k++)
	  inSample[k] = newSample[k];
  
  return;
}
 
void 
Eeg_Processing::pca()
{
  double inSamplePca[BIO_CHANS] = {};
    
  for(int j=0; j<BIO_CHANS; j++)
    inSamplePca[j] = 0;
    
  // pcaMatrix * inSample  
  for(int i=0; i<inchansLowD; i++)
    for(int k=0; k<inchans; k++)
      inSamplePca[i] = inSamplePca[i] + pcaMatrixIn[i][k] * inSample[k];
        
  for(int j=0; j<inchansLowD; j++)
    inSample[j] = inSamplePca[j];
    
  return;
}

void 
Eeg_Processing::mapminmax()
{
  int chans = 0;
  
  if(inchansLowD == 0)
    chans = inchans;
  else
    chans = inchansLowD;
    
  for(int j=0; j<chans; j++)
    inSample[j]  = 2 * ( inSample[j] - mapminmaxIn[j][1] ) / 
                    ( mapminmaxIn[j][0] - mapminmaxIn[j][1]) - 1;
                    
  return;    
}

void 
Eeg_Processing::ssDecode()
{
  int chans = 0;
  
  if(inchansLowD == 0)
    chans = inchans;
  else
    chans = inchansLowD;
    
  double ax[BIO_CHANS] = {};
  double bu[BIO_CHANS] = {};
    
  for(int i=0; i<BIO_CHANS; i++)
  {
    ax[i] = 0;
    bu[i] = 0;
    outSample[i] = 0;
  }  
    
  // ssA * ssX  
  for(int i=0; i<ssorder; i++)
    for(int k=0; k<ssorder; k++)
      ax[i] = ax[i] + ssA[i][k] * ssX[k];
  
  // ssB * inSample
  for(int i=0; i<ssorder; i++)
    for(int k=0; k<chans; k++)
      bu[i] = bu[i] + ssB[i][k] * inSample[k];
  
  for(int i=0; i<ssorder; i++)
    ssX[i] = ax[i] + bu[i];
  
  // ssC * ssX'
  for(int i=0; i<outchans; i++)
    for(int k=0; k<ssorder; k++)
      outSample[i] = outSample[i] + ssC[i][k] * ssX[k];
      
  return;
}

void 
Eeg_Processing::nnDecode()
{
  int chans = 0;
  
  if(inchansLowD == 0)
    chans = inchans;
  else
    chans = inchansLowD;
    
  double a1[NN_SIZE] = {};
  double a2[NN_SIZE] = {};  
    
  for(int i=0; i<NN_SIZE; i++)
  {
    a1[i] = 0;
    a2[i] = 0;
  }  
  for(int i=0; i<BIO_CHANS; i++)
    outSample[i] = 0;
     
  // a1 = tansig( w1 * inSample + b1)  
  for(int i=0; i<neuronsPerLayer[0]; i++)
    for(int k=0; k<chans; k++)
      a1[i] = a1[i] + w1[i][k] * inSample[k];
  for(int i=0; i<neuronsPerLayer[0]; i++)
    a1[i] = 2.0 / ( 1 + exp( -2 * ( a1[i] + b1[i]) ) ) - 1;
  
  // a2 = tansig( w2 * a1 + b2)  
  for(int i=0; i<neuronsPerLayer[1]; i++)
    for(int k=0; k<neuronsPerLayer[0]; k++)
      a2[i] = a2[i] + w2[i][k] * a1[k];
  for(int i=0; i<neuronsPerLayer[1]; i++)
    a2[i] = 2.0 / ( 1 + exp( -2 * ( a2[i] + b2[i]) ) ) - 1;
  
  // outSample =  w3 * a2 + b3  
  for(int i=0; i<outchans; i++)
    for(int k=0; k<neuronsPerLayer[1]; k++)
      outSample[i] = outSample[i] + w3[i][k] * a2[k];
  for(int i=0; i<outchans; i++)
    outSample[i] = outSample[i] + b3[i];
      
  return;
}

void 
Eeg_Processing::mapminmaxInv()
{
     
  for(int j=0; j<outchans; j++)
    outSample[j]  =  ( mapminmaxOut[j][0] - mapminmaxOut[j][1]) 
                        * ( outSample[j] + 1 ) / 2 + mapminmaxOut[j][1];
                    
  return;    
}

void 
Eeg_Processing::pcaInv()
{
  double outSamplePca[BIO_CHANS] = {};
    
  for(int j=0; j<BIO_CHANS; j++)
    outSamplePca[j] = 0;
  
  // pcaMatrix' * outSample  
  for(int i=0; i<outchansHighD; i++)
    for(int k=0; k<outchans; k++)
      outSamplePca[i] = outSamplePca[i] + pcaMatrixOutT[i][k] * outSample[k];
        
  for(int j=0; j<outchansHighD; j++)
    outSample[j] = outSamplePca[j] + meanOut[j];
    
  return;
}

/***************************   Biosemi_Acq   **********************************/

Biosemi_Acq::Biosemi_Acq()
{
	ringBufferSize = 59392; //65536
	bytes2Use = ((INT_PTR)ringBufferSize)*512;
	nbuffers = 0; 
  nextSync = 0;
	seam = 0;
  orion = 0;
	lastSeam = 0;
	bytesPerMsec = 0;
	throughPut = 0;
	
	/* allocate ring buffer */
	ringBuffer = (PUCHAR) malloc( (SIZE_T) bytes2Use);
	if (ringBuffer == 0)
	{
		printf(" Memory allocation error : %d\n", errno);
		abort();
	}
	
}

Biosemi_Acq::~Biosemi_Acq()
{
	free(ringBuffer);
}

int 
Biosemi_Acq::Biosemi_Acq_Init(void)
{	
	/* Step.1 Open USB2 driver */
	if ((handle=OPEN_DRIVER()) == (HANDLE)NULL)
	{
		printf("Can't open device driver!\n");
		return -1;
	}

	/* Step.2 Initialize USB2 interface */
	for (int i=0; i<64; i++)
		controlBuffer[i] = 0; // controlBuffer = zeros(64)

	if (USB_WRITE(handle, &controlBuffer[0]) == FALSE)
	{
		printf ("usb_write for enable handshake trouble %d\n", errno); 
		return -1;
	}

	/* Step.3 Initialize the ring buffer */
	READ_MULTIPLE_SWEEPS(handle, (PCHAR)ringBuffer, bytes2Use);
	
	/* Tuning if no labview_dll.ini file*/
	// BSIF_SET_SYNC(true);
	// BSIF_SET_STRIDE_KB(256); // 131072 bytes

	return 0;
}

int 
Biosemi_Acq::Biosemi_Acq_Start(void)
{
	/* Step.4 Enable the handshake */

	controlBuffer[0] = (CHAR)(-1); // MSByte of controlBuffer = 0xff
	
	if (USB_WRITE(handle, &controlBuffer[0]) == FALSE)
	{
		printf ("usb_write for enable handshake trouble %d\n", errno); 
		return -1;
	}
	
	/*** Past this point, the ring buffer is constantly filled ***/

	return 0;
}

int // returns number of measurements (rows) in datum
Biosemi_Acq::Biosemi_Acq_Get(void)
{
	BOOL read_ptr;
	int row = 0;
	
	/* Step.5 Read 128Kbyte from Buffer */	
	if ((read_ptr=READ_POINTER(handle, &seam)) == FALSE)
	{
		printf("Acquisition not running!\n");
		return -1;
	}
	
	/* Check seam */
	if (seam == lastSeam) 
	{	
		printf("No new data measured..\n");
		return -1; // exit main loop
	}
	
	/* define orion (longs read) */
	orion = seam - lastSeam;
	if(orion<0)
		orion += bytes2Use;
	orion /=4;
	
	lastSeam = seam;

	/*** For every data measurement ***/		
	for (int ii=0; ii<=orion; ii+=NUM_CHANNELS) 
	{
		/* Handle nextSync */
		if((nextSync+NUM_CHANNELS-1)*4 >= bytes2Use)
			nextSync += NUM_CHANNELS;// word does not fit in buffer
		if(nextSync*4 >= bytes2Use)
			nextSync -= (INT)bytes2Use/4;//loop index
		if((seam >= nextSync*4) && ((nextSync+NUM_CHANNELS-1)*4 >= seam))
			break; //word has not been read entirely

		/* check missing sync */
		/* cmPattern: "sync - nosync - nosync" */
		if(!((((unsigned int *)ringBuffer)[nextSync] == 0xffffff00) &&
			 (((unsigned int *)ringBuffer)[nextSync+1] != 0xffffff00) &&
			 (((unsigned int *)ringBuffer)[nextSync+2] != 0xffffff00)))
		{
			printf("missing sync: #buffer %d, @ %d\n", nbuffers, nextSync);
			return -1;		
		}
					
		/* store every data measurement (in uV) to datum */		
		for(int i=2; i<BIO_CHANS+2; i++) 
			datum[row][i-2] = ((int *)ringBuffer)[nextSync+i]/8192;  
		row++;		
		
		nextSync+=NUM_CHANNELS;	
	} 
	
	nbuffers++;
	if(nbuffers>1023)
		nbuffers = 0;
		
	return row;
}

int 
Biosemi_Acq::Biosemi_Acq_Close(void)
{
	/* Display Driver Information */
	GET_DRIVER_INFO(infoBuffer, sizeof(infoBuffer)); // actual stride 
	//printf("%s\n", infoBuffer);	// determined after the end of 5th buffer	
	
	/* Step.7 Disable the Handshake */
	controlBuffer[0] = 0;

	if (USB_WRITE(handle, (PCHAR)&controlBuffer[0]) == FALSE)
	{
		printf ("USB_WRITE for disable handshake trouble %d\n", errno); 
		return -1;
	}
	
	/* Step.8 Close the drivers & free memory */
	CLOSE_DRIVER(handle);
	
	return 0;
}

int 
Biosemi_Acq::Biosemi_Acq_Status(void)
{
	/*** Status output every PRINT_STATUS_FREQ ***/
	if (nbuffers % PRINT_STATUS_FREQ == 0)
	{
		/* determine throughput in words per channel per second */	
		bytesPerMsec = BSIF_GET_BYTES_PER_MSEC();
		throughPut = (int)(bytesPerMsec*1000./(double)(NUM_CHANNELS*4));
		/* output message */
		printf("#buffer %d, seam %.2lf, lastSync %d, orion %d, throughput %d\t", 
				nbuffers, (double)seam/232, nextSync/58, orion/58, throughPut);
		return 1;
	}
	else
		return 0;
}

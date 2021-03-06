/* ***************************************************** */
/*
 ** 	Charilaos P. Kalavritinos	@ CSL Mech NTUA 				**


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

PARAMETERS
	  0. paramfile.txt includes parameters for processing and decoding procedures
	  1. ClassificationParamfile includes parameters used in classification procedures and creation of weights
      2. MACROS
          READ_FROM_FILE
          FILE_TO_READ
          WRITE_IN_OR_OUT
          PA10_CONTROL etc.

*/
/*
#include "ros/ros.h"
#include "biosemi/biosemiMessage.h"
#include <geometry_msgs/Vector3Stamped.h>
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>

#include <sys/wait.h>
#include <string.h>
#include <sstream>

#include <math.h>
#include <iostream>
#include <fstream>

/*
#include "biosemi/labview_dll.h"
#include "biosemi/bsif.h"
*/

#define BIO_FREQ 16384
#define NEW_FREQ 500
#define SPEED_MODE 7
#define NUM_CHANNELS 58
#define BIO_CHANS 32						// 40 with 8 EX channels
#define PRINT_STATUS_FREQ 64
#define BIO_SAMPLES 128 					// Number of samples per read (stride)

#define BUFFERS_NUMBER 4 					// buffers used for filtering
#define BUFFER_ORDER 10


#define READ_FROM_FILE 1					// if input from file or from device
#define NN_SIZE 30						// max values permitted
#define NN_NEURONS 30
#define FILE_TO_READ 	"./eeg1"
#define WRITE_IN_OR_OUT 0
#define WINDOW 250

using namespace std;

#if defined(_WIN64)
 typedef __int64 INT_PTR;
#else
 typedef int INT_PTR;
#endif

typedef void * HANDLE;
typedef char CHAR;
typedef unsigned char UCHAR;
typedef UCHAR * PUCHAR;

#if defined(_WIN64)
 typedef unsigned __int64 ULONG_PTR;
#else
 typedef unsigned long ULONG_PTR;
#endif

typedef ULONG_PTR SIZE_T;



FILE * paramfile;

class Eeg_Processing
{
private:
  	int chIndex[BIO_CHANS];

  	struct meanStruct {								// rmean calculation
    	unsigned long int lastindex; 				// start from 0
    	double lastmean[BIO_CHANS];
  	} 	rmean;

 	int nnLayers;
 	int neuronsPerLayer[NN_SIZE];
  	int TransferFcn[NN_SIZE];						// for NN classifier

	double w1[NN_NEURONS][WINDOW];
  	double w2[NN_NEURONS][NN_NEURONS];
  	double w3[NN_NEURONS];
  	double b1[NN_NEURONS];
  	double b2[NN_NEURONS];
  	double b3[NN_NEURONS];

  	double mapminmaxIn[BIO_CHANS][2];
  	double mapminmaxOut[BIO_CHANS][2];



public:
  	Eeg_Processing();
	~Eeg_Processing();
	int inchans;
	int outchans;
	unsigned long int counter;

	double inSample[BIO_CHANS];
	double outSample;
	unsigned long int samplesCounter;	// samples counter

	double samplesTime;
	double buffers[BUFFERS_NUMBER][BIO_CHANS][BUFFER_ORDER+1];		// 4 buffers: 4x32x11
	double bufferCL[WINDOW];

	void centerdata();
  	void iir_filter(int bufferIn, int bufferOut, int iirOrder, double a_coeff[], double b_coeff[]);
  	void push_buffers(int buffer);
  	void weight_calculation();
  	bool resampleTime();

    int read_parameters();

	void channel_selection();	// select channels
  	void mapminmax();
  	void mapminmaxInv();
	void nnDecode();

};


class Biosemi_Acq
{
private:
	PUCHAR	ringBuffer;
	CHAR 	controlBuffer[64];
	char	infoBuffer[120];
	INT_PTR	ringBufferSize; 	// 32Mbytes
	INT_PTR bytes2Use;
	HANDLE	handle;

	float 	bytesPerMsec;
	int 	throughPut;
	int 	nbuffers; 		// buffer of 128Kbytes (= stride)
  	int		nextSync;
	int 	lastSync;

	INT_PTR	seam;
  	INT_PTR	orion;
	INT_PTR	lastSeam;

public:
    long int datum[4520][BIO_CHANS]; 	// 565 samples in 128Kbyte
    	// = round [stride / (NUM_CHANNELS * 4 bytes_per_channel)]

    Biosemi_Acq();
	~Biosemi_Acq();
 	int Biosemi_Acq_Init(void);
  	int Biosemi_Acq_Start(void);
  	int Biosemi_Acq_Get(void);
  	int Biosemi_Acq_Close(void);
  	int Biosemi_Acq_Status(void);
};



int main(int argc, char **argv)
{
	Eeg_Processing eegsig;
	Biosemi_Acq biosemi;

	int chansIn = 0;
	int chansOut = 0;

	// OPEN CLASSIFICATION PARAMETER FILE AND READ PARAMETERS
	paramfile = fopen ("CLparamfile.txt","r");

	if( eegsig.read_parameters() < 0 )
	return -1;
	fclose(paramfile);

	int bioSamplesRead;
	int i,j;
	bool writeFile = 1;

	// filters
	double butter1_44_1000a[2] = {1, -0.755745480048404};
  	double butter1_44_1000b[2] = {0.122127259975798, 0.122127259975798};
  	double butter2_2_500a[3] = {1, -1.924872341163554, 0.927307768331003};
  	double butter2_2_500b[3] = {0.036346115834498, 0, -0.036346115834498};

  	// if read from file
	FILE * readfile;
	FILE * outFile;
	FILE * inFile;

	double trash;


	readfile = fopen(FILE_TO_READ,"r");


	if (WRITE_IN_OR_OUT==1) inFile = fopen("input.txt","wt+");
	//save input file or output
	else  outFile = fopen("output.txt","wt+");
	bool isEnd=0;

	cout << "Start: Reading from file...\n";
	system("canberra-gtk-play -f beep.wav");

	/*------------------------- START READING LOOP -------------------------*/
	while(1)
    {
	   if(READ_FROM_FILE)
	    {
	     	bioSamplesRead=0;

	      for(i=0; i<BIO_SAMPLES; i++)
	      {
	        for(j=0; j<BIO_CHANS; j++)
	        {
				long int num;

	          if(fscanf(readfile, "%ld", &num) < 1 )
	          {

	            if(feof(readfile)!=0)
				{
				cout<< "Done reading input SCP file\n";
				isEnd=1;
				break;
				}
			  }
			  else
			  {
	            biosemi.datum[i][j]=num;

	          }
	        }
			fscanf(readfile, "%lf", &trash);

			if (isEnd) break;
	        bioSamplesRead++;
	      }

	    }

		/*** LOOP FOR ALL SAMPLES READ ***/


		for(i=0; i<bioSamplesRead; i++)
		{
		double Cz;
		Cz=(double)biosemi.datum[i][31] / 10000;

			for (j=0; j<BIO_CHANS; j++)
			{
 				eegsig.inSample[j] = (double)biosemi.datum[i][j] / 10000;
 				//re-reference
				eegsig.inSample[j] =eegsig.inSample[j] -Cz;
			}

				/*** PROCESSING ***/
				// filter
				eegsig.push_buffers(0);
				eegsig.iir_filter(0, 1, 1, butter1_44_1000a, butter1_44_1000b);
				eegsig.push_buffers(1);


			if( eegsig.resampleTime() ) 	// Resample to new frequency
			{

				// filter
				eegsig.push_buffers(2);
				eegsig.iir_filter(2, 3, 2, butter2_2_500a, butter2_2_500b);
				eegsig.push_buffers(3);
				// todo: centerdata() must start counting after 1st second
				eegsig.centerdata();
				eegsig.channel_selection();

				/*** CLASSIFIER ***/

				eegsig.mapminmax();
				eegsig.bufferCL[eegsig.counter]=eegsig.inSample[0];

				if (eegsig.counter==WINDOW)
				{
					eegsig.nnDecode();
					//cout<<eegsig.outSample<<endl;
					if(eegsig.outSample>=0.5)
					{
					eegsig.outSample=1;
					system("canberra-gtk-play -f beep.wav");
					}
					else
					eegsig.outSample=0;
					eegsig.counter=125;

					for (int k=0;k<125;k++)
					eegsig.bufferCL[k]=eegsig.bufferCL[k+125];
					for (int k=125;k<250;k++)
					eegsig.bufferCL[k]=0;


					if(writeFile)
					{
						if(WRITE_IN_OR_OUT == 1)
						{
							for (j=0; j<eegsig.inchans; j++)
						         fprintf(inFile,"%lf ", eegsig.inSample[j]);
						}
						else
						{
						    fprintf(outFile,"%lf ", eegsig.outSample);
						}
				    } 	// write file end

				}
			 	else
			 	{
				eegsig.counter++;		// counter for classification
				}
				/*** WRITE FILE ***/


			} 		// if resample end


		eegsig.samplesCounter++;			// counter for resampling
		}		// for samples_read end
	if (isEnd) break;

   }  // while(1) end



	if (WRITE_IN_OR_OUT==1)
	{
	 	fclose(inFile);
	 	cout <<"New file saved!Input\n";
	}
	else
	{
		fclose(outFile);
		cout <<"New file saved! Output\n";
	}

  return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// */-------------------------------------------------------------------------------------------------------------------------*/ //
///////////////////////////////////////////////////  F  U  N  C  T  I  O  N  S  ///////////////////////////////////////////////////
// */-------------------------------------------------------------------------------------------------------------------------*/ //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



Eeg_Processing::Eeg_Processing()
{
	samplesCounter = 1;							//for resampling
  	samplesTime = 0;

	inchans = 1;  							//Number of channels in input and output for DCD and CL
	outchans=1;

	for(int i=0;i<BUFFERS_NUMBER;i++)
    for(int j=0;j<BIO_CHANS;j++)
      for(int k=0;k<BUFFER_ORDER+1;k++)
  	    buffers[i][j][k] = 0;

  	rmean.lastindex = 0;

  	for(int i=0;i<BIO_CHANS;i++)
    	rmean.lastmean[i] = 0;

	for(int i=0;i<BIO_CHANS;i++)
	{
	    inSample[i] = 0;
	}
    for(int i=0;i<BIO_CHANS;i++)
    	chIndex[i] = i;

	//////////////////////////////////

	for (int k=0;k<WINDOW;k++)
		bufferCL[k]=0;

	counter=0;

	//////////////////////////////////

  	for(int i=0;i<BIO_CHANS;i++)
    	for(int j=0;j<2;j++)
    	{
		      if(j == 0)
		      {
		        mapminmaxIn[i][j] = 1;
		        mapminmaxOut[i][j] = 1;
		    	}

		      if(j == 1)
		      {
		      	mapminmaxIn[i][j] = -1;
		        mapminmaxOut[i][j] = -1;
			  }
    	}

  		outSample = 0;

	nnLayers = 0;
	for(int i=0;i<NN_SIZE;i++)
	  {
	    neuronsPerLayer[i] = 0;
	    b1[i] = 0;
	    b2[i] = 0;
	    b3[i] = 0;
	  }

	for(int i=0;i<NN_SIZE;i++)
		for(int j=0;j<WINDOW;j++)
	  	      w1[i][j] = 0;


	for(int i=0;i<NN_SIZE;i++)
		for(int j=0;j<NN_SIZE;j++)
	    {
	      w2[i][j] = 0;
	      w3[j] = 0;
	    }
}

Eeg_Processing::~Eeg_Processing() {};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Eeg_Processing::read_parameters()
{

	string line;
	int tmp,i,j;

	ifstream myfile ("CLparamfile.txt");

	if (myfile.is_open())
	{

		while (!myfile.eof())
		{

			getline(myfile,line);
			line=line.substr(0,line.size()-1);

			string str ="NoChannels:";
			if (str.compare(line)==0)
		    {

		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);

		    	istringstream (line) >> inchans;

		    	for (i=0;i<inchans;i++)
		    	{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> chIndex[i];
					//cout<<chIndexCL[i]<<"\n";
				}

			}

			str="Layers:";
			if (str.compare(line)==0)
		    {
			getline(myfile,line);
			line=line.substr(0,line.size()-1);

			istringstream ( line) >> nnLayers;
			//cout<<"NNlayers:"<<nnLayers<<endl;
			}

			str ="NeuronsPerLayer:";
			if (str.compare(line)==0)
		    {
		    	for (i=0;i<nnLayers;i++)
		    	{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> neuronsPerLayer[i];
					//cout<<"neuronsPerLayer: " << neuronsPerLayer[i] << '\n';

				}

			}

			str ="W1:";
			if (str.compare(line)==0)
		    {
		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);

			istringstream (line) >> tmp;

		    	for (j=0;j<neuronsPerLayer[0];j++)
		    	{
		    		for (int i=0;i<tmp/neuronsPerLayer[0];i++)
		    		{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> w1[j][i];
					//cout<<"w1: " << w1[j][i] << '\n';

					}
				}
			}

			str ="b1:";
			if (str.compare(line)==0)
		    {

		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
			istringstream (line) >> tmp;

		    		for (i=0;i<=tmp-1;i++)
		    		{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> b1[i];
					//cout<<"b1: " << b1[i]<<'\n';
					}

			}

			str ="TransferFcn1:";
			if (str.compare(line)==0)
		    {
		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
			istringstream (line) >> TransferFcn[0];
			}

			str ="W2:";
			if (str.compare(line)==0)
		    {
		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
			istringstream (line) >> tmp;
		    	for (j=0;j<neuronsPerLayer[1];j++)
		    	{
		    		for (int i=0;i<tmp/neuronsPerLayer[1];i++)
		    		{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> w2[j][i];
					//cout <<"w2: "<< w2[j][i] << '\n';

					}
				}
			}
			str ="b2:";
			if (str.compare(line)==0)
		    {

		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
				istringstream (line) >> tmp;

		    		for (i=0;i<=tmp-1;i++)
		    		{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> b2[i];
					//cout << b2[i]<<"\n";
					}
			}
			str ="TransferFcn2:";
			if (str.compare(line)==0)
		    {
		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
			istringstream (line) >> TransferFcn[1];
			}

			str ="W3:";
			if (str.compare(line)==0)
		    {
		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
				istringstream (line) >> tmp;

		    		for (int i=0;i<tmp/neuronsPerLayer[2];i++)
		    		{

					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> w3[i];
				//	cout <<"w3: "<<w3[i]<<"\n";
					}

			}

			str ="b3:";
			if (str.compare(line)==0)
		    {

		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
				istringstream (line) >> tmp;
		    		for (i=0;i<=tmp-1;i++)
		    		{
					getline(myfile,line);
					line=line.substr(0,line.size()-1);
					istringstream (line) >> b3[i];
					}
			}

			str ="TransferFcn3:";
			if (str.compare(line)==0)
		    {
		    	getline(myfile,line);
			line=line.substr(0,line.size()-1);
			istringstream (line) >> TransferFcn[2];
			}


			str ="mapInput:";
			if (str.compare(line)==0)
		    {

				istringstream (line) >> tmp;

		    		for (int i=0;i<inchans;i++)
		    		{
		    			for (int j=0;j<2;j++)
		    			{
		    				getline(myfile,line);
						line=line.substr(0,line.size()-1);
						istringstream (line) >> mapminmaxIn[i][j];

						}
					}
			}
			str ="mapOutput:";
			if (str.compare(line)==0)
		    {


				istringstream (line) >> tmp;
		    			for (int j=0;j<2;j++)
		    			{
		    				getline(myfile,line);
						line=line.substr(0,line.size()-1);
						istringstream (line) >> mapminmaxOut[0][j];

						}
			}

		}
		myfile.close();
		cout<<"Successfully Read CL Parameters."<<endl;
	}
	else cout << "Unable to open file";

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Eeg_Processing::nnDecode()
{
  int chans = WINDOW;

  double a1[NN_SIZE] = {};
  double a2[NN_SIZE] = {};

  for(int i=0; i<NN_SIZE; i++)	// initialize to 0
  {
    a1[i] = 0;
    a2[i] = 0;
  }
    outSample = 0;
	// -----------------------------------------//

	for(int i=0; i<neuronsPerLayer[0]; i++)
	{

		for(int k=0; k<chans; k++)
		{
			a1[i] = a1[i] + w1[i][k] * bufferCL[k];
		}


		if (TransferFcn[0]==1)  										// a1 = tansig( w1 * inSample + b1)
			a1[i] = 2.0 / ( 1 + exp( -2 * ( a1[i] + b1[i]) ) ) - 1;
		else															// Purelin
			a1[i]=a1[i]+b1[i];
	}

			  																	// a2 = tansig( w2 * a1 + b2)
		for(int i=0; i<neuronsPerLayer[1]; i++)
		{
			for(int k=0; k<neuronsPerLayer[0]; k++)
				a2[i] = a2[i] + w2[i][k] * a1[k];

			if (TransferFcn[1]==1)
				a2[i] = 2.0 / ( 1 + exp( -2 * ( a2[i] + b2[i]) ) ) - 1;
			else
				a2[i]=a2[i]+b2[i];
		}

			for(int k=0; k<neuronsPerLayer[1]; k++)
				outSample = outSample + w3[k] * a2[k];

		if (TransferFcn[2]==1)
			outSample = 2.0 / ( 1 + exp( -2 * ( outSample+ b3[0]) ) ) - 1;
		else
			outSample = outSample + b3[0];

	  return;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Eeg_Processing::centerdata()
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Eeg_Processing::iir_filter(int bufferIn, int bufferOut,
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  Eeg_Processing::push_buffers(int buffer)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool  Eeg_Processing::resampleTime()
{
  long int x;
  double y,modulo;

  x = ( floor((double)BIO_FREQ/NEW_FREQ) + samplesCounter );
  y = (double) BIO_FREQ/NEW_FREQ ;
  modulo  = x - floor(x/y) * y;
  return ( (int)floor(modulo) == 0 );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Eeg_Processing::channel_selection()						// selecting channels
{
	double newSample[BIO_CHANS] = {};

	for(int k=0; k<inchans; k++)
		newSample[k] = inSample[chIndex[k]];

	for(int k=0; k<BIO_CHANS; k++)
		inSample[k] = newSample[k];

return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Eeg_Processing::mapminmax()
{
	  	for(int j=0; j<inchans; j++)
	    	inSample[j]  = 2 * ( inSample[j] - mapminmaxIn[j][1] ) /
	                	    ( mapminmaxIn[j][0] - mapminmaxIn[j][1]) - 1;

  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Eeg_Processing::mapminmaxInv()
{

	    outSample  =  ( mapminmaxOut[0][0] - mapminmaxOut[0][1])
	                        * ( outSample + 1 ) / 2 + mapminmaxOut[0][1];
  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

Biosemi_Acq::~Biosemi_Acq()
{
	free(ringBuffer);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <poll.h>

#ifdef __GNUC__
#  if(__GNUC__ > 3 || __GNUC__ ==3)
#       define _GNUC3_
#  endif
#endif

#ifdef _GNUC3_
#  include <iostream>
#  include <fstream>
   using namespace std;
#else
#  include <iostream.h>
#  include <fstream.h>
#endif
#include <cstdlib>
#include <ctime>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "atmcdLXd.h"
#include "atspectrograph.h"
#include <poll.h>
#include <sstream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//all things inet/sock
#include <errno.h>  
#include <arpa/inet.h>    //close  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <sys/time.h> //FD_SET, FD_ISSET, FD_ZERO macros  

#include <sys/ioctl.h>
#include <linux/sockios.h>


#define TRUE   1  
#define FALSE  0  
#define PORT 8888  


//file descriptors
fstream fout;


long getCurrentTimeMillis() { //get current time
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

std::string client_buffer[30];

//Auxiliary function Prototypes
std::string dtToStr(); //function to write a line into the output pipe
int CameraSelect (int iNumArgs, char* szArgList[]);
long GetDataSize();
long datasize;
void DisplayAcquireInfo();
std::string GetAllStats();
void decideCase();
void InitCamActions();
void InitSpectroActions();
int sockHandler();
int sockHandlerNoBlock();

long stopdelay=100; //interval to check for STP command during long acquisitions in ms

//defaults,all globals(!)
int AcqMode=1;		//stat:Acquisition mode
int ReadMode=0;		//stat:Read Mode
float ext=1.0;	//stat:exposition time
int trigMode=0;
int filterMode=0;
int vspeed=0;
int baseline=1;
int numKins=2;		
int numAccs=1;		//stat:number of accums
float kin;	
float acc;	//stat:accumulation time
float rdout;	//stat:readout time
float clnout;	//stat:readout time
int trackpos; //singletrack
int trackwid;	  


int numTracks=1;
//int trackHeight=1;//multitrack
//int trackOffset=0;//multitrack
int Gain=0;		//stat:detector gain
//int trackBottom;	
int hspeed=2;		//stat:horizontal speed index
//int trackGap;
int *randomTracks;
struct Rectangle{
   int top;
   int left;
   int bottom;
   int right;
};
Rectangle subImage={1,1,1024,256};
int hbin=1;
int vbin=1;
int width, height;	//detector resolution
int spool=0;		
int settemp=-40;	//stat:preset temp.
int ngains;
float wavel=0;		//stat:current WL
float xSize, ySize;	//detector pixel size
float Min, Max;		//stat:WL range
int groffs; 		//stat:grating offset
int doffs;		//stat:detector offset
int fmpos;		//stat:focus mirror position
int igrat(0);		//stat:grating
int temp;		//stat:act temp
float speed;		//stat:horizontal speed (MHz)	
float calibrationValues[1024]; //stat:calibration
eATSpectrographPortPosition fport;
eATSpectrographPortPosition ifport;

//laziness rules the world (and leads to bad code)
bool quit;
bool errsyntax;
bool cmdavail=false;
std::string cmd;
std::string cmda;
std::string cmdb;
std::string resp;
float num;
int numi;
int bknumi;
std::string val;
unsigned int error;
eATSpectrographReturnCodes specrographError;
int random_variable = std::rand();
std::string msg;
at_32 *datas = NULL;


int opt = TRUE;   
int master_socket , addrlen , new_socket , client_socket[30] ,max_clients = 5 , activity, valread , sd;   
int max_sd;   
struct sockaddr_in address;   
char buffer[1025];  //data buffer of 1K  
//set of socket descriptors  
fd_set readfds;   
int pending; 



int main(int argc, char* argv[])
{
  	std::srand(std::time(nullptr)); // use current time as seed for random generator

	//initialise all client_socket[] to 0 and clear buffers
	for (int i = 0; i < max_clients; i++){
		client_socket[i] = 0;
		client_buffer[i].clear();
	}   
    	//create a master socket  
    	if( (master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0){perror("socket failed");exit(EXIT_FAILURE);}
        //type of socket created  
    	address.sin_family = AF_INET;   
    	address.sin_addr.s_addr = INADDR_ANY;   
    	address.sin_port = htons( PORT );   
    	//bind the socket to localhost port 8888  
    	if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0){ 
        	perror("bind failed");   
	        exit(EXIT_FAILURE);   
    	}   
    	printf("Listener on port %d \r\n", PORT);   
    	//try to specify maximum of 3 pending connections for the master socket  
    	if (listen(master_socket, 3) < 0){perror("listen");exit(EXIT_FAILURE);}   
         
	//accept the incoming connection  
	addrlen = sizeof(address);   
    	puts("Waiting for connections ...");   


	//Initialise CCD and setup defaults
		if (CameraSelect (argc, argv) < 0) { //no hardware, screw it
			msg="ER CCD_SEL";
			perror(msg.c_str());
   			return -1;
   		}
		char* rrr = (char*)"/usr/local/etc/andor";
		error=Initialize(rrr);
		sleep(1);
		InitCamActions(); //all actions to get/set camera parameters
		if(error!=DRV_SUCCESS){
   			msg="ER "+std::to_string(error);
			perror(msg.c_str());
			return 1;
		}

   		specrographError = ATSpectrographInitialize("/usr/local/etc/andor");
   		if(specrographError!=ATSPECTROGRAPH_SUCCESS){perror("ER SPECTRO_INIT");
        		ShutDown();
        		return(1);
   		}

		int nodevices(0);
   		ATSpectrographGetNumberDevices(&nodevices);
   		if(nodevices < 1) { //no spectrograph, quit
	 		msg= "ER SPECTRO_DET";
	 		perror(msg.c_str());
         		ATSpectrographClose();
         		ShutDown();
         		return(1);
   		}
		InitSpectroActions(); //all actions to get/set spectrograph parameters
	quit = false;
	
	do{	//main loop
		val="-1";msg="nada";cmd="";cmda="nada";cmdb="nada";num=-1.00001;numi=-1;errsyntax=false;
		error=DRV_SUCCESS;
		specrographError=ATSPECTROGRAPH_SUCCESS;

		sockHandler();

		cmdavail=false;
		if (cmd.length() >= 3){
			cmda = cmd.substr (0,3);
			if (cmd.length() > 3) {cmdb = cmd.substr (3);}
		}
		if (cmdb.compare("nada") != 0) {
			num = std::atof(cmdb.c_str()); //suplied number arg converted to int
			numi = std::atoi(cmdb.c_str()); //converted to float
		}
				
		decideCase();//the main camera action happens inside this


		if ((error==DRV_SUCCESS) && (specrographError==ATSPECTROGRAPH_SUCCESS)){
			msg="OK "+cmda+" "+val+"\r\n";
		}else{
			msg="ER "+cmda+" "+val+" "+std::to_string(error)+" "+std::to_string(specrographError)+"\r\n";
		}
		if (errsyntax){
			msg="ER SYN 0\r\n";
		}
		//write to all avail sockets	
		
        	for (int i = 0; i < max_clients; i++){      
			sd = client_socket[i];
			ioctl(sd, SIOCOUTQ, &pending); 
			if (pending == 0){send(sd , msg.c_str() , strlen(msg.c_str()) , 0 );}
              	}

	}while(!quit);

	delete[] datas;
	ATSpectrographClose();
	ShutDown();
	
	puts("Ended gracefully");
	close(master_socket);
	return(0);

}

int CameraSelect (int iNumArgs, char* szArgList[])
{
	if (iNumArgs == 2) {
    		at_32 lNumCameras;
		GetAvailableCameras(&lNumCameras);
		int iSelectedCamera = atoi(szArgList[1]);
		if (iSelectedCamera < lNumCameras && iSelectedCamera >= 0) {
			at_32 lCameraHandle;
			GetCameraHandle(iSelectedCamera, &lCameraHandle);
			SetCurrentCamera(lCameraHandle);
			return iSelectedCamera;
		}else{return -1;}
  	}
  	return 0;
}

long GetDataSize()
{
   	//Gets the size of the array needed to store the data
	long imagesize;
	switch(ReadMode){
	case 0: imagesize=width; break;
	case 2: imagesize=width*numTracks; break;
	case 4: imagesize=(subImage.right-subImage.left+1)*(subImage.bottom-subImage.top+1)/(hbin*vbin);
	break;
	default: imagesize=width;
	}
	if(AcqMode==3) {imagesize=imagesize*numKins;}
	printf("Datasize: %ld \r\n",imagesize);
	return imagesize;
}

std::string GetAllStats()
{
	GetTemperature(&temp);
	GetAcquisitionTimings(&ext, &acc, &kin);
	GetReadOutTime(&rdout);
	GetKeepCleanTime(&clnout);
	std::string s="0 ";
	s+="GAM "+std::to_string(AcqMode)+' ';		//stat:Acquisition mode
	s+="GRM "+std::to_string(ReadMode)+' ';	//stat:Read mode
	s+="GET "+std::to_string(ext)+' ';	//stat:exposition time
	s+="GAT "+std::to_string(acc)+' ';	//stat:exposition time
	s+="GAN "+std::to_string(numAccs)+' ';		//stat:number of accums
	s+="GKT "+std::to_string(kin)+' ';	//stat:kinetic cycle time
	s+="GRT "+std::to_string(rdout)+' ';	//stat:readout time
	s+="GCT "+std::to_string(clnout)+' ';	//stat:get clean time
	s+="GKN "+std::to_string(numKins)+' ';		//stat:kinetic series number
	s+="GGA "+std::to_string(Gain)+' ';		//stat:detector gain
	s+="GHS "+std::to_string(hspeed)+' ';		//stat:horizontal speed index
	s+="GTS "+std::to_string(settemp)+' ';		//stat:preset temp
	s+="GTA "+std::to_string(temp)+' ';		//stat:actual temp
	s+="GGR "+std::to_string(igrat)+' ';		//stat:current grating
	s+="GWL "+std::to_string(wavel)+' ';		//stat:current WL
	s+="GCL "+std::to_string(calibrationValues[0])+' ';		//stat:current min vis WL
	s+="GCH "+std::to_string(calibrationValues[1023])+' ';		//stat:current max vis WL
	s+="GMN "+std::to_string(Min)+' ';		//stat:current grating WL min
	s+="GMX "+std::to_string(Max)+' ';		//stat:current grating WL max
	s+="GDO "+std::to_string(doffs)+' ';		//stat:detector offset
	s+="GGO "+std::to_string(groffs)+' ';		//stat:grating offset
	s+="GFL "+std::to_string(fmpos)+' ';		//stat:focus mirror position
	s+="GOP "+std::to_string(fport)+' ';		//stat:output path
	s+="GIP "+std::to_string(ifport)+' ';		//stat:input path
	s+="GFM "+std::to_string(filterMode)+' ';	//stat:filter mode
	s+="GVS "+std::to_string(vspeed)+' ';	//stat:vertical freq
	s+="GBC "+std::to_string(baseline)+' ';	//stat:baseline clamp
	s+="GT0 "+std::to_string(randomTracks[0])+' ';	//stat:randomtracks
	s+="GT1 "+std::to_string(randomTracks[1])+' ';	//stat:randomtracks
	s+="GT2 "+std::to_string(randomTracks[2])+' ';	//stat:randomtracks
	s+="GT3 "+std::to_string(randomTracks[3])+' ';	//stat:randomtracks
	s+="GUP "+std::to_string(trackpos)+' ';	//stat:singletracks pos
	s+="GUW "+std::to_string(trackwid)+' ';	//stat:singletracks wid
	s+="GTN "+std::to_string(numTracks);		//stat:number of Tracks
	return s;
} 

std::string GetAllStatsDelim()
{
//	GetTemperature(&temp);
//	GetAcquisitionTimings(&ext, &acc, &kin);
	std::string s="";
	s+="GAM "+std::to_string(AcqMode)+"\r\n";		//stat:Acquisition mode
	s+="GRM "+std::to_string(ReadMode)+"\r\n";	//stat:Read mode
	s+="GET "+std::to_string(ext)+"\r\n";	//stat:exposition time
	s+="GAT "+std::to_string(acc)+"\r\n";	//stat:exposition time
	s+="GAN "+std::to_string(numAccs)+"\r\n";		//stat:number of accums
	s+="GKT "+std::to_string(kin)+"\r\n";	//stat:kinetic cycle time
	s+="GRT "+std::to_string(rdout)+"\r\n";	//stat:readout time
	s+="GCT "+std::to_string(clnout)+"\r\n";	//stat:cleanout time
	s+="GKN "+std::to_string(numKins)+"\r\n";		//stat:kinetic series number
	s+="GGA "+std::to_string(Gain)+"\r\n";		//stat:detector gain
	s+="GHS "+std::to_string(hspeed)+"\r\n";		//stat:horizontal speed index
	s+="GTS "+std::to_string(settemp)+"\r\n";		//stat:preset temp
	s+="GTA "+std::to_string(temp)+"\r\n";		//stat:actual temp
	s+="GGR "+std::to_string(igrat)+"\r\n";		//stat:current grating
	s+="GWL "+std::to_string(wavel)+"\r\n";		//stat:current WL
	s+="GCL "+std::to_string(calibrationValues[0])+"\r\n";		//stat:current min vis WL
	s+="GCH "+std::to_string(calibrationValues[1023])+"\r\n";		//stat:current max vis WL
	s+="GMN "+std::to_string(Min)+"\r\n";		//stat:current grating WL min
	s+="GMX "+std::to_string(Max)+"\r\n";		//stat:current grating WL max
	s+="GDO "+std::to_string(doffs)+"\r\n";		//stat:detector offset
	s+="GGO "+std::to_string(groffs)+"\r\n";		//stat:grating offset
	s+="GFL "+std::to_string(fmpos)+"\r\n";		//stat:focus mirror position
	s+="GOP "+std::to_string(fport)+"\r\n";		//stat:output path
	s+="GIP "+std::to_string(ifport)+"\r\n";		//stat:input path
	s+="GFM "+std::to_string(filterMode)+"\r\n";	//stat:filter mode
	s+="GVS "+std::to_string(vspeed)+"\r\n";	//stat:vertical freq
	s+="GBC "+std::to_string(baseline);+"\r\n";	//stat:baseline clamp
	s+="GT0 "+std::to_string(randomTracks[0])+"\r\n";	//stat:randomtracks
	s+="GT1 "+std::to_string(randomTracks[1])+"\r\n";	//stat:randomtracks
	s+="GT2 "+std::to_string(randomTracks[2])+"\r\n";	//stat:randomtracks
	s+="GT3 "+std::to_string(randomTracks[3])+"\r\n";	//stat:randomtracks
	s+="GUP "+std::to_string(trackpos)+"\r\n";	//stat:singletracks pos
	s+="GUW "+std::to_string(trackwid)+"\r\n";	//stat:singletracks wid
	s+="GTN "+std::to_string(numTracks);		//stat:number of Tracks
	
	return s;
}

std::string dtToStr() //function to serialize calibration and datas arrays
{
	std::ostringstream oss;
	for(int i=0;i<1024;i++){
		oss << calibrationValues[i] << " ";
	}
	if (datas && datasize > 0) {
		for(int i=0;i<(datasize-1);i++){
			oss << static_cast<int>(datas[i]) << " ";
		}
		oss << static_cast<int>(datas[datasize-1]);
	}
	return oss.str();
}



void decideCase()
{
	if (cmda == "AQD" && AcqMode!=3){	//single shot acquisition, send data only via socket, not in kin series!
		//long datasize;
       		delete[] datas; datas=NULL;
		datasize=GetDataSize();
		//allocate memory to store the acquisition
//                if (wavel>300 && !sim){ATSpectrographGetCalibration(0, calibrationValues, 1024);}else{for(int i=0;i<1024;i++){calibrationValues[i]=i;}}
			error = StartAcquisition();
       			if(error==DRV_SUCCESS){

       				at_32 acm, kim;
         			int stat;
				//GetStatus(&stat);
				//for single exposure mode
				//if (AcqMode==1){WaitForAcquisition();}
				long lastSecondTime = getCurrentTimeMillis();
				if (AcqMode==1){
					
			        	do{
						long currentTime = getCurrentTimeMillis();
						GetStatus(&stat);
						usleep(1000);
						if (currentTime - lastSecondTime >= stopdelay) {
							sockHandlerNoBlock();
            						lastSecondTime = currentTime;  // update the last second time
       						} 
       		
					}while(stat==DRV_ACQUIRING && !cmdavail);

					if(cmdavail){
						AbortAcquisition();
						cmda="STP";val="0";printf("User interrupt \r\n");
					}
				}
	


				//for accumulation/kinetic mode
				if (AcqMode==2 || AcqMode==3){
			        	do{
						unsigned int ret = WaitForAcquisition();
       						if(ret==DRV_NO_NEW_DATA) {AbortAcquisition(); break;}
       						GetAcquisitionProgress(&acm, &kim);
       						GetStatus(&stat);
       					}while(stat==DRV_ACQUIRING);
				}
				//acquiring data
				
				
       				datas = new at_32[datasize];
				error = GetAcquiredData(datas,datasize);
				if(error==DRV_SUCCESS){
					if (ReadMode!=4){
						//fout.open("/run/shm/andor.dat", ios::out);
              					//for(int i=0;i<datasize;i++){fout << calibrationValues[i] << " " << datas[i] << endl;}
						//fout << endl;
						//fout << GetAllStatsDelim();
               					//fout.close();
						val=std::to_string(datasize) + " " + dtToStr();
					}else{
						SaveAsBmp("/run/shm/andor.bmp", "./GREY.PAL", 0, 0);
						fout.open("/run/shm/andor.cal", ios::out); //store calibration
						for(int i=0;i<1024;i++){fout << calibrationValues[i] << endl;}
				             	fout.close();
	
						val="0";
					}
					
			//		char* qqq = (char*)"/run/shm/andor.sif";
			//		SaveAsSif(qqq);
				}
			}

	} else if (cmda == "AQR"){	//single shot acquisition, saves data to file
		//long datasize;
       		delete[] datas; datas=NULL;
		datasize=GetDataSize();
		//allocate memory to store the acquisition
		
//printf("Allocating memory for the buffer.. \r\n");	
//     		datas = new at_32[datasize];
//                if (wavel>300 && !sim){ATSpectrographGetCalibration(0, calibrationValues, 1024);}else{for(int i=0;i<1024;i++){calibrationValues[i]=i;}}
			printf("Starting Acquisition.. \r\n");	
			error = StartAcquisition();

			printf("Acquisition started.. \r\n");	
			if(error==DRV_SUCCESS){
       				at_32 acm, kim;
         			int stat;
				//GetStatus(&stat);
				//for single exposure mode
				//if (AcqMode==1){WaitForAcquisition();}

				long lastSecondTime = getCurrentTimeMillis();
				if (AcqMode==1){
					
			        	do{
						long currentTime = getCurrentTimeMillis();
						GetStatus(&stat);
						usleep(1000);
						if (currentTime - lastSecondTime >= stopdelay) {
							sockHandlerNoBlock();
            						lastSecondTime = currentTime;  // update the last second time
       						} 
       		
					}while(stat==DRV_ACQUIRING && !cmdavail);

					if(cmdavail){
						AbortAcquisition();
						cmda="STP";val="0";printf("User interrupt \r\n");
					}
				}
	
				//for accumulation/kinetic mode
				if (AcqMode==2 || AcqMode==3){

					printf("Acquisition progress: \r\n");

					cmdavail=false;

			        	do{
						unsigned int ret = WaitForAcquisition();
       						if(ret==DRV_NO_NEW_DATA) {AbortAcquisition(); break;}
       						//if(ret==DRV_NO_NEW_DATA) {break;}
       						GetAcquisitionProgress(&acm, &kim);
						printf("Accums: %d  Kins: %d \r\n",acm,kim);
       						GetStatus(&stat);
						sockHandlerNoBlock();
       					}while(stat==DRV_ACQUIRING && !cmdavail);

					if(cmdavail){
						AbortAcquisition();
						cmda="STP";val="0";printf("User interrupt \r\n");
					}
				}

				printf("Acquisition done \r\n");
				//acquiring data

				printf("Allocating memory for the buffer.. \r\n");	
       				datas = new at_32[datasize];
				error = GetAcquiredData(datas,datasize);

				printf("Attempt to get data done ");
				if(error==DRV_SUCCESS){

					printf("..data ok, writing \r\n");
					if (ReadMode==4){
						SaveAsBmp("/run/shm/andor.bmp", "./GREY.PAL", 0, 0);
						fout.open("/run/shm/andor.cal", ios::out); //store calibration
						for(int i=0;i<1024;i++){fout << calibrationValues[i] << endl;}
				             	fout.close();
						val="0";
						printf("All [images] written \r\n");
					}
					if (AcqMode==3){
						fout.open("/run/shm/andor.cal", ios::out); //store calibration
						for(int i=0;i<1024;i++){fout << calibrationValues[i] << endl;}
						fout << endl;
						fout << GetAllStatsDelim();
               					fout.close();
						fout.open("/run/shm/andor.dat", ios::out);
						for(int i=0;i<datasize;i++){fout << datas[i] << endl;}
               					fout.close();
						val="0";//std::to_string(datasize) + " " + dtToStr();
						printf("All [kin. series] written \r\n");
					
					} else {
						fout.open("/run/shm/andor.dat", ios::out);
               					for(int i=0;i<datasize;i++){fout << calibrationValues[i] << " " << datas[i] << endl;}
						fout << endl;
						fout << GetAllStatsDelim();
               					fout.close();
						val="0";//std::to_string(datasize) + " " + dtToStr();
						printf("All [data] written \r\n");
					}
				}
					
			//		char* qqq = (char*)"/run/shm/andor.sif";
		
			//		SaveAsSif(qqq);
			}
	

				
		

	} else if (cmda == "AQC" && AcqMode!=3){ //continuous acquisition (FVB or Image mode, also Accumulation Mode supported, but NOT Kinetic!)
		//long datasize;
 		datasize = GetDataSize();
		int stat;
       		delete[] datas; datas=NULL;
		int stop=0;
//                if (wavel>300){ATSpectrographGetCalibration(0, calibrationValues, 1024);}else{for(int i=0;i<1024;i++){calibrationValues[i]=i;}}
		fout.open("/run/shm/andor.cal", ios::out); //store calibration
		for(int i=0;i<1024;i++){
               		fout << calibrationValues[i] << endl;
		}
               	fout.close();
		error = StartAcquisition();
       		if(error==DRV_SUCCESS){
			do{
				//allocate memory to store the acquisition
       					datas = new at_32[datasize];
      					//GetStatus(&stat);
					//wait for end of acq or interrupt
					cmdavail=false;
					long lastSecondTime = getCurrentTimeMillis();
					
			        	do{
						long currentTime = getCurrentTimeMillis();
						GetStatus(&stat);
						usleep(1000);
						if (currentTime - lastSecondTime >= stopdelay) {
							sockHandlerNoBlock();
            						lastSecondTime = currentTime;  // update the last second time
       						} 
       		
					}while(stat==DRV_ACQUIRING && !cmdavail);

					if(cmdavail){
						AbortAcquisition();
						cmda="STP";val="0";printf("User interrupt \r\n");
						break;
					}
				
	




//					while(stat==DRV_ACQUIRING && !cmdavail){
//						WaitForAcquisition();
//						GetStatus(&stat);
//						sockHandlerNoBlock();
//					}
//					if(cmdavail){cmda="STP";val="0";break;}
				//}
	
				//if (!sim){
					//acquire the data
					error = GetAcquiredData(datas,datasize);
					if (error==DRV_SUCCESS && ReadMode==4){SaveAsBmp("/run/shm/andor.bmp", "./GREY.PAL", 0, 0);}

					if (error==DRV_SUCCESS) error = StartAcquisition();
				//}else{
					error=DRV_SUCCESS;
				//}
				if(error==DRV_SUCCESS){
					if (ReadMode!=4){
						val=std::to_string(datasize) + " " + dtToStr();
					}else{
						val="0";
					}
					msg="OK "+cmda+" "+val+"\r\n";
				//write to all avail sockets	
				
		        		for (int i = 0; i < max_clients; i++){      
						sd = client_socket[i];
						ioctl(sd, SIOCOUTQ, &pending); 
						if (pending == 0){send(sd , msg.c_str() , strlen(msg.c_str()) , 0 );}
			              	}
				}
				delete[] datas; datas=NULL;
			}while(true);
    			//if (!sim){AbortAcquisition();}else{error=DRV_SUCCESS;}
			error=DRV_SUCCESS;
		}

	} else if (cmda == "AQP"){      //preparation cmd for kinetic series etc.
	        error = FreeInternalMemory();  
		if(error==DRV_SUCCESS){error = PrepareAcquisition();}


                if(error==DRV_SUCCESS){
                        val="0";
                        printf("Camera prepared");
                }

	} else if ( cmda == "BRK"){ 	//quit
		quit=true;
	} else if (cmda == "GTA"){	//Get Temperature
		GetTemperature(&temp);
		val=std::to_string(temp);
	} else if (cmda == "GTS"){	//Get Set Temperature
		val=std::to_string(settemp);
	} else if (cmda == "GAN"){	//Get Accumulation number
		val=std::to_string(numAccs);
	} else if (cmda == "GKN"){	//Get kinetic s. number
		val=std::to_string(numKins);
	} else if (cmda == "GAM"){	//Get acquisition mode
		val=std::to_string(AcqMode);
	} else if (cmda == "GRM"){	//Get read mode
		val=std::to_string(ReadMode);
	} else if (cmda == "GET"){	//Get exposition time
		GetAcquisitionTimings(&ext, &acc, &kin);
		GetReadOutTime(&rdout);
		GetKeepCleanTime(&clnout);
		val=std::to_string(ext);
	} else if (cmda == "GAT"){	//Get accumulation time (corrected for cycle overhead)
		GetAcquisitionTimings(&ext, &acc, &kin);
		GetReadOutTime(&rdout);
		val=std::to_string(acc);
	} else if (cmda == "GHS"){	//Get horizontal speed
//		error=GetHSSpeed(0, 0, hspeed, &speed);
		val=std::to_string(hspeed);
	} else if (cmda == "SHS"){	//Set horizontal speed (index 0..2)
		error=SetHSSpeed(0, numi);
		if(error==DRV_SUCCESS){hspeed=numi;GetAcquisitionTimings(&ext, &acc, &kin);GetReadOutTime(&rdout);GetKeepCleanTime(&clnout);}
		val=std::to_string(hspeed)+" GET "+std::to_string(ext)+" GAT "+std::to_string(acc)+" GKT "+std::to_string(kin);
	} else if (cmda == "SVS"){	//Set vertical speed (index 0..?) manual says: careful!
		error=SetVSSpeed(numi);
		if(error==DRV_SUCCESS){vspeed=numi;GetAcquisitionTimings(&ext, &acc, &kin);GetReadOutTime(&rdout);GetKeepCleanTime(&clnout);}
		val=std::to_string(vspeed)+" GET "+std::to_string(ext)+" GAT "+std::to_string(acc)+" GKT "+std::to_string(kin);
	} else if (cmda == "GKT"){	//Get kinetic cycle time
		GetAcquisitionTimings(&ext, &acc, &kin);
		GetReadOutTime(&rdout);
		val=std::to_string(kin);
	} else if (cmda == "GRT"){	//Get readout time
		GetReadOutTime(&rdout);
		val=std::to_string(rdout);
	} else if (cmda == "GCT"){	//Get clean time
		GetKeepCleanTime(&clnout);
		val=std::to_string(clnout);
	} else if (cmda == "GST"){	//Get ALL Stats
		val=GetAllStats();//DisplayAcquireInfo();
	} else if (cmda == "GGR"){	//get current grating
                ATSpectrographGetGrating(0, &igrat);
		val=std::to_string(igrat);
	} else if (cmda == "GFM"){	//get current filter mode
		GetFilterMode(&filterMode);
		val=std::to_string(filterMode);
//	} else if (cmda == "SFM"){	//set current filter mode - BUG in ANDOR, do not USE!!
//		error=SetFilterMode(numi);
//		if(error==DRV_SUCCESS) GetFilterMode(&filterMode);
//		val=std::to_string(filterMode);
	} else if (cmda == "SAN"){	//number of accums
		error=SetNumberAccumulations(numi);
		if(error==DRV_SUCCESS){numAccs=numi;}
		val=std::to_string(numAccs);
	} else if (cmda == "SKN"){	//number in kinetic series
		error=SetNumberKinetics(numi);
		if(error==DRV_SUCCESS){numKins=numi;}
		val=std::to_string(numKins);
	} else if (cmda == "SET"){	//exposure time
		error=SetExposureTime(num);
		GetAcquisitionTimings(&ext, &acc, &kin);
		GetReadOutTime(&rdout);
		GetKeepCleanTime(&clnout);
//		if (sim && num > 0.01f){ext=num;acc=num+0.05;}
		val=std::to_string(ext)+" GAT "+std::to_string(acc)+" GKT "+std::to_string(kin);
	} else if (cmda == "SKT"){	//kinetic exposure time
		error=SetKineticCycleTime(num);
		GetAcquisitionTimings(&ext, &acc, &kin);
		GetReadOutTime(&rdout);
		GetKeepCleanTime(&clnout);
		val=std::to_string(kin)+" GAT "+std::to_string(acc)+" GET "+std::to_string(ext);
//		if(error==DRV_SUCCESS || sim){msg=cmda+" "+std::to_string(num)+" OK";expoTime=num;}else{msg="ER " + std::to_string(error);}
//	} else if (cmda == "SAT"){	//accumulation exposure time
//		error=SetAccumulationCycleTime(num);
//		if(error==DRV_SUCCESS || sim){msg=cmda+" "+std::to_string(num)+" OK";accTime=num;}else{msg="ER " + std::to_string(error);}
	} else if (cmda == "SRM"){	//select FVB mode
		if (num == 0 || num==4 || num == 2 || num == 3){
			error=SetReadMode(num);
			if(error==DRV_SUCCESS){ReadMode=num;}
		}
		GetAcquisitionTimings(&ext, &acc, &kin);
		GetReadOutTime(&rdout);
		GetKeepCleanTime(&clnout);
		val=std::to_string(ReadMode)+" GET "+std::to_string(ext)+" GAT "+std::to_string(acc)+" GKT "+std::to_string(kin);
	} else if (cmda == "SAM"){	//select Single/Accumulation/Kinetic series Scan mode
		if (num == 1 || num==2 || num==3){
			error=SetAcquisitionMode(num);
			if(error==DRV_SUCCESS){AcqMode=num;}
		}
			GetAcquisitionTimings(&ext, &acc, &kin);
			GetReadOutTime(&rdout);
			GetKeepCleanTime(&clnout);
			val=std::to_string(AcqMode)+" GET "+std::to_string(ext)+" GAT "+std::to_string(acc)+" GKT "+std::to_string(kin);
	} else if (cmda == "SGA"){	//set gain
		error=SetPreAmpGain(numi);
		if(error==DRV_SUCCESS){Gain=numi;}
		val=std::to_string(Gain);
	} else if (cmda == "GGA"){	//get gain
		val=std::to_string(Gain);
	} else if (cmda == "GCL"){	//get calibration of wavelengths MIN
        	ATSpectrographGetCalibration(0, calibrationValues, 1024);
		val=std::to_string(calibrationValues[0]);
	} else if (cmda == "GCH"){	//get calibration of wavelengths MAX
              	ATSpectrographGetCalibration(0, calibrationValues, 1024);
		val=std::to_string(calibrationValues[1023]);
	} else if (cmda == "GWL"){	//get current center wavelength
                ATSpectrographGetWavelength(0,&wavel);
		val=std::to_string(wavel);
//	} else if (cmda == "GWR"){	//get wavelength range for current grating
//              ATSpectrographGetGrating(0, &igrat);
//              ATSpectrographGetWavelengthLimits(0, igrat, &Min, &Max);
//              msg=std::to_string(Min)+"-"+std::to_string(Max);
	} else if (cmda == "STS"){	//set temperature
		error=SetTemperature(num);
		if(error==DRV_SUCCESS){settemp=num;}
		val=std::to_string(settemp);
	} else if (cmda == "SWL"){	//set wavelength
		specrographError=ATSpectrographSetWavelength(0, num);
                if(specrographError==ATSPECTROGRAPH_SUCCESS){
			wavel=num;
	                if (wavel>300){ATSpectrographGetCalibration(0, calibrationValues, 1024);}else{for(int i=0;i<1024;i++){calibrationValues[i]=i;}}
		}
		val=std::to_string(wavel)+" GCL "+std::to_string(calibrationValues[0])+" GCH "+std::to_string(calibrationValues[1023]);
//	} else if (cmda == "SZR"){	//go to zero order
//		specrographError=ATSpectrographGotoZeroOrder(0);
//              if(specrographError==ATSPECTROGRAPH_SUCCESS){msg= cmda+" OK";}
//		else{msg="ER "+std::to_string(specrographError);}
	} else if (cmda == "SDO"){	//set detector offset
		specrographError=ATSpectrographSetDetectorOffset(0,SIDE,fport,numi);
		if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetDetectorOffset(0,SIDE,fport,&doffs);}
                val=std::to_string(doffs);
	} else if (cmda == "GDO"){	//get detector offset
		specrographError=ATSpectrographGetDetectorOffset(0,SIDE,fport,&doffs);
                val=std::to_string(doffs);
	} else if (cmda == "SGO"){	//set grating offset
		specrographError=ATSpectrographSetGratingOffset(0,igrat,numi);
		if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetGratingOffset(0,igrat,&groffs);}
                val=std::to_string(groffs);
	} else if (cmda == "GGO"){	//get grating offset
		specrographError=ATSpectrographGetGratingOffset(0,igrat,&groffs);
                val=std::to_string(groffs);
	} else if (cmda == "SFL"){	//set focus mirror
		specrographError=ATSpectrographSetFocusMirror(0,numi);
		if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetFocusMirror(0,&fmpos);}
                val=std::to_string(fmpos);
	} else if (cmda == "GFL"){	//get focus mirror pos.
		specrographError=ATSpectrographGetFocusMirror(0,&fmpos);
                val=std::to_string(fmpos);
	} else if (cmda == "SOP"){	//set flipper mirror (output path) to direct output (CCD)
		if (num==0){specrographError=ATSpectrographSetFlipperMirror(0,OUTPUT_FLIPPER,DIRECT);}
		if (num==1){specrographError=ATSpectrographSetFlipperMirror(0,OUTPUT_FLIPPER,SIDE);}
                if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetFlipperMirror(0,OUTPUT_FLIPPER,&fport);}
                val=std::to_string(fport);
	} else if (cmda == "GOP"){	//get flipper mirror output path
		specrographError=ATSpectrographGetFlipperMirror(0,OUTPUT_FLIPPER,&fport);
                val=std::to_string(fport);
	} else if (cmda == "SIP"){	//set flipper mirror (input path) to side output (APD)
		if (num==0){specrographError=ATSpectrographSetFlipperMirror(0,INPUT_FLIPPER,DIRECT);}
		if (num==1){specrographError=ATSpectrographSetFlipperMirror(0,INPUT_FLIPPER,SIDE);}
                if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetFlipperMirror(0,INPUT_FLIPPER,&ifport);}
                val=std::to_string(ifport);
	} else if (cmda == "GIP"){	//get flipper mirror input path
		specrographError=ATSpectrographGetFlipperMirror(0,INPUT_FLIPPER,&ifport);
                val=std::to_string(ifport);

	} else if (cmda == "GTN"){	//get number of random tracks
                val=std::to_string(numTracks);
	} else if (cmda == "STN"){	//set number of random tracks
		error=SetRandomTracks(numi, randomTracks);
		if(error==DRV_SUCCESS){numTracks=numi;}
                val=std::to_string(numTracks);
	} else if (cmda == "ST0"){	//set pos of random tracks
		bknumi=randomTracks[0];
		randomTracks[0]=numi;
		error=SetRandomTracks(numTracks, randomTracks);
		if(error!=DRV_SUCCESS){randomTracks[0]=bknumi;}
                val=std::to_string(randomTracks[0]);
	} else if (cmda == "ST1"){	//set pos of random tracks
		bknumi=randomTracks[1];
		randomTracks[1]=numi;
		error=SetRandomTracks(numTracks, randomTracks);
		if(error!=DRV_SUCCESS){randomTracks[1]=bknumi;}
                val=std::to_string(randomTracks[1]);
	} else if (cmda == "ST2"){	//set pos of random tracks
		bknumi=randomTracks[2];
		randomTracks[2]=numi;
		error=SetRandomTracks(numTracks, randomTracks);
		if(error!=DRV_SUCCESS){randomTracks[2]=bknumi;}
                val=std::to_string(randomTracks[2]);
	} else if (cmda == "ST3"){	//set pos of random tracks
		bknumi=randomTracks[3];
		randomTracks[3]=numi;
		error=SetRandomTracks(numTracks, randomTracks);
		if(error!=DRV_SUCCESS){randomTracks[3]=bknumi;}
                val=std::to_string(randomTracks[3]);
	} else if (cmda == "SUP"){	//set singletrack pos
		error=SetSingleTrack(numi, trackwid);
		if(error==DRV_SUCCESS){trackpos=numi;}
                val=std::to_string(trackpos);
	} else if (cmda == "SUW"){	//set singletrack wid
		error=SetSingleTrack(trackpos, numi);
		if(error==DRV_SUCCESS){trackwid=numi;}
                val=std::to_string(trackwid);
	} else if (cmda == "GT0"){	//get random tracks pos
                val=std::to_string(randomTracks[0]);
	} else if (cmda == "GT1"){	//get random tracks pos
                val=std::to_string(randomTracks[1]);
	} else if (cmda == "GT2"){	//get random tracks pos
                val=std::to_string(randomTracks[2]);
	} else if (cmda == "GT3"){	//get random tracks pos
                val=std::to_string(randomTracks[3]); 
	} else if (cmda == "GT3"){	//get random tracks pos
                val=std::to_string(randomTracks[3]);

	} else if (cmda == "GUP"){	//get single track pos
                val=std::to_string(trackpos);
	} else if (cmda == "GUW"){	//get single track wid
                val=std::to_string(trackwid);



	} else if (cmda == "SGR"){	//set grating
                specrographError=ATSpectrographSetGrating(0, numi);
                if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetGrating(0, &igrat);}
                if(specrographError==ATSPECTROGRAPH_SUCCESS){ATSpectrographGetCalibration(0, calibrationValues, 1024);ATSpectrographGetWavelength(0,&wavel);}

		if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetDetectorOffset(0,SIDE,fport,&doffs);}
		if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetGratingOffset(0,igrat,&groffs);}
		if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetFocusMirror(0,&fmpos);}
                val=std::to_string(igrat)+" GWL "+std::to_string(wavel)+" GCL "+std::to_string(calibrationValues[0])+" GCH "+std::to_string(calibrationValues[1023])+ " GDO " + std::to_string(doffs)+ " GGO " + std::to_string(groffs)+ " GFL " + std::to_string(fmpos);
	



	}else{errsyntax=true;}

}

void InitCamActions()
{
	if(error==DRV_SUCCESS) error=GetDetector(&width, &height);
	if(error==DRV_SUCCESS) error=GetPixelSize(&xSize, &ySize);
//	if(error==DRV_SUCCESS) error=SetShutter(1,0,50,50);
	if(error==DRV_SUCCESS) error=SetTriggerMode(trigMode);
	if(error==DRV_SUCCESS) error=SetAcquisitionMode(AcqMode);
	if(error==DRV_SUCCESS) error=SetReadMode(ReadMode);
	if(error==DRV_SUCCESS) error=SetExposureTime(ext);
	if(error==DRV_SUCCESS) error=GetBaselineClamp(&baseline);
//	if(error==DRV_SUCCESS) error=SetFilterMode(filterMode);
	if(error==DRV_SUCCESS) error=GetFilterMode(&filterMode);
	// if(error==DRV_SUCCESS) error=SetAccumulationCycleTime(acc);
	if(error==DRV_SUCCESS) error=SetNumberAccumulations(numAccs);
	if(error==DRV_SUCCESS) error=SetPreAmpGain(Gain); //set preamp gain
	//   if(error==DRV_SUCCESS) error=SetKineticCycleTime(kin);
	if(error==DRV_SUCCESS) error=SetNumberKinetics(numKins);
	if(error==DRV_SUCCESS) error=GetAcquisitionTimings(&ext, &acc, &kin);
	if(error==DRV_SUCCESS) error=GetReadOutTime(&rdout);
	if(error==DRV_SUCCESS) error=GetKeepCleanTime(&clnout);
//	if(error==DRV_SUCCESS) error=SetMultiTrack(numTracks,trackHeight,trackOffset, &trackBottom, &trackGap);
	   randomTracks = new int[height*2];
	   randomTracks[0]=1; randomTracks[1]=2;
	   randomTracks[2]=3; randomTracks[3]=4;
	   numTracks=1;
	//   if(error==DRV_SUCCESS) SetReadMode(2);
	if(error==DRV_SUCCESS) error=SetRandomTracks(numTracks, randomTracks);
		trackpos = 128;
		trackwid = 8;

	if(error==DRV_SUCCESS) error=SetSingleTrack(trackpos, trackwid);
	if(error==DRV_SUCCESS) SetTemperature(settemp);
	if(error==DRV_SUCCESS) CoolerON();
	subImage.left = 1; subImage.right = width; subImage.top = 1; subImage.bottom = height;
	if(error==DRV_SUCCESS) error=SetImage(hbin,vbin,subImage.left, subImage.right, subImage.top, subImage.bottom);
	if(error==DRV_SUCCESS) error=SetHSSpeed(0,hspeed);
	
}

void InitSpectroActions()
{
        ATSpectrographSetNumberPixels(0, width);
       	ATSpectrographSetPixelWidth(0, xSize);
	ATSpectrographSetWavelength(0, wavel);
     	ATSpectrographGetGrating(0, &igrat);
        ATSpectrographGetWavelengthLimits(0, igrat, &Min, &Max);
	ATSpectrographGetDetectorOffset(0,SIDE,DIRECT,&doffs);
	ATSpectrographGetGratingOffset(0,igrat,&groffs);
	ATSpectrographGetFocusMirror(0,&fmpos);
	ATSpectrographGetFlipperMirror(0,OUTPUT_FLIPPER,&fport);
	ATSpectrographGetFlipperMirror(0,INPUT_FLIPPER,&ifport);

	if (wavel>300){ATSpectrographGetCalibration(0, calibrationValues, 1024);}else{for(int i=0;i<1024;i++){calibrationValues[i]=i;}}

	//ATSpectrographGetCalibration(0, calibrationValues, 1024);
}




int sockHandler(){ //waits for activity and handles the sockets
	char* message = (char*)"OK CON \r\n";   
	do {
		//clear the socket set,add master socket  
		FD_ZERO(&readfds);
		FD_SET(master_socket, &readfds);   
		max_sd = master_socket;   
             
		//add child sockets to set  
		for (int i = 0 ; i < max_clients ; i++){   
			sd = client_socket[i];
			if(sd > 0){FD_SET( sd , &readfds);}
			if(sd > max_sd){max_sd = sd;}
		}

		//wait for an activity on one of the sockets , timeout is NULL
		activity = select( max_sd + 1 , &readfds , NULL , NULL , NULL);  
		if ((activity < 0) && (errno!=EINTR)){perror("select error");}   
             
		//If something happened on the master socket ,  
		//then its an incoming connection  
		if (FD_ISSET(master_socket, &readfds)){   
			if ((new_socket = accept(master_socket,  
					(struct sockaddr *)&address, (socklen_t*)&addrlen))<0){   
				perror("accept");   
				exit(EXIT_FAILURE);   
			}   
			printf("New connection , socket fd is %d , ip is : %s , port : %d \r\n" , new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   
			if( send(new_socket, message, strlen(message), 0) != (ssize_t)strlen(message) ){perror("send");}   
			puts("Welcome message sent successfully");   
			//add new socket to array of sockets  
			for (int i = 0; i < max_clients; i++){   
				if( client_socket[i] == 0 ){   
					client_socket[i] = new_socket;   
					client_buffer[i].clear(); // clear buffer for safety
					printf("Adding to list of sockets as %d \r\n" , i);   
					break;   
				}   
			}   
		}   

		//else its some IO operation on some other socket 
		for (int i = 0; i < max_clients; i++){      
			sd = client_socket[i];   
			if (sd > 0 && FD_ISSET( sd , &readfds)){   
				valread = recv( sd , buffer, 1024, 0);
				if (valread <= 0)   
				{   
					// Somebody disconnected or error occurred
					if (valread < 0) {
						perror("recv error");
					} else {
						getpeername(sd , (struct sockaddr*)&address , (socklen_t*)&addrlen);   
						printf("Host disconnected , ip %s , port %d \r\n" ,  
							inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   
					}
					close( sd );   
					client_socket[i] = 0;   
					client_buffer[i].clear();
				} else {   
					buffer[valread] = '\0';
					client_buffer[i] += buffer;
					
					// Parse line-by-line
					size_t newline_pos = client_buffer[i].find('\n');
					if (newline_pos == std::string::npos) {
						newline_pos = client_buffer[i].find('\r');
					}
					if (newline_pos != std::string::npos) {
						cmd = client_buffer[i].substr(0, newline_pos);
						client_buffer[i].erase(0, newline_pos + 1);
						
						// Trim trailing \r, \n and whitespace
						while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == '\n' || isspace(cmd.back()))) {
							cmd.pop_back();
						}
						// Trim leading whitespace
						size_t first = cmd.find_first_not_of(" \t\r\n");
						if (first != std::string::npos) {
							cmd = cmd.substr(first);
						} else {
							cmd.clear();
						}
						
						if (!cmd.empty()) {
							cmdavail = true;
							// Broadcast the command to everyone except the commander 
							std::string broadcast_msg = cmd + "\r\n";
							for (int j = 0; j < max_clients; j++){     
								int target_sd = client_socket[j];
								if (target_sd > 0 && i != j){
									send(target_sd , broadcast_msg.c_str() , broadcast_msg.length() , 0 );
								}
							}
							break; // exit client processing loop, cmd is ready
						}
					}
				}   
			}   
		}   
	} while(!cmdavail);
	return 0;
}

int sockHandlerNoBlock(){ //detects activity and handles the sockets without blocking
	bool jacks=false;	
	char* message = (char*)"CON\r\n";
	do {	
		jacks=false;
		//clear the socket set,add master socket  
		FD_ZERO(&readfds);
		FD_SET(master_socket, &readfds);   
		max_sd = master_socket;   
             
		//add child sockets to set  
		for (int i = 0 ; i < max_clients ; i++){   
			sd = client_socket[i];
			if(sd > 0){FD_SET( sd , &readfds);}
			if(sd > max_sd){max_sd = sd;}
		}  

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 0;
	        		
		//wait for activity with 0 timeout
		activity = select( max_sd + 1 , &readfds , NULL , NULL , &tv);  
		if ((activity < 0) && (errno!=EINTR)){perror("select error");}   
             
		//If something happened on the master socket ,  
		//then its an incoming connection  
		if (FD_ISSET(master_socket, &readfds)){   
			if ((new_socket = accept(master_socket,  
					(struct sockaddr *)&address, (socklen_t*)&addrlen))<0){   
				perror("accept");   
				exit(EXIT_FAILURE);   
			}   
			printf("New connection , socket fd is %d , ip is : %s , port : %d \r\n" , new_socket , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   
			if( send(new_socket, message, strlen(message), 0) != (ssize_t)strlen(message) ){perror("send");}   
			puts("Welcome message sent successfully");   
			jacks=true; 
			//add new socket to array of sockets 
			for (int i = 0; i < max_clients; i++){   
				if( client_socket[i] == 0 ){   
					client_socket[i] = new_socket;   
					client_buffer[i].clear();
					printf("Adding to list of sockets as %d \r\n" , i);   
					break;   
				}   
			}   
		}   

		//else its some IO operation on some other socket 
		for (int i = 0; i < max_clients; i++){      
			sd = client_socket[i];   
			if (sd > 0 && FD_ISSET( sd , &readfds)){   
				valread = recv( sd , buffer, 1024, 0);
				if (valread <= 0)   
				{   
					// Somebody disconnected or error occurred
					if (valread < 0) {
						perror("recv error");
					} else {
						getpeername(sd , (struct sockaddr*)&address , (socklen_t*)&addrlen);   
						printf("Host disconnected , ip %s , port %d \r\n" ,  
							inet_ntoa(address.sin_addr) , ntohs(address.sin_port));   
					}
					close( sd );   
					client_socket[i] = 0;
					client_buffer[i].clear();
					jacks=true;	
				} else {   
					buffer[valread] = '\0';
					client_buffer[i] += buffer;
					
					// Parse line-by-line
					size_t newline_pos = client_buffer[i].find('\n');
					if (newline_pos == std::string::npos) {
						newline_pos = client_buffer[i].find('\r');
					}
					if (newline_pos != std::string::npos) {
						cmd = client_buffer[i].substr(0, newline_pos);
						client_buffer[i].erase(0, newline_pos + 1);
						
						// Trim trailing \r, \n and whitespace
						while (!cmd.empty() && (cmd.back() == '\r' || cmd.back() == '\n' || isspace(cmd.back()))) {
							cmd.pop_back();
						}
						// Trim leading whitespace
						size_t first = cmd.find_first_not_of(" \t\r\n");
						if (first != std::string::npos) {
							cmd = cmd.substr(first);
						} else {
							cmd.clear();
						}
						
						if (!cmd.empty()) {
							if (cmd == " " || cmd == "STP") {
								cmdavail = true;
								break; // cmd is ready
							} else {
								// Not a stop command, reply busy and ignore
								std::string busy_msg = "BUSY\r\n";
								send(sd, busy_msg.c_str(), busy_msg.length(), 0);
							}
						}
					}
				}   
			}   
		}   
	} while(jacks && !cmdavail);
	return 0;
}

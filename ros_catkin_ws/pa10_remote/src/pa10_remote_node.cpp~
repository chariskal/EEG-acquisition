#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <pa10_remote/pa10remote.h>

#define PI acos(-1)
using namespace std;
using namespace Eigen;

double qdot[7];

int pa10CommInit(){

     int status;
     struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
     struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.

 
     memset(&host_info, 0, sizeof host_info);

     std::cout << "Setting up the structs..."  << std::endl;

     host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
     host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

     status = getaddrinfo("147.102.51.71", "4534", &host_info, &host_info_list);
     
     if (status != 0){  
         std::cout << "getaddrinfo error" << gai_strerror(status) ;
         exit(0);
     }

     std::cout << "Creating a socket..."  << std::endl;
     int socketfd ; // The socket descripter
     socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
 
     if (socketfd == -1){
         std::cout << "socket error\n" ;
         exit(0);
     }

     std::cout << "Connecting..."  << std::endl;
     status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);

     if (status == -1){
         std::cout << "connect error\n" ;
         exit(0);
      }

     std::cout << "Connection with PA 10 PC Established !!!";

     std::cout << "sending message..."  << std::endl;
     char *msg = "I040"; 
     int len;
     ssize_t bytes_sent;
     len = strlen(msg);

     bytes_sent = send(socketfd, msg, len, 0);

     return socketfd;
}

void getPA10data(int socketfd, double* qjoints){

     float pa10data[7];

     std::cout << "Waiting to receive data..."  << std::endl;
     ssize_t bytes_received2;
     char incomming_data_buffer2[1000];
     bytes_received2 = recv(socketfd,incomming_data_buffer2,1000, 0);

     if (bytes_received2 == 0){
         std::cout << "host shut down." << std::endl ;
         exit(0);
     }

     if (bytes_received2 == -1){
         std::cout << "receive error!" << std::endl ;
         exit(0);
     }
         


     std::cout << bytes_received2 << " bytes received :" << std::endl ;
     std::cout << incomming_data_buffer2 << std::endl;
     incomming_data_buffer2[bytes_received2 -2] = '\0';

     std::cout << "Bypassed Get Angles !!!" << "\n";

     char *mstring;
     mstring = (char *)malloc(20 * sizeof(char));

     char *mstart,*mend;

     if(incomming_data_buffer2[0]=='I')
         mstart = &(incomming_data_buffer2[7]);
     else
         mstart = &(incomming_data_buffer2[1]);
         

     for (int count=0; count<7; count++){
         mend = strchr(mstart,97);
         strncpy(mstring,mstart,(int)(mend - mstart));
         //cout << "OK" << "\n";
         strcat(mstring,"\0");
         pa10data[count]=atof(mstring);
         mstart=mend+1;
         }

     std::cout << "PA10: " << " q1: " << pa10data[0] << " q2: " << pa10data[1]
                           << " q3: " << pa10data[2] << " q4: " << pa10data[3]
                           << " q5: " << pa10data[4] << " q6: " << pa10data[5]
                           << " q7: " << pa10data[6] << "\n";

     for (int i = 0; i < 7; i++)
         qjoints[i] = pa10data[i];
}

void setPA10data(int socketfd, double* qdot){


     float DOF1 = qdot[0]*180/PI;
     float DOF2 = qdot[1]*180/PI;
     float DOF3 = qdot[2]*180/PI;
     float DOF4 = qdot[3]*180/PI;
     float DOF5 = qdot[4]*180/PI;
     float DOF6 = qdot[5]*180/PI;
     float DOF7 = qdot[6]*180/PI;


     std::string result;
     std::stringstream convert;
     convert<<"M"<<DOF1<<"a"<<DOF2<<"a"<<DOF3<<"a"<<DOF4<<"a"<<DOF5<<"a"<<DOF6<<"a"<<DOF7<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
     result=convert.str();

     //std::cout << "Result:" << result << std::endl;

     char *msg2 = new char[result.size()+1];
     msg2[result.size()]=0;
     memcpy(msg2,result.c_str(),result.size());
     int len2;
     ssize_t bytes_sent2;
     len2 = strlen(msg2);

     bytes_sent2 = send(socketfd, msg2, len2, 0);

     std::cout << "Send Message !!!" << "\n";
}

void updateQdot(const std_msgs::Float64MultiArrayConstPtr& msg)
{
     int i = 0;
     // print all the remaining numbers
     for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		qdot[i] = *it;
		i++;
	}
}


int main (int argc, char **argv){

    ros::init (argc, argv, "pa10_remote");
    ros::NodeHandle nh;
    ros::Publisher q_pub = nh.advertise<std_msgs::Float64MultiArray>("/pa10/joint_positions", 1);
    ros::Subscriber qdot_sub = nh.subscribe<std_msgs::Float64MultiArray>("/pa10/qdot_cmd", 1, updateQdot);

    ros::Rate loop_rate( 10.0 );

    double qjoints[7]; 
    for (int i = 0 ; i < 7; i++)
         qjoints[i] = 0.0;

    for (int i = 0 ; i < 7; i++)
         qdot[i] = 0.0;

    int socket_hd = pa10CommInit();
   
    while (ros::ok())
	{
          
	  double start =ros::Time::now().toSec();   
	  //Get Joint Positions 
          getPA10data(socket_hd, qjoints);


          std_msgs::MultiArrayDimension dim;
          dim.size = 7;
          dim.stride = 7;

          //Publish Joint positions
          std_msgs::Float64MultiArray pub_array;
          
          pub_array.layout.dim.push_back(dim);
          pub_array.layout.data_offset = 0;

          //for loop, pushing data in the size of the array
	  for (int i = 0; i < 7; i++){
	      pub_array.data.push_back(qjoints[i]);
	  }
          cout << qjoints << endl;
	  //Publish array
          q_pub.publish(pub_array);
 


          setPA10data(socket_hd, qdot);

           

          ros::spinOnce();
	  loop_rate.sleep();
          
          double end =ros::Time::now().toSec();
          printf("Loop dt:%lf\n", end-start);
	}
        
 
    return 0;

}

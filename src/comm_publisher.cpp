
#include <string>
#include <thread>

#include <sys/socket.h> /* socket specific definitions */
#include <arpa/inet.h> /* IP address conversion stuff */

#include <ros/ros.h>

#include "g2o/stuff/command_args.h"

#include "cg_mrslam/Ping.h"
#include "definitions.h"

using namespace g2o;
using namespace std;

class CommPublisher{
public: 
  CommPublisher(int idRobot_,
		int nRobots_,
		std::string baseAddr_){
    _idRobot = idRobot_;
    _nRobots = nRobots_;
    _baseAddr = baseAddr_;
  }
  
  void sendToThread(){
    while (ros::ok()){
      for (int r = 0; r < _nRobots; r++){
	if (r == _idRobot)
	  continue;

	std::string toAddr = _baseAddr + std::to_string(r+1);

	struct sockaddr_in toSockAddr;
	toSockAddr.sin_family=AF_INET;
	toSockAddr.sin_addr.s_addr=inet_addr(toAddr.c_str());
	toSockAddr.sin_port=htons(42001);

	std::string msg = "I am robot " + std::to_string(_idRobot); 
    if (DEBUG) cout << "[MR] " << "Sending to robot: " << r << std::endl;
	sendto(_iSock, msg.c_str(), strlen(msg.c_str())+1, 0, (struct sockaddr*) &toSockAddr, sizeof(toSockAddr));
      }
    
      usleep(250000);
    }
    if (DEBUG) cout << "[MR] " << "Send thread finished." << std::endl;
    stop();
  }

  void receiveFromThread(){
    while (ros::ok()){
      struct sockaddr_in fromSockAddr;
      int fromSockAddrLen=sizeof(struct sockaddr_in);
  
      int sizebuf = 100;
      char buffer[sizebuf];
      
      int nbytes = recvfrom(_iSock, &buffer, sizebuf ,0,(struct sockaddr*)&fromSockAddr, (socklen_t*)&fromSockAddrLen);
      if (DEBUG) fprintf(stderr, "Received %i bytes.\n", nbytes);
      char ipAddress[INET_ADDRSTRLEN];

      inet_ntop(AF_INET, &(fromSockAddr.sin_addr.s_addr), ipAddress, INET_ADDRSTRLEN);
      if (DEBUG) cout << "[MR] " << "Received from: " << ipAddress << std::endl;
      int a,b,c,d,idRobotFrom;
      sscanf(ipAddress, "%d.%d.%d.%d", &a,&b,&c,&d);
      idRobotFrom = d-1;
      if (DEBUG) cout << "[MR] " << "Robot id: " << idRobotFrom << std::endl;

      if (nbytes == 0) //Connection was shutdown
	break;

      if (DEBUG) cout << "[MR] " << "Received: " << buffer << std::endl;
      
      cg_mrslam::Ping pingmsg;
      pingmsg.header.stamp = ros::Time::now();
      pingmsg.robotFrom = idRobotFrom;
      pingmsg.robotTo   = _idRobot;
  
      _pubPing.publish(pingmsg);
    }
    if (DEBUG) cout << "[MR] " << "Receive thread finished." << std::endl;
  }

  void init(){
    //Setting up network
    std::string my_addr = _baseAddr+std::to_string(_idRobot+1);
    if (DEBUG) cout << "[MR] " << "My address: " << my_addr << std::endl;
    _iSock = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

    struct sockaddr_in sockAddr;
    sockAddr.sin_family=AF_INET;
    sockAddr.sin_addr.s_addr=inet_addr(my_addr.c_str());
    sockAddr.sin_port=htons(42001);
    bind(_iSock,(struct sockaddr*)&sockAddr,sizeof(sockAddr));
  }

  void start(){
    //Start ROS cg_mrslam::Ping Publisher
    _pubPing = _n.advertise<cg_mrslam::Ping>("ping_msgs", 1);

    //Start threads
    if (DEBUG) cout << "[MR] " << "creating send thread for robot " << _idRobot << std::endl;
    _sendThread = std::thread(&CommPublisher::sendToThread, this);
    if (DEBUG) cout << "[MR] " << "creating receive thread for robot " << _idRobot << std::endl;
    _receiveThread = std::thread(&CommPublisher::receiveFromThread, this);

    _sendThread.join();
    _receiveThread.join();
  }

  void stop(){
    //Closing socket
    if (DEBUG) cout << "[MR] " << "Closing socket" << std::endl;
    //close(_iSock);
    shutdown(_iSock, SHUT_RDWR);
  }

protected:
  int _idRobot;
  int _nRobots;
  std::string _baseAddr;
  int _iSock;
  
  ros::NodeHandle _n;
  ros::Publisher _pubPing;

  std::thread _sendThread;
  std::thread _receiveThread;
};


int main(int argc, char **argv){

  CommandArgs arg;
  int nRobots;
  int idRobot;
  std::string base_addr;

  ros::init(argc, argv, "comm_publisher");

  arg.param("idRobot", idRobot, 0, "robot identifier" );
  arg.param("nRobots", nRobots, 1, "number of robots in the network" );
  arg.param("base_addr", base_addr, "127.0.0.", "base network addres");
  arg.parseArgs(argc, argv);

  CommPublisher cp(idRobot, nRobots, base_addr);
  cp.init();

  cp.start();

  ros::spin();

  if (DEBUG) cout << "[MR] " << "Finished" << std::endl;

}

// Copyright (c) 2013, Maria Teresa Lazaro Grañon
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "graph_comm2.h"
#include <string>
#include "definitions.h"

GraphComm2::GraphComm2 (MRGraphSLAM* gslam, int idRobot, int nRobots, RosHandler* rh){

  _idRobot = idRobot;
  _nRobots = nRobots;

  _gslam = gslam;
  _rh = rh;

}

void GraphComm2::init_threads(){

  for (int i=0; i<_nRobots; i++) {
     if (i!=_idRobot)
        _subRecvReal2.push_back(_rh->GetNodeHandle().subscribe<cg_mrslam::SLAM>("/" + _ns + to_string(i) + "/mrslam_msgs_r" + to_string(_idRobot), 100,  &GraphComm2::receiveFromThrd, this));
  }

  for (int i=0; i<_nRobots; i++) {
     if (i!=_idRobot)
        _pubsSentReal2.push_back(_rh->GetNodeHandle().advertise<cg_mrslam::SLAM>("/" + _ns + to_string(_idRobot) + "/mrslam_msgs_r" + to_string(i), 100));
     else _pubsSentReal2.push_back(ros::Publisher());
  }

  sthread = boost::thread(&GraphComm2::sendToThrd, this);
  pthread = boost::thread(&GraphComm2::processQueueThrd, this);
}



bool isEqualCondGraphs(CondensedGraphMessage one, CondensedGraphMessage two) {
    if (one.edgeVector.size() != two.edgeVector.size() || one.closures.size() != two.closures.size())
        return false;

    for (int i=0; i < one.edgeVector.size(); i++) {
        if (one.edgeVector[i].idfrom != one.edgeVector[i].idfrom || one.edgeVector[i].idto != one.edgeVector[i].idto)
            return false;
    }

    for (int i=0; i < one.edgeVector.size(); i++) {
        if (one.closures[i] != one.closures[i])
            return false;
    }

    return true;
}


void GraphComm2::sendToThrd() {
  int lastSentVertex = -1;
  CondensedGraphMessage* previous = new CondensedGraphMessage();
  while(1){

     //ComboMessage
     ComboMessage* cmsg = nullptr;
     VertexSE2* last = _gslam->lastVertex();
     if (last->id() != lastSentVertex){
        lastSentVertex = last->id();
        cmsg = _gslam->constructComboMessage();
     }

     for (size_t i = 0; i < _nRobots; i++) {
         //send only if receiver subscribed to topic
         if (i != _idRobot && _pubsSentReal2.at(i).getNumSubscribers() > 0) {

             //ComboMessage
             if (cmsg != nullptr){
                cg_mrslam::SLAM dslamMsg;
                _rh->createDSlamMsg(cmsg, dslamMsg);
                _pubsSentReal2.at(i).publish(dslamMsg);
                ROS_INFO_STREAM("Sent ComboMessage to robot " << i);
             }

             //CondensedGraphMessage
             CondensedGraphMessage* gmsg = _gslam->constructCondensedGraphMessage(i);
             if (gmsg && !isEqualCondGraphs(*gmsg, *previous)) {
                 previous = gmsg;
                 cg_mrslam::SLAM dslamMsg;
                 _rh->createDSlamMsg(gmsg, dslamMsg);
                 _pubsSentReal2.at(i).publish(dslamMsg);
                 ROS_INFO_STREAM("Sent CondensedGraphMessage to robot " << i);
             }
         }
     }
     usleep(1000000); //1 sec
  }
}

void GraphComm2::receiveFromThrd(cg_mrslam::SLAM dslamMsg){

    if (dslamMsg.type == 4)
        ROS_INFO_STREAM("Received ComboMessage from robot " << dslamMsg.robotId);
    else if (dslamMsg.type == 7)
        ROS_INFO_STREAM("Received CondensedGraphMessage from robot " << dslamMsg.robotId);

    StampedRobotMessage vmsg;
    RobotMessage* rmsg;
    _rh->restoreDSlamMsg(&rmsg, dslamMsg);
    vmsg.msg = rmsg;
    vmsg.refVertex = _gslam->lastVertex();

    boost::mutex::scoped_lock lock(_queueMutex);
    _queue.push(vmsg);
}

void GraphComm2::processQueueThrd(){

  while(1){
    if (!_queue.empty()){

      boost::mutex::scoped_lock lock(_queueMutex);
      StampedRobotMessage vmsg = _queue.front();
      _queue.pop();
      lock.unlock();

      _gslam->addInterRobotData(vmsg);

    }else
      usleep(10000);
  }
}

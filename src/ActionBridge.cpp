#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <apc/MoveToAction.h>
#include <actionlib/client/simple_action_client.h>

#include <apc/Recognized.h>

class ActionBridge{
  public:
    ActionBridge();
    void callback(apc::Recognized);
  private:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    int id;
    double centX;
    double centY;
    double centZ;
    char bin;
    actionlib::SimpleActionClient<apc::MoveToAction> client;
};

ActionBridge::ActionBridge() : client("MoveTo", true){
  sub = nh.subscribe("/apc/move_server", 100, &ActionBridge::callback,this);
  pub = nh.advertise<apc::Recognized>("/apc/successful_pickup",1);
  client.waitForServer();
}

void ActionBridge::callback(apc::Recognized info_msg){
  ROS_INFO("received move message");
  apc::Recognized msg_clone = info_msg;
  id = info_msg.obj_id;
  centX = info_msg.centroid_x;
  centY = info_msg.centroid_y;
  centZ = info_msg.centroid_z;
  bin = info_msg.bin_loc;
  int bin_num = bin-'A';
  apc::MoveToGoal goal;
  //set the arm
  if (bin_num%3 < 1){
    goal.arm = apc::MoveToGoal::LEFT_ARM;
  }
  else{
    goal.arm = apc::MoveToGoal::RIGHT_ARM;
  }
  //set the bin identification letter
  std_msgs::String shelf;
  shelf.data = bin;
  goal.shelf = shelf;
  //check if object is recognized
  if (id == 0){
    //MOVE_TO_SHELF
    goal.moveAction = apc::MoveToGoal::MOVE_TO_SHELF;
  }
  else {
    goal.movePose.position.x = centX;
    goal.movePose.position.y = centY;
    goal.movePose.position.z = centZ;
    if (info_msg.is3d){
      //MOVE_TO_PICK if 3D
      goal.moveAction = apc::MoveToGoal::MOVE_TO_PICK;
    }
    else{
      //MOVE_ALONG_LIINE if 2D
      //goal.moveAction = apc::MoveToGoal::MOVE_ALONG_LINE;
    }
  }
  ROS_INFO("sending goal");
  client.sendGoal(goal);
  ROS_INFO("starting to wait");
  client.waitForResult(ros::Duration(60.0));
  ROS_INFO("done waiting");
  //publish
  if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    msg_clone.confidence = 1;
  }
  else{
    msg_clone.confidence = 0;
  }
  pub.publish(msg_clone);
  ROS_INFO("published");
}

int main(int argc, char** argv){
  ROS_INFO(" init action server node");
  ros::init(argc, argv, "ActionBridge");
  ActionBridge ab;
  ROS_INFO("action bridge created");
  ros::spin();
}

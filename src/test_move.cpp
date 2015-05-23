#include <stdlib.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <apc/MoveToAction.h>
#include <actionlib/client/simple_action_client.h>

class MoveController{
	public:
		MoveController();
		void pickObject();
	protected:
		actionlib::SimpleActionClient<apc::MoveToAction> client;
		ros::NodeHandle nh;
};

MoveController::MoveController() : client("MoveTo", true) {
	client.waitForServer();
}

void MoveController::pickObject(){

	apc::MoveToGoal goal;
	goal.moveAction = apc::MoveToGoal::MOVE_TO_PICK;
	goal.movePose.position.x = 0.73;
	goal.movePose.position.y = 0.3;
	goal.movePose.position.z = 0.75;
	goal.arm = apc::MoveToGoal::LEFT_ARM;
	std_msgs::String shelf;
	while(nh.ok()){
		std::cout << "Enter shelf name (single character): ";
		std::getline(std::cin, shelf.data);
		goal.shelf = shelf;

		client.sendGoal(goal);
		client.waitForResult(ros::Duration(60.0));
		if(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_ERROR("Could not get to requested point");
		}
	}
}

int main(int argc, char **argv){
	ROS_INFO("Initializing test_move_client");
	ros::init(argc, argv, "test_move_client");
	MoveController controller;
	controller.pickObject();

	return 0;
}

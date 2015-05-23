#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/Range.h>

#include <apc/MoveToAction.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Point.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

#include <map>

class Actuator{
	public:
		enum Grip { PUSH, REGULAR };

		static const float SHELF_DEPTH = 0.435;
		static const float BOTTOM_SHELF_OFFSET = 0.8297;
		static const float SHELF_HEIGHT = 0.22860;
		static const float BOTTOM_SHELF_HEIGHT = 0.2667;
		static const float HALF_SHELF_WIDTH = 0.1524;
		static const int NUM_SHELVES = 12;
		static const float TOLERANCE = 0.02;
		static const float SCOOP_LENGTH = 0.10;
		static const float OBJ_DIST_RANGE = 7.0;
		Actuator();
		void executeMotion(const apc::MoveToGoalConstPtr& goal);
	protected:
		ros::NodeHandle nh;
		ros::Publisher rightGripper;
		ros::Publisher leftGripper;
		ros::Subscriber rangeSubscriber;
		actionlib::SimpleActionServer<apc::MoveToAction> moveToServer;
		moveit::planning_interface::MoveGroup rightArm;
		std::map<char, geometry_msgs::Point> shelf_positions;

		void calcShelfPositions();
	private:
		bool approachAndTestRange(geometry_msgs::Point, char arm, geometry_msgs::Point);
		bool tooFarFromObject();
		void rangeCallback(const sensor_msgs::Range& rangemsg);
		float currentRange;
 
		bool pushGrip(char armName, char shelfName, geometry_msgs::Point);
		bool regularGrip(char armName, geometry_msgs::Point);
		void closeGripper(char armName);
		void openGripper(char armName);
		bool isInsideShelf(geometry_msgs::Point, char);
		bool moveToPose(geometry_msgs::Pose, char arm);
		static std::map<std::string, Grip> init_grasps();

		static const std::map<std::string, Grip> grasps;
};

const std::map<std::string,Actuator::Grip> Actuator::grasps =  Actuator::init_grasps();

std::map<std::string,Actuator::Grip> Actuator::init_grasps()
{
std::map<std::string,Actuator::Grip> m;
  m["duckies"] = PUSH;
  m["Elmer's_Glue"] = REGULAR;
  return m;
}

Actuator::Actuator() : rightArm("both_arms"), moveToServer(nh, "MoveTo", boost::bind(&Actuator::executeMotion, this, _1), false) {


//	rightArm.setPlannerId("PRMkConfigDefault");
	rightArm.setPlanningTime(15);

	moveToServer.start();

	calcShelfPositions();

	rightGripper = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 1);
	leftGripper = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
	rangeSubscriber = nh.subscribe("/robot/range/left_hand_range/state", 1, &Actuator::rangeCallback, this); 
	currentRange = 1000.0;
}

void Actuator::executeMotion(const apc::MoveToGoalConstPtr& goal){

	if(goal->moveAction == apc::MoveToGoal::MOVE_TO_POSE){
		ROS_INFO("Moving arm to specified location");
		if(moveToPose(goal->movePose, goal->arm))
			moveToServer.setSucceeded();
		else
			moveToServer.setAborted();
	}
	else if (goal->moveAction == apc::MoveToGoal::MOVE_TO_PICK){
		// Do some error handling
		if(goal->shelf.data.size() != 1 || goal->shelf.data.at(0) > 'L' || goal->shelf.data.at(0) < 'A'){
			ROS_WARN("Incorrect shelf name passed in");
			moveToServer.setAborted();
		}
		else if(!isInsideShelf(goal->movePose.position, goal->shelf.data.at(0))){
			ROS_WARN("Point is not inside specified shelf");
			moveToServer.setAborted();
		}
		else{
			ROS_INFO("Moving arm to front of shelf %s", goal->shelf.data.c_str());

			geometry_msgs::Pose shelfPose;
			shelfPose.position = shelf_positions.at(goal->shelf.data.at(0));
			// make orientation horizontal in front of shelf
			shelfPose.orientation.w = 0.7071;
			shelfPose.orientation.y = 0.7071;
			// move to point in front of shelf, if successful, move into shelf
			if(moveToPose(shelfPose, goal->arm) && regularGrip(goal->arm, goal->movePose.position)){// && moveToPose(goal->movePose, goal->arm)){
				moveToServer.setSucceeded();
			}
			else
				moveToServer.setAborted();
		}
	}
	else if (goal->moveAction == apc::MoveToGoal::MOVE_ALONG_LINE){
		// Do some error handling
		if(goal->shelf.data.size() != 1 || goal->shelf.data.at(0) > 'L' || goal->shelf.data.at(0) < 'A'){
			ROS_WARN("Incorrect shelf name passed in");
			moveToServer.setAborted();
		}
		else if(!isInsideShelf(goal->movePose.position, goal->shelf.data.at(0))){
			ROS_WARN("Point is not inside specified shelf");
			moveToServer.setAborted();
		}
		else{
			ROS_INFO("Moving arm to front of shelf %s", goal->shelf.data.c_str());

			geometry_msgs::Pose shelfPose;
			shelfPose.position = shelf_positions.at(goal->shelf.data.at(0));
			// make orientation horizontal in front of shelf
			shelfPose.orientation.w = 0.7071;
			shelfPose.orientation.y = 0.7071;
			// move to point in front of shelf, if successful, move into shelf
			if(moveToPose(shelfPose, goal->arm) && approachAndTestRange(goal->movePose.position, goal->arm, shelfPose.position)){
				moveToServer.setSucceeded();
			}
			else
				moveToServer.setAborted();
		}
	}
	else if (goal->moveAction == apc::MoveToGoal::MOVE_TO_SHELF){
		// Do some error handling
		if(goal->shelf.data.size() != 1 || goal->shelf.data.at(0) > 'L' || goal->shelf.data.at(0) < 'A'){
			ROS_WARN("Incorrect shelf name passed in");
			moveToServer.setAborted();
		}
		else{
			ROS_INFO("Moving arm to front of shelf %s", goal->shelf.data.c_str());

			geometry_msgs::Pose shelfPose;
			shelfPose.position = shelf_positions.at(goal->shelf.data.at(0));
			// make orientation horizontal in front of shelf
			shelfPose.orientation.w = 0.7071;
			shelfPose.orientation.y = 0.7071;
			// move to point in front of shelf, if successful, move into shelf
			if(moveToPose(shelfPose, goal->arm)){
				moveToServer.setSucceeded();
			}
			else
				moveToServer.setAborted();
		}
	}
	else if (goal->moveAction == apc::MoveToGoal::MOVE_TO_DROP){
		// Do some error handling
		if(goal->shelf.data.size() != 1 || goal->shelf.data.at(0) > 'L' || goal->shelf.data.at(0) < 'A'){
			ROS_WARN("Incorrect shelf name passed in");
			moveToServer.setAborted();
		}
		else if(!isInsideShelf(rightArm.getCurrentPose().pose.position, goal->shelf.data.at(0))){
			ROS_WARN("Robot is not inside specified shelf");
			moveToServer.setAborted();
		}
		else{
			ROS_INFO("Moving arm to front of shelf %s and then drop location", goal->shelf.data.c_str());

			geometry_msgs::Pose shelfPose;
			shelfPose.position = shelf_positions.at(goal->shelf.data.at(0));
			// make orientation horizontal in front of shelf
			shelfPose.orientation.w = 0.7071;
			shelfPose.orientation.y = 0.7071;
			// move to point in front of shelf, if successful, move into shelf
			if(moveToPose(shelfPose, goal->arm) && moveToPose(goal->movePose, goal->arm)){
				moveToServer.setSucceeded();
			}
			else
				moveToServer.setAborted();
		}
	}
	else if (goal->moveAction == apc::MoveToGoal::SCOOP_SHELF){
		// Do some error handling
		if(goal->shelf.data.size() != 1 || goal->shelf.data.at(0) > 'L' || goal->shelf.data.at(0) < 'A'){
			ROS_WARN("Incorrect shelf name passed in");
			moveToServer.setAborted();
		}
		else{
			ROS_INFO("Moving arm to front of shelf %s", goal->shelf.data.c_str());

			geometry_msgs::Pose shelfPose;
			shelfPose.position = shelf_positions.at(goal->shelf.data.at(0));
			shelfPose.position.x -= SCOOP_LENGTH;
			// make orientation the same as the slope of shelf
			shelfPose.orientation.w = 0.4777;
			shelfPose.orientation.x = -0.5214;
			shelfPose.orientation.y = 0.5214;
			shelfPose.orientation.z = -0.4777;
			// move to point in front of shelf, if successful, move into shelf
			if(moveToPose(shelfPose, goal->arm)){
				moveToServer.setSucceeded();
			}
			else
				moveToServer.setAborted();
		}
	}
}

void Actuator::calcShelfPositions(){
	geometry_msgs::Point shelfPos;
	char shelfName = 'A';

	// Get point in front of each shelf
	double shelfX;
	nh.getParam("shelf_distance", shelfX);
	shelfPos.x = shelfX - SHELF_DEPTH - 0.02;  // Subtract 2cm for tolerance
	// Iterate over shelves from A - L and add them to map
	for(int i = 0; i < NUM_SHELVES; i++){
		shelfPos.y = (1-(i % 3))*2*HALF_SHELF_WIDTH;
		if(i < 9)
			shelfPos.z = BOTTOM_SHELF_OFFSET + BOTTOM_SHELF_HEIGHT + (ceil((NUM_SHELVES -  i) / 3.0) - 1)*SHELF_HEIGHT - SHELF_HEIGHT/2.0 - 0.91;
		else
			shelfPos.z = BOTTOM_SHELF_OFFSET + BOTTOM_SHELF_HEIGHT/2.0 - 0.91; // Offset for Baxter's coordinate system
		shelf_positions.insert(std::pair<char, geometry_msgs::Point>(shelfName+i,shelfPos));
	}
}

bool Actuator::pushGrip(char armName, char shelfName, geometry_msgs::Point surfacePt){
	geometry_msgs::Pose gripPose;
	geometry_msgs::Point shelfPoint;
	gripPose.position = surfacePt;
	gripPose.orientation.w = 0.7071;
	gripPose.orientation.x = 0;
	gripPose.orientation.y = 0.7071;
	gripPose.orientation.z = 0;
	if(moveToPose(gripPose, armName)){
		//shelfPoint = shelf_positions.at(shelfName);
		//gripPose.position.z = shelfPoint.z + 0.1; // needs to change based on shelf location
		gripPose.orientation.w = 0.64278760968;
		gripPose.orientation.x = 0;
		gripPose.orientation.y = 0.76604444311;
		gripPose.orientation.z = 0;
		if(moveToPose(gripPose, armName)){
			closeGripper(armName);
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool Actuator::regularGrip(char armName, geometry_msgs::Point surfacePt){
	geometry_msgs::Pose gripPose;
	gripPose.position = surfacePt;
	gripPose.orientation.w = 0.7071;
	gripPose.orientation.x = 0;
	gripPose.orientation.y = 0.7071;
	gripPose.orientation.z = 0;
	if(moveToPose(gripPose, armName)){
		closeGripper(armName);
		return true;
	}
	else
		return false;
}


bool Actuator::approachAndTestRange(geometry_msgs::Point objectPoint, char armName, geometry_msgs::Point shelfPoint){
	float moveAmount = 0.01;
	geometry_msgs::Pose pose;
	// horizontal orientation
	pose.orientation.w = 0.7071;
	pose.orientation.y = 0.7071;
	pose.position = objectPoint;
	pose.position.x = shelfPoint.x;
	while(tooFarFromObject() && moveToPose(pose, armName)){
		pose.position.x += moveAmount;
	}
	// if the hand is too far from the object
	// then the loop must have exited due to failed planning so return false
	if(!tooFarFromObject()){
		rightArm.stop();
		closeGripper(armName);
		return true;
	}
	return false;
}


bool Actuator::tooFarFromObject(){
	return currentRange <= OBJ_DIST_RANGE;
}

bool Actuator::moveToPose(geometry_msgs::Pose goalPose, char armName){
	bool isPlanningSuccess;
	moveit::planning_interface::MoveGroup::Plan my_plan;
	rightArm.setStartState(*rightArm.getCurrentState());
	std::string planningArm = "left_gripper";
	if(armName == apc::MoveToGoal::RIGHT_ARM){
		planningArm = "right_gripper";
	}
	else if(armName != apc::MoveToGoal::LEFT_ARM){
		ROS_WARN("Unknown arm received. Using left arm");
	}
	rightArm.setPoseTarget(goalPose, planningArm.c_str());

	// call the planner to compute the plan and visualize it
	isPlanningSuccess = rightArm.plan(my_plan);

	/* *****************  FOR DEBUGGING ***************/
	ROS_INFO("Visualizing plan %s", isPlanningSuccess?"":"FAILED");

	if(isPlanningSuccess){
		rightArm.execute(my_plan);
		return true;
	}
	else{
		return false;
	}
}

bool Actuator::isInsideShelf(geometry_msgs::Point location, char shelf){
	int shelfNum = shelf - 'A';
	float shelfBottom, shelfTop, shelfLeft, shelfFront;

	// Get point in front of each shelf
	double shelfX;
	nh.getParam("shelf_distance", shelfX);
	shelfFront = shelfX - SHELF_DEPTH;

	shelfLeft = (1-(shelfNum % 3))*2*HALF_SHELF_WIDTH + HALF_SHELF_WIDTH;
	if(shelfNum < 9){
		shelfBottom = BOTTOM_SHELF_OFFSET + (ceil((NUM_SHELVES -  shelfNum) / 3.0) - 1)*SHELF_HEIGHT - 0.91;
		shelfTop = shelfBottom + SHELF_HEIGHT;
	}
	else{
		shelfBottom = BOTTOM_SHELF_OFFSET - 0.91;
		shelfTop = shelfBottom + BOTTOM_SHELF_HEIGHT;
	}

	return location.y < (shelfLeft + TOLERANCE) &&
		 location.y > (shelfLeft - 2*HALF_SHELF_WIDTH - TOLERANCE) &&
		 location.z < (shelfTop + TOLERANCE) && location.z > (shelfBottom - TOLERANCE) &&
		 location.x > (shelfFront - TOLERANCE) && location.x < (shelfX + TOLERANCE);
}

void Actuator::closeGripper(char armName){
	baxter_core_msgs::EndEffectorCommand cmd;
	cmd.id = 65538;
	cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
	if(armName == apc::MoveToGoal::RIGHT_ARM){
		rightGripper.publish(cmd);
	}
	else
		leftGripper.publish(cmd);
}

void Actuator::openGripper(char armName){
	baxter_core_msgs::EndEffectorCommand cmd;
	cmd.id = 65538;
	cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
	if(armName == apc::MoveToGoal::RIGHT_ARM){
		rightGripper.publish(cmd);
	}
	else
		leftGripper.publish(cmd);
}

void Actuator::rangeCallback(const sensor_msgs::Range& rangemsg){
	currentRange = rangemsg.range;	
}

int main(int argc, char **argv){
	ROS_INFO("Starting actuate server");
	ros::init(argc, argv, "actuate_server");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	Actuator actuate;

	ros::waitForShutdown();
	return 0;
}

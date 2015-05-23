#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

class Actuator{
	public:
		Actuator();
		void goToPos(const geometry_msgs::Pose& grasp);
	protected:
		ros::Subscriber graspSubscriber;
		ros::NodeHandle nh;
		moveit::planning_interface::MoveGroup rightArm;
};

Actuator::Actuator() : rightArm("right_arm") {
	ros::NodeHandle nh;

	// Print the name of the reference frame for this robot
	ROS_INFO("Reference frame: %s", rightArm.getPlanningFrame().c_str());
	ROS_INFO("Reference frame: %s", rightArm.getEndEffectorLink().c_str());

	rightArm.setPlannerId("RRTkConfigDefault");

	// subscribe to grasp point
	graspSubscriber = nh.subscribe("move_point", 1, &Actuator::goToPos, this);
}

void Actuator::goToPos(const geometry_msgs::Pose &grasp){
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "right_hand";
	ocm.header.frame_id = rightArm.getPlanningFrame();
	ocm.orientation.w = 0.7071;
	ocm.orientation.x = 0.0;
	ocm.orientation.y = 0.7071;
	ocm.orientation.z = 0.0;
	ocm.absolute_x_axis_tolerance = 3.14;
	ocm.absolute_y_axis_tolerance = 3.14;
	ocm.absolute_z_axis_tolerance = 3.14;
	ocm.weight = 1.0;

	// set the constraint 
	moveit_msgs::Constraints constraints;
	constraints.orientation_constraints.push_back(ocm);
	rightArm.setPathConstraints(constraints);

        rightArm.setStartState(*rightArm.getCurrentState());

	rightArm.setPoseTarget(grasp);
	rightArm.setPlanningTime(60);

	// call the planner to compute the plan and visualize it
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = rightArm.plan(my_plan);

	ROS_INFO("Visualizing plan %s", success?"":"FAILED");

	// Sleep to give Rviz time to visualize the plan
	sleep(5.0);
}
	

int main(int argc, char **argv){
	ROS_INFO("Starting move arm node");
	ros::init(argc, argv, "move_arm_node");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	Actuator actuate;

	ros::waitForShutdown();
	return 0;
}

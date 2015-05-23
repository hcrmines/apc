#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include "apc/AddScoop.h"
#include "std_msgs/Bool.h"

#include <string>

class AddObjects{
	public:
		AddObjects();
		void handleScoop(apc::AddScoop);
		void handleShelf(std_msgs::Bool);
	protected:
		ros::NodeHandle nh;
		moveit::planning_interface::MoveGroup arm;
		moveit_msgs::PlanningScene planning_scene;
		ros::Publisher planning_scene_diff_publisher;
		ros::Subscriber scoopSubscriber;
		ros::Subscriber shelfSubscriber;
	private:
		void addShelf();
		void removeShelf();
		void addScoop(std::string);
		void removeScoop();
};

AddObjects::AddObjects() : arm("both_arms") {
	ros::NodeHandle nh;

	planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
	  ros::WallDuration sleep_t(0.5);
	  sleep_t.sleep();
	}
	scoopSubscriber = nh.subscribe("add_scoop", 1, &AddObjects::handleScoop, this);
	shelfSubscriber = nh.subscribe("add_shelf", 1, &AddObjects::handleShelf, this);

	addShelf();
}

void AddObjects::handleShelf(std_msgs::Bool message){
	if(message.data)
		addShelf();
	else
		removeShelf();
}

void AddObjects::handleScoop(apc::AddScoop message){
	if(message.add.data)
		addScoop(message.arm.data);
	else
		removeScoop();
}

void AddObjects::addShelf(){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = arm.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = "shelf";

	shapes::Mesh* m = shapes::createMeshFromResource("package://apc/collision_objects/pod_lowres.stl");
	shape_msgs::Mesh shelf_mesh;
	shapes::ShapeMsg shelf_mesh_msg;
	shapes::constructMsgFromShape(m,shelf_mesh_msg);
	shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose shelf_pose;
	shelf_pose.orientation.w = 0.5;
	shelf_pose.orientation.x = 0.5;
	shelf_pose.orientation.y = 0.5;
	shelf_pose.orientation.z = 0.5;
	shelf_pose.position.x =  1.0;
	shelf_pose.position.y =  0.0;
	shelf_pose.position.z =  -0.91;

	collision_object.meshes.push_back(shelf_mesh);
	collision_object.mesh_poses.push_back(shelf_pose);
	collision_object.operation = collision_object.ADD;

	ROS_INFO("Add an shelf into the world");

	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(collision_object);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);
}

void AddObjects::addScoop(std::string side){
	        // put in collision avoidance data
        moveit_msgs::AttachedCollisionObject scoop_object;
	if(side == "right"){
		scoop_object.link_name = "right_gripper";
		scoop_object.object.header.frame_id = "right_gripper";
	}
	else{
		scoop_object.link_name = "left_gripper";
		scoop_object.object.header.frame_id = "left_gripper";
	}

        /* The id of the object is used to identify it. */
        scoop_object.object.id = "scoop";

        shapes::Mesh* m = shapes::createMeshFromResource("package://apc/collision_objects/scoop.stl");
        shape_msgs::Mesh scoop_mesh;
        shapes::ShapeMsg scoop_mesh_msg;
        shapes::constructMsgFromShape(m,scoop_mesh_msg);
        scoop_mesh = boost::get<shape_msgs::Mesh>(scoop_mesh_msg);

        /* A pose for the box (specified relative to frame_id) */
        geometry_msgs::Pose scoop_pose;
        scoop_pose.orientation.w = 1.0;
        scoop_pose.orientation.x = 0.0;
        scoop_pose.orientation.y = 0.0;
        scoop_pose.orientation.z = 0.0;
        scoop_pose.position.x =  0.010;
        scoop_pose.position.y =  0.0;
        scoop_pose.position.z =  0.00;

        scoop_object.object.meshes.push_back(scoop_mesh);
        scoop_object.object.mesh_poses.push_back(scoop_pose);
        scoop_object.object.operation = scoop_object.object.ADD;

        ROS_INFO("Add scoop onto the robot");

        planning_scene.robot_state.attached_collision_objects.push_back(scoop_object);
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
}

void AddObjects::removeShelf(){
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	std::vector<std::string> object_ids;
	object_ids.push_back("shelf");
	planning_scene_interface.removeCollisionObjects(object_ids);
}

void AddObjects::removeScoop(){
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	arm.detachObject("scoop");
	std::vector<std::string> object_ids;
	object_ids.push_back("scoop");
	planning_scene_interface.removeCollisionObjects(object_ids);
}


int main(int argc, char **argv){
	ROS_INFO("Starting add objects node ");
	ros::init(argc, argv, "add_objects_node");

	AddObjects addObjects;

	ros::spin();
	return 0;
}

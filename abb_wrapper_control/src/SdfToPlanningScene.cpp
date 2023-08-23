#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <sdf/sdf.hh>

#include <ignition/math/Pose3.hh>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
	tf2::Quaternion quaternion_tf2;
	quaternion_tf2.setRPY(roll, pitch, yaw);
	geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
	return quaternion;
}

/* ******************************************************************************************** */
void setObjectProperties(string name, ignition::math::v6::Vector3d loc, ignition::math::v6::Quaterniond rot, ignition::math::Vector3d dim,
						 moveit_msgs::CollisionObject &object)
{

	// Set the name of the object and its link to the world
	object.id = name;

	// Set the location

	object.primitive_poses.back().position.x = loc.X();
	object.primitive_poses.back().position.y = loc.Y();
	object.primitive_poses.back().position.z = loc.Z();

	// Set the orientation
	geometry_msgs::Quaternion quat = quaternion_from_rpy(rot.X(), rot.Y(), rot.Z());

	object.primitive_poses.back().orientation.x = quat.x;
	object.primitive_poses.back().orientation.y = quat.y;
	object.primitive_poses.back().orientation.z = quat.z;
	object.primitive_poses.back().orientation.w = quat.w;

	// Set the dimensions

	object.primitives.back().dimensions[0] = dim.X();
	object.primitives.back().dimensions[1] = dim.Y();
	object.primitives.back().dimensions[2] = dim.Z();
}

/* ******************************************************************************************** */
int main(int argc, char **argv)
{

	// Wait for other nodes to start (assuming launch file)
	// Start the ros node
	ros::init(argc, argv, "sdf_to_planning_scene");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;

	if (argc < 2)
	{
		ROS_ERROR("Usage: rosrun adaptive_grasp_controller SdfToPlanningScene <world> <optional: --skipIfco>");
	}
	ROS_INFO("%s: Sleeping to allow other nodes to start\n", ros::this_node::getName().c_str());
	ROS_INFO("%s: Input world: '%s'\n", ros::this_node::getName().c_str(), argv[1]);
	sleep(2);

	bool skipIfco = false;
	if (argc > 2 && strcmp(argv[2], "--skipIfco") == 0)
		skipIfco = true;

	// ------------------------------------------------------------
	// Define the attached object message

	// Initialize the msg
	moveit_msgs::CollisionObject object;
	object.header.frame_id = "world";

	// Set the default pose and dimensions
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	object.primitives.push_back(primitive);
	object.primitive_poses.push_back(pose);

	// An attach operation requires an ADD
	object.operation = object.ADD;

	// Advertise the planning scene message publisher
	ros::Publisher planning_scene_diff_publisher =
		node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		ROS_INFO("Waiting for a subscriber to the planning_scene channel\n");
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}

	/* PUT THE OBJECT IN THE ENVIRONMENT*/
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.is_diff = true;
	planning_scene.robot_state.is_diff = true;

	cout << "STARTING TO PARSE SDF IN SdfToPlanningScene!!!" << endl;

	// ------------------------------------------------------------
	// Parse the sdf and publish the planning scene

	// Read the sdf
	sdf::SDFPtr sdfParsed(new sdf::SDF());
	sdf::init(sdfParsed);

	const std::string sdfStringPath(argv[1]);

	std::ifstream ifs(sdfStringPath);
	std::string world_content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

	if (!sdf::readString(world_content, sdfParsed))
		cout << "COULDN'T READ SDF FILE OF SdfToPlanningScene!!!" << endl;
	;
	// assert(sdf::readFile(sdfStringPath, sdfParsed));

	// cout << "THE PATH TO FILE IS " << argv[1] << "." << endl;
	// cout << "THE PARSED SDF OF SdfToPlanningScene IS: " << sdfParsed->ToString() << endl;

	// Print the objects

	sdf::ElementPtr e = sdfParsed->Root();
	cout << e->GetDescription() << endl;
	sdf::ElementPtr world = e->GetElement("world");
	sdf::ElementPtr model = world->GetFirstElement();

	// cout << "THE SDF OF SdfToPlanningScene IS: " << e->ToString("**") << endl;

	// cout << "THE WORLD OF SdfToPlanningScene IS: " << world->ToString("--") << endl;

	// cout << "ENTERING WHILE LOOP OF SdfToPlanningScene!!!" << endl;

	while (model != NULL)
	{
		// cout << "ENTERED WHILE LOOP OF SdfToPlanningScene!!!" << endl;

		// Make sure this SDF element is a model
		if (model->GetName().compare("model") != 0)
		{
			// cout << "MODEL NOT VALID IN SdfToPlanningScene!!! The model name was " << model->GetName() << "." << endl;
			model = model->GetNextElement();
			continue;
		}

		printf("-------------------------------------------------\n");
		unsigned int param_idx = 0; // to stop some warning...
		sdf::ParamPtr param = model->GetAttribute(param_idx);
		cout << param->GetKey() << " " << param->GetAsString() << endl;
		string name = param->GetAsString();

		// Skip if ifco
		if (skipIfco && name.find("ifco") != string::npos)
		{
			model = model->GetNextElement();
			continue;
		}

		// Get the static property
		sdf::ElementPtr staticElem = model->GetElement("static");
		assert(staticElem != NULL && "No static information in model!");
		cout << "Static: " << staticElem->GetValue()->GetAsString() << endl;

		// Skip if the object is not static
		if (atoi(staticElem->GetValue()->GetAsString().c_str()) == 0)
		{
			model = model->GetNextElement();
			continue;
		}

		// Get the pose

		sdf::ElementPtr poseElem = model->GetElement("pose");
		assert(poseElem != NULL && "No pose information in model!");
		// ignition::math::Pose3d pose;

		// ignition::math::v4::Pose3<double> pose;

		ignition::math::v6::Pose3<double> pose;

		// sdf::Pose pose;
		stringstream ss_pose(poseElem->GetValue()->GetAsString());
		ss_pose >> pose;
		cout << "Pose: " << pose << endl;

		// Check that the collision object is a box
		sdf::ElementPtr linkElem = model->GetElement("link");
		assert(linkElem != NULL && "No link element in model!");
		sdf::ElementPtr collisionElem = linkElem->GetElement("collision");
		assert(collisionElem != NULL && "No collision element in link model!");
		sdf::ElementPtr geometryElem = collisionElem->GetElement("geometry");
		assert(geometryElem != NULL && "No geometry element in collision model!");
		sdf::ElementPtr boxElem = geometryElem->GetElement("box");
		assert(boxElem != NULL && "No box element in collision geometry!");

		// Get the collision box dimensions
		sdf::ElementPtr sizeElem = boxElem->GetElement("size");
		assert(sizeElem != NULL && "No size element in box!");
		stringstream ss_size(sizeElem->GetValue()->GetAsString());
		ignition::math::Vector3d size;
		// sdf::Vector3 size;
		ss_size >> size;
		cout << "Box dims: " << size << endl;
		// Fill the object pose and dimensions into the planning_scene message
		// setObjectProperties(name, pose.pos, size, object);

		setObjectProperties(name, pose.Pos(), pose.Rot(), size, object);
		planning_scene.world.collision_objects.clear();
		planning_scene.world.collision_objects.push_back(object);

		// Send the message
		ROS_INFO("Putting the object back into the environment");
		planning_scene_diff_publisher.publish(planning_scene);

		// Switch to the next model element
		model = model->GetNextElement();
		sleep(1);
	}

	// cout << "EXITING WHILE LOOP OF SdfToPlanningScene!!!" << endl;

	return EXIT_SUCCESS;
}

void setObjectProperties(string, ignition::math::v6::Vector3d, ignition::math::v6::Quaterniond, ignition::math::Vector3d,
						 moveit_msgs::CollisionObject &);

geometry_msgs::Quaternion quaternion_from_rpy(double, double, double);

/* ******************************************************************************************** */
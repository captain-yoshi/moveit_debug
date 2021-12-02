/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc:
*/

#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>

constexpr char ROBOT_DESCRIPTION[] = "robot_description";
constexpr std::size_t TRIALS = 1;

int main(int argc, char** argv) {
	ros::init(argc, argv, "urdf_to_scene");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle pnh("~");

	// Destroy loader
	moveit::core::RobotModelPtr model;
	{
		robot_model_loader::RobotModelLoader loader(ROBOT_DESCRIPTION);
		model = loader.getModel();
		if (!model)
			std::runtime_error("No robot model for robot_description: '" + std::string(ROBOT_DESCRIPTION));
	}

	planning_pipeline::PlanningPipelinePtr pipeline;

	ros::NodeHandle pn(pnh, "/move_group/planning_pipelines/ompl");
	pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(model, pn);

	pipeline->checkSolutionPaths(false);

	// Prepare planning arguments
	auto scene = std::make_shared<planning_scene::PlanningScene>(model);
	planning_interface::MotionPlanRequest req;

	// Start state
	moveit_msgs::RobotState start_state;
	start_state.joint_state.header.frame_id = "panda_link0";
	start_state.joint_state.name = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
		                              "panda_joint5", "panda_joint6", "panda_joint7" };
	start_state.joint_state.position = { 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785 };

	// Pose Goal
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "panda_link0";
	pose.pose.position.x = 0.5;
	pose.pose.position.y = 0.0;
	pose.pose.position.z = 0.3;
	pose.pose.orientation.w = 1.0;

	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	moveit_msgs::Constraints pose_goal =
	    kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

	moveit_msgs::Constraints path_constraint;
	moveit_msgs::PositionConstraint pc;
	pc.header.frame_id = "panda_link0";
	pc.link_name = "panda_link8";
	pc.target_point_offset.x = 0;
	pc.target_point_offset.y = 0;
	pc.target_point_offset.z = 0;
	pc.weight = 1;
	pc.constraint_region.primitives.emplace_back();
	pc.constraint_region.primitive_poses.emplace_back();

	auto& primitive = pc.constraint_region.primitives.back();
	primitive.type = primitive.BOX;
	primitive.dimensions = { 0.4, 0.1, 0.4 };

	auto& primitive_pose = pc.constraint_region.primitive_poses.back();
	primitive_pose.position.x = 0.4;
	primitive_pose.position.y = 0.0;
	primitive_pose.position.z = 0.45;
	primitive_pose.orientation.x = 0;
	primitive_pose.orientation.y = 0;
	primitive_pose.orientation.z = 0;
	primitive_pose.orientation.w = 1;

	moveit_msgs::Constraints co;
	path_constraint.position_constraints.emplace_back(pc);

	// Fill Motion Plan Request
	req.pipeline_id = "ompl";
	req.planner_id = "RRTConnectkConfigDefault";
	req.allowed_planning_time = 5.0;
	req.group_name = "panda_arm";
	req.start_state = start_state;
	req.goal_constraints.emplace_back(pose_goal);
	req.path_constraints = path_constraint;  // Comment this line and the error goes magically away!

	// Plan
	planning_interface::MotionPlanResponse res;
	for (std::size_t i = 0; i < TRIALS; ++i) {
		pipeline->generatePlan(scene, req, res);

		if (res.error_code_.val != res.error_code_.SUCCESS) {
			ROS_ERROR("Motion planning failed");
		}
	}
	return 0;
}

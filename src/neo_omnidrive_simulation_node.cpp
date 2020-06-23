/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
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
 *   * Neither the name of the Neobotix nor the names of its
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

#include "../include/OmniKinematics.h"
#include "../include/VelocitySolver.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <mutex>


class NeoOmniDriveNode {
public:
	NeoOmniDriveNode()
	{
		m_node_handle.param("broadcast_tf", m_broadcast_tf, true);

		if(!m_node_handle.getParam("num_wheels", m_num_wheels)) {
			throw std::logic_error("missing num_wheels param");
		}
		if(!m_node_handle.getParam("wheel_radius", m_wheel_radius)) {
			throw std::logic_error("missing wheel_radius param");
		}
		if(!m_node_handle.getParam("wheel_lever_arm", m_wheel_lever_arm)) {
			throw std::logic_error("missing wheel_lever_arm param");
		}
		m_node_handle.param("cmd_timeout", m_cmd_timeout, 0.1);
		m_node_handle.param("homeing_button", m_homeing_button, 0);
		m_node_handle.param("steer_reset_button", m_steer_reset_button, 1);

		if(m_num_wheels < 1) {
			throw std::logic_error("invalid num_wheels param");
		}
		m_wheels.resize(m_num_wheels);

		for(int i = 0; i < m_num_wheels; ++i)
		{
			m_wheels[i].lever_arm = m_wheel_lever_arm;

			if(!m_node_handle.getParam("drive" + std::to_string(i) + "/joint_name", m_wheels[i].drive_joint_name)) {
				throw std::logic_error("joint_name param missing for drive motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/joint_name", m_wheels[i].steer_joint_name)) {
				throw std::logic_error("joint_name param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/center_pos_x", m_wheels[i].center_pos_x)) {
				throw std::logic_error("center_pos_x param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/center_pos_y", m_wheels[i].center_pos_y)) {
				throw std::logic_error("center_pos_y param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/home_angle", m_wheels[i].home_angle)) {
				throw std::logic_error("home_angle param missing for steering motor" + std::to_string(i));
			}
			m_wheels[i].home_angle = M_PI * m_wheels[i].home_angle / 180.;
		}

		//m_pub_odometry = m_node_handle.advertise<nav_msgs::Odometry>("/odom", 1);
		fl_caster_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_caster_front_left_controller/command", 1);
		bl_caster_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_caster_back_left_controller/command", 1);
		br_caster_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_caster_back_right_controller/command", 1);
		fr_caster_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_caster_front_right_controller/command", 1);
		fl_drive_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_wheel_front_left_controller/command", 1);
		bl_drive_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_wheel_back_left_controller/command", 1);
		br_drive_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_wheel_back_right_controller/command", 1);
		fr_drive_pub = m_node_handle.advertise<std_msgs::Float64>("/mpo_700_wheel_front_right_controller/command", 1);
		

		m_sub_cmd_vel = m_node_handle.subscribe("/cmd_vel", 3, &NeoOmniDriveNode::cmd_vel_callback, this);
		m_sub_joint_state = m_node_handle.subscribe("/joint_states", 10, &NeoOmniDriveNode::joint_state_callback, this);
		m_pub_joint_trajectory = m_node_handle.advertise<trajectory_msgs::JointTrajectory>("/drives/joint_trajectory", 1);

		m_kinematics = std::make_shared<OmniKinematics>(m_num_wheels);
		m_velocity_solver = std::make_shared<VelocitySolver>(m_num_wheels);

		m_node_handle.param("zero_vel_threshold", m_kinematics->zero_vel_threshold, 0.005);
		m_node_handle.param("small_vel_threshold", m_kinematics->small_vel_threshold, 0.03);
		m_node_handle.param("steer_hysteresis", m_kinematics->steer_hysteresis, 30.0);
		m_node_handle.param("steer_hysteresis_dynamic", m_kinematics->steer_hysteresis_dynamic, 5.0);
		m_kinematics->steer_hysteresis = M_PI * m_kinematics->steer_hysteresis / 180;
		m_kinematics->steer_hysteresis_dynamic = M_PI * m_kinematics->steer_hysteresis_dynamic / 180;
		m_kinematics->initialize(m_wheels);
	}

	void control_step()
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);
		std_msgs::Float64 f1,f2,f3,f4,f5,f6,f7,f8;


		const ros::Time now = ros::Time::now();

		// check for input timeout
		if((now - m_last_cmd_time).toSec() > m_cmd_timeout)
		{
			if(!is_cmd_timeout && !m_last_cmd_time.isZero()
				&& (m_last_cmd_vel.linear.x != 0 || m_last_cmd_vel.linear.y != 0 || m_last_cmd_vel.angular.z != 0))
			{
				ROS_WARN_STREAM("cmd_vel input timeout! Stopping now.");
			}
			// reset values to zero
			m_last_cmd_vel = geometry_msgs::Twist();
			is_cmd_timeout = true;
		}
		else {
			is_cmd_timeout = false;
		}
		// compute new wheel angles and velocities
		auto cmd_wheels = m_kinematics->compute(m_wheels, m_last_cmd_vel.linear.x, m_last_cmd_vel.linear.y, m_last_cmd_vel.angular.z);

		trajectory_msgs::JointTrajectory::Ptr joint_trajectory = boost::make_shared<trajectory_msgs::JointTrajectory>();
		joint_trajectory->header.stamp = now;

		trajectory_msgs::JointTrajectoryPoint point;

		for(const auto& wheel : cmd_wheels)
		{
			joint_trajectory->joint_names.push_back(wheel.drive_joint_name);
			joint_trajectory->joint_names.push_back(wheel.steer_joint_name);
			const double drive_rot_vel = wheel.wheel_vel / m_wheel_radius;
			point.positions.push_back(0);
			point.velocities.push_back(drive_rot_vel);
			point.positions.push_back(wheel.wheel_angle);
			point.velocities.push_back(0);
		}
		joint_trajectory->points.push_back(point);
		// std::cout<<joint_trajectory->joint_names[7]<<std::endl;
		f1.data = joint_trajectory->points[0].velocities[0];
		f2.data = joint_trajectory->points[0].positions[1];
		f3.data = joint_trajectory->points[0].velocities[2];
		f4.data = joint_trajectory->points[0].positions[3];
		f5.data = joint_trajectory->points[0].velocities[4];
		f6.data = joint_trajectory->points[0].positions[5];
		f7.data = joint_trajectory->points[0].velocities[6];
		f8.data = joint_trajectory->points[0].positions[7];
		fl_caster_pub.publish(f2);
		bl_caster_pub.publish(f4);
		br_caster_pub.publish(f6);
		fr_caster_pub.publish(f8);
		fl_drive_pub.publish(f1);
		bl_drive_pub.publish(f3);
		br_drive_pub.publish(f5);
		fr_drive_pub.publish(f7);

		m_pub_joint_trajectory.publish(joint_trajectory);
	}

private:
	void cmd_vel_callback(const geometry_msgs::Twist& twist)
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);
		m_last_cmd_time = ros::Time::now();
		m_last_cmd_vel = twist;
	}

	void joint_state_callback(const sensor_msgs::JointState& joint_state)
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		const size_t num_joints = joint_state.name.size();

		if(joint_state.position.size() < num_joints) {
			ROS_ERROR("joint_state.position.size() < num_joints");
			return;
		}
		if(joint_state.velocity.size() < num_joints) {
			ROS_ERROR("joint_state.velocity.size() < num_joints");
			return;
		}
	
		// update wheels with new data
		for(size_t i = 0; i < num_joints; ++i)
		{
	

			for(auto& wheel : m_wheels)
			{
				if(joint_state.name[i] == wheel.drive_joint_name)
				{
					// update wheel velocity
					wheel.wheel_vel = -1 * joint_state.velocity[i] * m_wheel_radius;
				}
				if(joint_state.name[i] == wheel.steer_joint_name && i>0)
				{
					// update wheel steering angle and wheel position (due to lever arm)
					wheel.set_wheel_angle(joint_state.position[i] + M_PI);
				}
				if(joint_state.name[i] == wheel.steer_joint_name && i==0)
				{
					wheel.set_wheel_angle(joint_state.position[i] +  M_PI );
				}
			}
		}
		
		// compute velocities
		m_velocity_solver->solve(m_wheels);
	}

private:
	std::mutex m_node_mutex;

	ros::NodeHandle m_node_handle;

	//ros::Publisher m_pub_odometry;
	ros::Publisher m_pub_joint_trajectory;
	ros::Publisher br_drive_pub;
	ros::Publisher bl_drive_pub;
	ros::Publisher fr_drive_pub;
	ros::Publisher fl_drive_pub;
	ros::Publisher br_caster_pub;
	ros::Publisher bl_caster_pub;
	ros::Publisher fr_caster_pub;
	ros::Publisher fl_caster_pub;


	ros::Subscriber m_sub_cmd_vel;
	ros::Subscriber m_sub_joint_state;
	ros::Subscriber m_sub_joy;

	tf::TransformBroadcaster m_tf_odom_broadcaster;

	bool m_broadcast_tf = false;
	int m_num_wheels = 0;
	int m_homeing_button = -1;
	int m_steer_reset_button = -1;
	double m_wheel_radius = 0;
	double m_wheel_lever_arm = 0;
	double m_cmd_timeout = 0;

	std::vector<OmniWheel> m_wheels;

	std::shared_ptr<OmniKinematics> m_kinematics;
	std::shared_ptr<VelocitySolver> m_velocity_solver;

	ros::Time m_last_cmd_time;
	geometry_msgs::Twist m_last_cmd_vel;
	bool is_cmd_timeout = false;

	ros::Time m_curr_odom_time;
	double m_curr_odom_x = 0;
	double m_curr_odom_y = 0;
	double m_curr_odom_yaw = 0;
	geometry_msgs::Twist m_curr_odom_twist;

};


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_omnidrive_simulation_node");

	ros::NodeHandle nh;

	double control_rate = 0;   // [1/s]
	nh.param("control_rate", control_rate, 50.0);

	// frequency of publishing states (cycle time)
	ros::Rate rate(control_rate);

	try {
		NeoOmniDriveNode node;

		while(ros::ok())
		{
			ros::spinOnce();

			node.control_step();

			rate.sleep();
		}
	} catch(std::exception& ex) {
		ROS_ERROR_STREAM("NeoOmniDriveNode: " << ex.what());
	}

	return 0;
}


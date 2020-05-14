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
#include <neo_srvs/LockPlatform.h>
#include <neo_srvs/UnlockPlatform.h>
#include <neo_srvs/ResetOmniWheels.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Joy.h>

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
		m_node_handle.param("cmd_timeout", m_cmd_timeout, 0.2);
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

		m_pub_odometry = m_node_handle.advertise<nav_msgs::Odometry>("/odom", 1);
		m_pub_joint_trajectory = m_node_handle.advertise<trajectory_msgs::JointTrajectory>("/drives/joint_trajectory", 1);

		m_sub_cmd_vel = m_node_handle.subscribe("/cmd_vel", 3, &NeoOmniDriveNode::cmd_vel_callback, this);
		m_sub_joint_state = m_node_handle.subscribe("/drives/joint_states", 10, &NeoOmniDriveNode::joint_state_callback, this);
		m_sub_joy = m_node_handle.subscribe("/joy", 1, &NeoOmniDriveNode::joy_callback, this);

		m_srv_lock_platform = m_node_handle.advertiseService("lock_platform", &NeoOmniDriveNode::lock_platform, this);
		m_srv_unlock_platform = m_node_handle.advertiseService("unlock_platform", &NeoOmniDriveNode::unlock_platform, this);
		m_srv_reset_omni_wheels = m_node_handle.advertiseService("reset_omni_wheels", &NeoOmniDriveNode::reset_omni_wheels, this);

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

		// check if platform is locked
		if(is_locked) {
			m_last_cmd_vel = geometry_msgs::Twist();	// use zero cmd_vel
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
			{
				const double drive_rot_vel = wheel.wheel_vel / m_wheel_radius;
				point.positions.push_back(0);
				point.velocities.push_back(drive_rot_vel);
			}
			{
				point.positions.push_back(wheel.wheel_angle);
				point.velocities.push_back(0);
			}
		}
		joint_trajectory->points.push_back(point);

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
				if(joint_state.name[i] == wheel.steer_joint_name)
				{
					// update wheel steering angle and wheel position (due to lever arm)
					wheel.set_wheel_angle(joint_state.position[i] + M_PI);
				}
			}
		}

		// compute velocities
		m_velocity_solver->solve(m_wheels);

		nav_msgs::Odometry::Ptr odometry = boost::make_shared<nav_msgs::Odometry>();
		odometry->header.frame_id = "odom";
		odometry->header.stamp = joint_state.header.stamp;
		odometry->child_frame_id = "base_link";

		// integrate odometry (using second order midpoint method)
		if(!m_curr_odom_time.is_zero())
		{
			const double dt = (joint_state.header.stamp - m_curr_odom_time).toSec();

			// check for valid delta time
			if(dt > 0 && dt < 1)
			{
				// compute second order midpoint velocities
				const double vel_x_mid = 0.5 * (m_velocity_solver->move_vel_x + m_curr_odom_twist.linear.x);
				const double vel_y_mid = 0.5 * (m_velocity_solver->move_vel_y + m_curr_odom_twist.linear.y);
				const double yawrate_mid = 0.5 * (m_velocity_solver->move_yawrate + m_curr_odom_twist.angular.z);

				// compute midpoint yaw angle
				const double yaw_mid = m_curr_odom_yaw + 0.5 * yawrate_mid * dt;

				// integrate position using midpoint velocities and yaw angle
				m_curr_odom_x += vel_x_mid * dt * cos(yaw_mid) + vel_y_mid * dt * -sin(yaw_mid);
				m_curr_odom_y += vel_x_mid * dt * sin(yaw_mid) + vel_y_mid * dt * cos(yaw_mid);

				// integrate yaw angle using midpoint yawrate
				m_curr_odom_yaw += yawrate_mid * dt;
			}
			else
			{
				ROS_WARN_STREAM("invalid joint state delta time: " << dt << " sec");
			}
		}
		m_curr_odom_time = joint_state.header.stamp;

		// assign odometry pose
		odometry->pose.pose.position.x = m_curr_odom_x;
		odometry->pose.pose.position.y = m_curr_odom_y;
		odometry->pose.pose.position.z = 0;
		tf::quaternionTFToMsg(tf::createQuaternionFromYaw(m_curr_odom_yaw), odometry->pose.pose.orientation);

		// assign odometry twist
		m_curr_odom_twist.linear.x = m_velocity_solver->move_vel_x;
		m_curr_odom_twist.linear.y = m_velocity_solver->move_vel_y;
		m_curr_odom_twist.linear.z = 0;
		m_curr_odom_twist.angular.x = 0;
		m_curr_odom_twist.angular.y = 0;
		m_curr_odom_twist.angular.z = m_velocity_solver->move_yawrate;
		odometry->twist.twist = m_curr_odom_twist;

		// assign bogus covariance values
		odometry->pose.covariance.assign(0.1);
		odometry->twist.covariance.assign(0.1);

		// publish odometry
		m_pub_odometry.publish(odometry);
		odometry = 0;

		// broadcast odometry
		if(m_broadcast_tf)
		{
			// compose and publish transform for tf package
			geometry_msgs::TransformStamped odom_tf;
			// compose header
			odom_tf.header.stamp = joint_state.header.stamp;
			odom_tf.header.frame_id = "odom";
			odom_tf.child_frame_id = "base_link";
			// compose data container
			odom_tf.transform.translation.x = m_curr_odom_x;
			odom_tf.transform.translation.y = m_curr_odom_y;
			odom_tf.transform.translation.z = 0;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(m_curr_odom_yaw), odom_tf.transform.rotation);

			// publish the transform
			m_tf_odom_broadcaster.sendTransform(odom_tf);
		}
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		if(m_homeing_button >= 0 && int(joy->buttons.size()) > m_homeing_button)
		{
			if(joy->buttons[m_homeing_button])
			{
				::usleep(500 * 1000);					// wait for homeing to start before sending new commands
				m_kinematics->initialize(m_wheels);		// reset stop position to home
			}
		}
		if(m_steer_reset_button >= 0 && int(joy->buttons.size()) > m_steer_reset_button)
		{
			if(joy->buttons[m_steer_reset_button])
			{
				m_kinematics->initialize(m_wheels);		// reset stop position to home
			}
		}
	}

	bool lock_platform(neo_srvs::LockPlatform::Request& request, neo_srvs::LockPlatform::Response& response)
	{
		if(		fabs(m_last_cmd_vel.linear.x) < 0.1
			&&	fabs(m_last_cmd_vel.linear.y) < 0.1
			&&	fabs(m_last_cmd_vel.angular.z) < 0.1)
		{
			is_locked = true;
			response.success = true;
			return true;
		}
		response.success = false;
		return false;
	}

	bool unlock_platform(neo_srvs::UnlockPlatform::Request& request, neo_srvs::UnlockPlatform::Response& response)
	{
		is_locked = false;
		response.success = true;
		return true;
	}

	bool reset_omni_wheels(neo_srvs::ResetOmniWheels::Request& request, neo_srvs::ResetOmniWheels::Response& response)
	{
		if(m_num_wheels >= request.steer_angles_rad.size()) {
			response.success = true;
			for(size_t i = 0; i < request.steer_angles_rad.size(); ++i)
			{
				m_kinematics->last_stop_angle[i] = request.steer_angles_rad[i];
				response.success = response.success && !m_kinematics->is_driving[i];
			}
			return true;
		}
		response.success = false;
		return false;
	}

private:
	std::mutex m_node_mutex;

	ros::NodeHandle m_node_handle;

	ros::Publisher m_pub_odometry;
	ros::Publisher m_pub_joint_trajectory;

	ros::Subscriber m_sub_cmd_vel;
	ros::Subscriber m_sub_joint_state;
	ros::Subscriber m_sub_joy;

	ros::ServiceServer m_srv_lock_platform;
	ros::ServiceServer m_srv_unlock_platform;
	ros::ServiceServer m_srv_reset_omni_wheels;

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
	bool is_locked = false;

	ros::Time m_curr_odom_time;
	double m_curr_odom_x = 0;
	double m_curr_odom_y = 0;
	double m_curr_odom_yaw = 0;
	geometry_msgs::Twist m_curr_odom_twist;

};


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_omnidrive_node");

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


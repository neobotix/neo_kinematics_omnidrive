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

#include <ros/ros.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <neo_msgs/EmergencyStopState.h>
#include <sensor_msgs/Joy.h>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>


class NeoSocketCanNode {
public:
	enum motor_state_e
	{
		ST_PRE_INITIALIZED,
		ST_OPERATION_ENABLED,
		ST_OPERATION_DISABLED,
		ST_MOTOR_FAILURE
	};

	struct motor_t
	{
		std::string joint_name;					// ROS joint name
		int32_t can_id = -1;					// motor "CAN ID"
		int32_t rot_sign = 0;					// motor rotation direction
		int32_t enc_ticks_per_rev = 0;			// encoder ticks per motor revolution
		int32_t enc_home_offset = 0;			// encoder offset for true home position
		int32_t max_vel_enc_s = 500000;			// max motor velocity in ticks/s (positive)
		int32_t max_accel_enc_s = 1000000;		// max motor acceleration in ticks/s^2 (positive)
		int32_t can_Tx_PDO1 = -1;
		int32_t can_Tx_PDO2 = -1;
		int32_t can_Rx_PDO2 = -1;
		int32_t can_Tx_SDO = -1;
		int32_t can_Rx_SDO = -1;
		double gear_ratio = 0;					// gear ratio
		double torque_constant = 0;				// conversion factor from current to torque

		motor_state_e state = ST_PRE_INITIALIZED;
		int32_t curr_enc_pos_inc = 0;			// current encoder position value in ticks
		int32_t curr_enc_vel_inc_s = 0;			// current encoder velocity value in ticks/s
		int32_t curr_status = 0;				// current status as received by SR msg
		int32_t curr_motor_failure = 0;			// current motor failure status as received by MF msg
		double curr_torque = 0;					// current measure motor torque
		ros::Time request_send_time;			// time of last status update request
		ros::Time status_recv_time;				// time of last status update received
		ros::Time update_recv_time;				// time of last sync update received
		ros::Time homing_start_time;			// time of homing start
		int homing_state = -1;					// current homing state (-2 = restart, -1 = unknown, 0 = active, 1 = finished, 2 = done)
	};

	struct module_t
	{
		motor_t drive;
		motor_t steer;

		int32_t home_dig_in = 0;				// digital input for homing switch
		double home_angle = 0;					// home steering angle in rad

		double target_wheel_vel = 0;			// current wheel velocity target in rad/s
		double target_steer_pos = 0;			// current steering target angle in rad
		double control_wheel_vel = 0;			// last commanded wheel velocity in rad/s
		double control_steer_vel = 0;			// last commanded steering velocity in rad/s
		double curr_wheel_pos = 0;				// current wheel angle in rad
		double curr_wheel_vel = 0;				// current wheel velocity in rad/s
		double curr_steer_pos = 0;				// current steering angle in rad
		double curr_steer_vel = 0;				// current steering velocity in rad/s
	};

	struct can_msg_t
	{
		int id = -1;
		int length = 0;
		uint8_t data[8] = {};
	};

	NeoSocketCanNode()
	{
		if(!m_node_handle.getParam("control_rate", m_control_rate)) {
			throw std::logic_error("missing control_rate param");
		}
		if(!m_node_handle.getParam("num_wheels", m_num_wheels)) {
			throw std::logic_error("missing num_wheels param");
		}
		if(!m_node_handle.getParam("can_iface", m_can_iface)) {
			throw std::logic_error("missing can_iface param");
		}
		m_node_handle.param("request_status_divider", m_request_status_divider, 10);
		m_node_handle.param("heartbeat_divider", m_heartbeat_divider, 10);
		m_node_handle.param("motor_group_id", m_motor_group_id, -1);
		m_node_handle.param("motor_timeout", m_motor_timeout, 1.);
		m_node_handle.param("home_vel", m_home_vel, -1.);
		m_node_handle.param("steer_gain", m_steer_gain, 1.);
		m_node_handle.param("steer_lookahead", m_steer_lookahead, 0.1);
		m_node_handle.param("steer_low_pass", m_steer_low_pass, 0.5);
		m_node_handle.param("max_steer_vel", m_max_steer_vel, 10.);
		m_node_handle.param("drive_low_pass", m_drive_low_pass, 0.5);
		m_node_handle.param("motor_delay", m_motor_delay, 0.);
		m_node_handle.param("trajectory_timeout", m_trajectory_timeout, 0.1);
		m_node_handle.param("auto_home", m_auto_home, true);
		m_node_handle.param("measure_torque", m_measure_torque, false);
		m_node_handle.param("homeing_button", m_homeing_button, 0);

		if(m_motor_group_id >= 0) {
			ROS_INFO_STREAM("Using motor group id: " << m_motor_group_id);
			m_motor_group_id += 0x300;
		}

		if(m_num_wheels < 1) {
			throw std::logic_error("invalid num_wheels param");
		}
		m_wheels.resize(m_num_wheels);

		for(int i = 0; i < m_num_wheels; ++i)
		{
			if(!m_node_handle.getParam("drive" + std::to_string(i) + "/can_id", m_wheels[i].drive.can_id)) {
				throw std::logic_error("can_id param missing for drive motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/can_id", m_wheels[i].steer.can_id)) {
				throw std::logic_error("can_id param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("drive" + std::to_string(i) + "/joint_name", m_wheels[i].drive.joint_name)) {
				throw std::logic_error("joint_name param missing for drive motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/joint_name", m_wheels[i].steer.joint_name)) {
				throw std::logic_error("joint_name param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("drive" + std::to_string(i) + "/rot_sign", m_wheels[i].drive.rot_sign)) {
				throw std::logic_error("rot_sign param missing for drive motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/rot_sign", m_wheels[i].steer.rot_sign)) {
				throw std::logic_error("rot_sign param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("drive" + std::to_string(i) + "/gear_ratio", m_wheels[i].drive.gear_ratio)) {
				throw std::logic_error("gear_ratio param missing for drive motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/gear_ratio", m_wheels[i].steer.gear_ratio)) {
				throw std::logic_error("gear_ratio param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("drive" + std::to_string(i) + "/enc_ticks_per_rev", m_wheels[i].drive.enc_ticks_per_rev)) {
				throw std::logic_error("enc_ticks_per_rev param missing for drive motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/enc_ticks_per_rev", m_wheels[i].steer.enc_ticks_per_rev)) {
				throw std::logic_error("enc_ticks_per_rev param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/home_angle", m_wheels[i].home_angle)) {
				throw std::logic_error("home_angle param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/home_dig_in", m_wheels[i].home_dig_in)) {
				throw std::logic_error("home_dig_in param missing for steering motor" + std::to_string(i));
			}
			if(!m_node_handle.getParam("steer" + std::to_string(i) + "/enc_home_offset", m_wheels[i].steer.enc_home_offset)) {
				throw std::logic_error("enc_home_offset param missing for steering motor" + std::to_string(i));
			}
			m_node_handle.param("drive" + std::to_string(i) + "/torque_constant", m_wheels[i].drive.torque_constant, 0.);
			m_node_handle.param("steer" + std::to_string(i) + "/torque_constant", m_wheels[i].steer.torque_constant, 0.);

			m_wheels[i].home_angle = M_PI * m_wheels[i].home_angle / 180.;
		}

		m_pub_joint_state = m_node_handle.advertise<sensor_msgs::JointState>("/drives/joint_states", 10);
		m_pub_joint_state_raw = m_node_handle.advertise<sensor_msgs::JointState>("/drives/joint_states_raw", 10);

		m_sub_joint_trajectory = m_node_handle.subscribe("/drives/joint_trajectory", 1, &NeoSocketCanNode::joint_trajectory_callback, this);
		m_sub_emergency_stop = m_node_handle.subscribe("emergency_stop_state", 1, &NeoSocketCanNode::emergency_stop_callback, this);
		m_sub_joy = m_node_handle.subscribe("/joy", 1, &NeoSocketCanNode::joy_callback, this);

		m_can_thread = std::thread(&NeoSocketCanNode::receive_loop, this);
	}

	void update()
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		const ros::Time now = ros::Time::now();

		// check for motor timeouts
		for(auto& wheel : m_wheels)
		{
			check_motor_timeout(wheel.drive, now);
			check_motor_timeout(wheel.steer, now);
		}

		// check if we should stop motion
		if(!all_motors_operational())
		{
			stop_motion();
		}

		// check for motor reset done
		if(is_motor_reset)
		{
			if(all_motors_operational())
			{
				ROS_INFO_STREAM("All motors operational!");
				is_motor_reset = false;
			}
		}

		// check if we are stopped
		is_stopped = true;
		for(const auto& wheel : m_wheels) {
			if(std::abs(wheel.drive.curr_enc_vel_inc_s) > 100) {
				is_stopped = false;
			}
		}

		// check if we should start homing
		if(m_auto_home && !is_all_homed && m_sync_counter > 100)
		{
			start_homing();
		}

		// check if homing done
		if(is_homing_active)
		{
			if(!all_motors_operational())
			{
				ROS_ERROR_STREAM("Homing has been interrupted!");
				is_homing_active = false;
			}
			else if(check_homing_done())
			{
				finish_homing();
				ROS_INFO_STREAM("Homing successful!");
			}
			else
			{
				// send status request
				for(auto& wheel : m_wheels)
				{
					canopen_query(wheel.steer, 'H', 'M', 1);
				}
			}
		}

		// check if we should reset steering
		if(is_steer_reset_active && all_motors_operational())
		{
			bool is_all_reached = true;

			for(auto& wheel : m_wheels)
			{
				wheel.target_wheel_vel = 0;			// stop driving
				wheel.target_steer_pos = 0;			// set target steering

				if(fabs(angles::normalize_angle(wheel.curr_steer_pos)) > 0.01)
				{
					is_all_reached = false;
				}
			}

			if(is_all_reached)
			{
				ROS_INFO_STREAM("Steering reset successful!");
				is_steer_reset_active = false;
			}
		}

		// steering and motion control
		if(is_all_homed && all_motors_operational())
		{
			// check for input timeout
			if((now - m_last_trajectory_time).toSec() > m_trajectory_timeout)
			{
				if(!is_trajectory_timeout && !m_last_trajectory_time.isZero()) {
					ROS_WARN_STREAM("joint_trajectory input timeout! Stopping now.");
				}
				is_trajectory_timeout = true;
			}
			else {
				is_trajectory_timeout = false;
			}

			for(auto& wheel : m_wheels)
			{
				if(is_trajectory_timeout) {
					wheel.target_wheel_vel = 0;		// stop when input timed out
				}

				const double future_steer_pos = wheel.curr_steer_pos + wheel.curr_steer_vel * m_steer_lookahead;
				const double delta_rad = angles::shortest_angular_distance(wheel.target_steer_pos, future_steer_pos);
				const double control_vel = -1 * delta_rad * m_steer_gain;

				wheel.control_wheel_vel = wheel.target_wheel_vel * m_drive_low_pass + wheel.control_wheel_vel * (1 - m_drive_low_pass);
				wheel.control_steer_vel = control_vel * m_steer_low_pass + wheel.control_steer_vel * (1 - m_steer_low_pass);

				motor_set_vel(wheel.drive, wheel.control_wheel_vel);
				motor_set_vel(wheel.steer, fmin(fmax(wheel.control_steer_vel, -m_max_steer_vel), m_max_steer_vel));
			}
			begin_motion();
		}

		// check for update timeout
		if(m_last_update_time < m_last_sync_time)
		{
			if(is_all_homed) {
				ROS_DEBUG_STREAM("Sync update timeout!");
			}
		}

		// request current motor values
		{
			can_msg_t msg;
			msg.id  = 0x80;
			msg.length = 0;
			can_transmit(msg);
		}

		m_last_sync_time = ros::Time::now();
		m_sync_counter++;

		// measure torque if enabled
		if(m_measure_torque)
		{
			if(m_motor_group_id >= 0) {
				canopen_query(m_motor_group_id, 'I', 'Q', 0);	// query motor current
			}
			else {
				for(auto& wheel : m_wheels) {
					canopen_query(wheel.drive, 'I', 'Q', 0);	// query motor current
				}
			}
		}

		// check if we need to request status
		{
			int i = 0;
			for(auto& wheel : m_wheels)
			{
				if((m_sync_counter + i + 0) % m_request_status_divider == 0) {
					request_status(wheel.drive);		// request status update
				}
				if((m_sync_counter + i + 1) % m_request_status_divider == 0) {
					request_status(wheel.steer);		// request status update
				}
				i += 2;
			}
		}

		// check if we need to send a heartbeat
		if((m_sync_counter + 2 * m_num_wheels) % m_heartbeat_divider == 0)
		{
			can_msg_t msg;				// send heartbeat message
			msg.id  = 0x700;
			msg.length = 5;
			can_transmit(msg);
		}
	}

	void initialize()
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		ROS_INFO_STREAM("Initializing ...");

		// wait for CAN socket to be available
		m_wait_for_can_sock = true;

		// reset states
		for(auto& wheel : m_wheels)
		{
			set_motor_can_id(wheel.drive, wheel.drive.can_id);
			set_motor_can_id(wheel.steer, wheel.steer.can_id);
		}
		is_all_homed = false;
		is_homing_active = false;
		is_steer_reset_active = false;

		// start network
		{
			can_msg_t msg;
			msg.id = 0;
			msg.length = 2;
			msg.data[0] = 1;
			msg.data[1] = 0;
			can_transmit(msg);
		}
		can_sync();

		::usleep(100 * 1000);

		all_motors_off();

		stop_motion();

		disable_watchdog_all();

		// set modulo to one wheel revolution (to preserve absolute position for homed motors)
		for(auto& wheel : m_wheels)
		{
			set_motor_modulo(wheel.drive, 1);
			set_motor_modulo(wheel.steer, 1);
		}
		can_sync();

		// set motion control to velocity mode first
		for(auto& wheel : m_wheels)
		{
			set_motion_vel_ctrl(wheel.drive);
			set_motion_vel_ctrl(wheel.steer);
		}
		can_sync();

		// set position counter to zero
		for(auto& wheel : m_wheels)
		{
			reset_pos_counter(wheel.drive);
			reset_pos_counter(wheel.steer);
		}
		can_sync();

		// ---------- set PDO mapping
		// Mapping of TPDO1:
		// - position (byte 0 to 3)
		// - velocity (byte 4 to 7)
		for(auto& wheel : m_wheels)
		{
			configure_PDO_mapping(wheel.drive);
			configure_PDO_mapping(wheel.steer);
		}
		can_sync();

		all_motors_on();

		request_status_all();

		ROS_INFO_STREAM("Initializing done.");
	}

	void shutdown()
	{
		{
			std::lock_guard<std::mutex> lock(m_node_mutex);

			// disable waiting for CAN socket, since we are shutting down
			m_wait_for_can_sock = false;

			try {
				stop_motion();
				can_sync();
				all_motors_off();
				can_sync();
			}
			catch(...) {
				// ignore
			}
		}
		{
			std::lock_guard<std::mutex> lock(m_can_mutex);
			do_run = false;
			if(m_can_sock >= 0) {
				::close(m_can_sock);
				m_can_sock = -1;
			}
		}
		if(m_can_thread.joinable()) {
			m_can_thread.join();
		}
	}

private:
	void joint_trajectory_callback(const trajectory_msgs::JointTrajectory& joint_trajectory)
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		// check if we are ready for normal operation
		if(!is_all_homed || is_steer_reset_active) {
			return;
		}

		// check if we are fully operational
		if(!all_motors_operational()) {
			return;
		}

		// check proper message
		if(joint_trajectory.points.size() < 1) {
			ROS_WARN_STREAM("Invalid JointTrajectory message!");
			stop_motion();
			return;
		}

		std::vector<double> wheel_vel(m_num_wheels);
		std::vector<double> wheel_angle(m_num_wheels);
		std::vector<int> got_value(m_num_wheels);

		for(size_t i = 0; i < joint_trajectory.joint_names.size(); ++i)
		{
			for(int k = 0; k < m_num_wheels; ++k)
			{
				if(joint_trajectory.joint_names[i] == m_wheels[k].drive.joint_name) {
					if(joint_trajectory.points[0].velocities.size() > i) {
						wheel_vel[k] = joint_trajectory.points[0].velocities[i];
						got_value[k] |= 1;
					}
				}
				if(joint_trajectory.joint_names[i] == m_wheels[k].steer.joint_name) {
					if(joint_trajectory.points[0].positions.size() > i) {
						wheel_angle[k] = joint_trajectory.points[0].positions[i];
						got_value[k] |= 2;
					}
				}
			}
		}

		// check that we have new values for every motor
		for(int i = 0; i < m_num_wheels; ++i) {
			if(got_value[i] != 3) {
				ROS_WARN_STREAM("Invalid JointTrajectory message!");
				stop_motion();
				return;
			}
		}

		// apply new commands
		for(int i = 0; i < m_num_wheels; ++i)
		{
			m_wheels[i].target_wheel_vel = wheel_vel[i];
			m_wheels[i].target_steer_pos = wheel_angle[i] - m_wheels[i].home_angle;
		}
		m_last_trajectory_time = ros::Time::now();
	}

	void emergency_stop_callback(const neo_msgs::EmergencyStopState::ConstPtr& state)
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		if(is_em_stop && state->emergency_state == neo_msgs::EmergencyStopState::EMFREE)
		{
			ROS_INFO_STREAM("Reactivating motors ...");

			// reset states
			for(auto& wheel : m_wheels)
			{
				wheel.drive.state = ST_PRE_INITIALIZED;
				wheel.steer.state = ST_PRE_INITIALIZED;
			}
			is_motor_reset = true;

			all_motors_on();			// re-activate the motors

			request_status_all();		// request new status
		}

		is_em_stop = state->emergency_state != neo_msgs::EmergencyStopState::EMFREE;
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
	{
		std::lock_guard<std::mutex> lock(m_node_mutex);

		if(m_homeing_button >= 0 && int(joy->buttons.size()) > m_homeing_button)
		{
			if(joy->buttons[m_homeing_button])
			{
				start_homing();
			}
		}
	}

	void check_motor_timeout(motor_t& motor, ros::Time now)
	{
		if(motor.status_recv_time < motor.request_send_time
			&& (now - motor.request_send_time).toSec() > m_motor_timeout)
		{
			if(motor.state != ST_MOTOR_FAILURE) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor status timeout!");
			}
			motor.state = ST_MOTOR_FAILURE;
		}
	}

	bool all_motors_operational() const
	{
		for(auto& wheel : m_wheels)
		{
			if(wheel.drive.state != ST_OPERATION_ENABLED || wheel.steer.state != ST_OPERATION_ENABLED) {
				return false;
			}
		}
		return !is_em_stop;
	}

	void start_homing()
	{
		if(is_homing_active || !is_stopped || !all_motors_operational()) {
			return;
		}
		ROS_INFO_STREAM("Start homing procedure ...");

		stop_motion();

		disable_watchdog_all();

		for(auto& wheel : m_wheels)
		{
			// disarm homing
			canopen_set_int(wheel.steer, 'H', 'M', 1, 0);
			can_sync();

			// configure homing sequences
			// setting the value such that increment counter resets after the homing event occurs
			canopen_set_int(wheel.steer, 'H', 'M', 2, wheel.steer.enc_home_offset);
			can_sync();

			// choosing channel/switch on which controller has to listen for change of homing event(high/low/falling/rising)
			canopen_set_int(wheel.steer, 'H', 'M', 3, wheel.home_dig_in);
			can_sync();

			// choose the action that the controller shall perform after the homing event occurred
			// HM[4] = 0 : after Event stop immediately
			// HM[4] = 2 : do nothing
			canopen_set_int(wheel.steer, 'H', 'M', 4, 2);
			can_sync();

			// choose the setting of the position counter (i.e. to the value defined in 2.a) after the homing event occured
			// HM[5] = 0 : absolute setting of position counter: PX = HM[2]
			canopen_set_int(wheel.steer, 'H', 'M', 5, 0);
			can_sync();
		}

		// start turning motors
		for(auto& wheel : m_wheels)
		{
			motor_set_vel(wheel.drive, 0);
			motor_set_vel(wheel.steer, m_home_vel);
		}
		can_sync();

		begin_motion();

		// arm homing
		for(auto& wheel : m_wheels)
		{
			arm_homing(wheel.steer);
		}
		can_sync();

		is_all_homed = false;
		is_homing_active = true;
	}

	void arm_homing(motor_t& motor)
	{
		motor.homing_state = -1;		// reset state
		motor.homing_start_time = ros::Time::now();

		canopen_set_int(motor, 'H', 'M', 1, 1);		// arm homeing
	}

	bool check_homing_done()
	{
		const ros::Time now = ros::Time::now();

		for(auto& wheel : m_wheels)
		{
			// check if we can stop wheels
			if(wheel.steer.homing_state == 1)
			{
				stop_motion(wheel.steer);			// stop when finished homeing
				wheel.steer.homing_state = 2;
			}

			// check for restart
			if(wheel.steer.homing_state == -2)
			{
				arm_homing(wheel.steer);
			}

			// check for timeout
			if((now - wheel.steer.homing_start_time).toSec() > 20)
			{
				ROS_WARN_STREAM("Homeing timeout on motor " << wheel.steer.joint_name << ", restarting ...");				

				arm_homing(wheel.steer);
			}
		}

		// check if all done
		for(auto& wheel : m_wheels)
		{
			if(wheel.steer.homing_state != 2) {
				return false;
			}
		}
		return true;
	}

	void finish_homing()
	{
		stop_motion();

		// activate watchdog
		for(auto& wheel : m_wheels)
		{
			configure_watchdog(wheel.drive);
			configure_watchdog(wheel.steer);
		}

		is_all_homed = true;
		is_homing_active = false;
		is_steer_reset_active = true;
		m_last_trajectory_time = ros::Time();
	}

	void set_motor_can_id(motor_t& motor, int id)
	{
		motor.can_id = id;
		motor.can_Tx_PDO1 = id + 0x180;
		// motor.can_Rx_PDO1 = id + 0x200;
		motor.can_Tx_PDO2 = id + 0x280;
		motor.can_Rx_PDO2 = id + 0x300;
		motor.can_Tx_SDO = id + 0x580;
		motor.can_Rx_SDO = id + 0x600;
	}

	void configure_PDO_mapping(const motor_t& motor)
	{
		// stop all emissions of TPDO1
		canopen_SDO_download(motor, 0x1A00, 0, 0);

		// position 4 byte of TPDO1
		canopen_SDO_download(motor, 0x1A00, 1, 0x60640020);

		// velocity 4 byte of TPDO1
		canopen_SDO_download(motor, 0x1A00, 2, 0x60690020);

		// transmission type "synch"
		canopen_SDO_download(motor, 0x1800, 2, 1);

		// activate mapped objects
		canopen_SDO_download(motor, 0x1A00, 0, 2);

		can_sync();
	}

	void configure_watchdog(const motor_t& motor)
	{
		// configure to fail after missing 3 heartbeats
		const int heartbeat_time_ms = 4 * 1000 * m_heartbeat_divider / m_control_rate;
		const int pc_node_id = 0x00;

		// consumer (PC) heartbeat time
		canopen_SDO_download(motor, 0x1016, 1, (pc_node_id << 16) | heartbeat_time_ms);

		// error behavior after failure: 0=pre-operational, 1=no state change, 2=stopped"
		canopen_SDO_download(motor, 0x1029, 1, 2);

		// motor behavior after heartbeat failure: "quick stop"
		canopen_SDO_download(motor, 0x6007, 0, 3);

		// activate emergency events: "heartbeat event"
		// Object 0x2F21 = "Emergency Events" which cause an Emergency Message
		// Bit 3 is responsible for Heartbeart-Failure.--> Hex 0x08
		canopen_SDO_download(motor, 0x2F21, 0, 0x08);

		can_sync();
	}

	void disable_watchdog(const motor_t& motor)
	{
		// Motor action after Hearbeat-Error: No Action
		canopen_SDO_download(motor, 0x6007, 0, 0);

		// Error Behavior: No state change
		canopen_SDO_download(motor, 0x1029, 1, 1);

		// Deacivate emergency events: "heartbeat event"
		// Object 0x2F21 = "Emergency Events" which cause an Emergency Message
		// Bit 3 is responsible for Heartbeart-Failure.
		canopen_SDO_download(motor, 0x2F21, 0, 0x00);

		can_sync();
	}

	void disable_watchdog_all()
	{
		for(auto& wheel : m_wheels)
		{
			disable_watchdog(wheel.drive);
			disable_watchdog(wheel.steer);
		}
		can_sync();
	}

	void set_motor_modulo(const motor_t& motor, int32_t num_wheel_rev)
	{
		const int32_t ticks_per_rev = motor.enc_ticks_per_rev * motor.gear_ratio;
		canopen_set_int(motor, 'X', 'M', 1, -1 * num_wheel_rev * ticks_per_rev);
		canopen_set_int(motor, 'X', 'M', 2, num_wheel_rev * ticks_per_rev);

		can_sync();
	}

	void reset_pos_counter(const motor_t& motor)
	{
		canopen_set_int(motor, 'P', 'X', 0, 0);
	}

	void request_status(motor_t& motor)
	{
		canopen_query(motor, 'S', 'R', 0);
		motor.request_send_time = ros::Time::now();
	}

	void request_status_all()
	{
		if(m_motor_group_id >= 0)
		{
			canopen_query(m_motor_group_id, 'S', 'R', 0);

			for(auto& wheel : m_wheels) {
				wheel.drive.request_send_time = ros::Time::now();
				wheel.steer.request_send_time = ros::Time::now();
			}
		}
		else {
			for(auto& wheel : m_wheels) {
				request_status(wheel.drive);
				request_status(wheel.steer);
			}
			can_sync();
		}
	}

	void motor_on(motor_t& motor)
	{
		canopen_set_int(motor, 'M', 'O', 0, 1);
	}

	void motor_off(motor_t& motor)
	{
		canopen_set_int(motor, 'M', 'O', 0, 0);
		motor.state = ST_PRE_INITIALIZED;
	}

	void all_motors_on()
	{
		if(m_motor_group_id >= 0) {
			canopen_set_int(m_motor_group_id, 'M', 'O', 0, 1);
		}
		else {
			for(auto& wheel : m_wheels) {
				motor_on(wheel.drive);
				motor_on(wheel.steer);
			}
			can_sync();
		}
	}

	void all_motors_off()
	{
		if(m_motor_group_id >= 0)
		{
			canopen_set_int(m_motor_group_id, 'M', 'O', 0, 0);

			for(auto& wheel : m_wheels) {
				wheel.drive.state = ST_PRE_INITIALIZED;
				wheel.steer.state = ST_PRE_INITIALIZED;
			}
		}
		else {
			for(auto& wheel : m_wheels) {
				motor_off(wheel.drive);
				motor_off(wheel.steer);
			}
			can_sync();
		}
		is_motor_reset = true;
	}

	void set_motion_vel_ctrl(const motor_t& motor)
	{
		// switch Unit Mode
		canopen_set_int(motor, 'U', 'M', 0, 2);

		// set profile mode (only if Unit Mode = 2)
		canopen_set_int(motor, 'P', 'M', 0, 1);

		// set maximum acceleration to X Incr/s^2
		canopen_set_int(motor, 'A', 'C', 0, motor.max_accel_enc_s);

		// set maximum decceleration to X Incr/s^2
		canopen_set_int(motor, 'D', 'C', 0, motor.max_accel_enc_s);

		can_sync();
	}

	void set_motion_pos_ctrl(const motor_t& motor)
	{
		// switch Unit Mode
		canopen_set_int(motor, 'U', 'M', 0, 5);

		// set Target Radius to X Increments
		canopen_set_int(motor, 'T', 'R', 1, 15);		// TODO: add ROS param

		// set Target Time to X ms
		canopen_set_int(motor, 'T', 'R', 2, 100);		// TODO: add ROS param

		// set maximum acceleration to X Incr/s^2
		canopen_set_int(motor, 'A', 'C', 0, motor.max_accel_enc_s);

		// set maximum decceleration to X Incr/s^2
		canopen_set_int(motor, 'D', 'C', 0, motor.max_accel_enc_s);

		can_sync();
	}

	void begin_motion()
	{
		if(m_motor_group_id >= 0) {
			canopen_query(m_motor_group_id, 'B', 'G', 0);
		}
		else {
			for(const auto& wheel : m_wheels)
			{
				canopen_query(wheel.drive, 'B', 'G', 0);
				canopen_query(wheel.steer, 'B', 'G', 0);
			}
			can_sync();
		}
	}

	void stop_motion(const motor_t& motor)
	{
		canopen_query(motor, 'S', 'T', 0);
	}

	void stop_motion()
	{
		if(m_motor_group_id >= 0) {
			canopen_query(m_motor_group_id, 'S', 'T', 0);
		}
		else {
			for(const auto& wheel : m_wheels)
			{
				stop_motion(wheel.drive);
				stop_motion(wheel.steer);
			}
			can_sync();
		}
	}

	void motor_set_vel(const motor_t& motor, double rot_vel_rad_s)
	{
		const double motor_vel_rev_s = motor.gear_ratio * rot_vel_rad_s / (2 * M_PI);
		const int32_t motor_vel_inc_s = motor.rot_sign * int(motor_vel_rev_s * motor.enc_ticks_per_rev);
		const int32_t lim_motor_vel_inc_s = std::min(std::max(motor_vel_inc_s, -motor.max_vel_enc_s), motor.max_vel_enc_s);

		canopen_set_int(motor, 'J', 'V', 0, lim_motor_vel_inc_s);
	}

	void motor_set_pos_abs(const motor_t& motor, double angle_rad)
	{
		const double motor_pos_rev = motor.gear_ratio * angle_rad / (2 * M_PI);
		const int32_t motor_pos_inc = motor.rot_sign * int(motor_pos_rev * motor.enc_ticks_per_rev);

		canopen_set_int(motor, 'P', 'A', 0, motor_pos_inc);
	}

	void canopen_query(const motor_t& motor, char cmd_char_1, char cmd_char_2, int32_t index)
	{
		canopen_query(motor.can_Rx_PDO2, cmd_char_1, cmd_char_2, index);
	}

	void canopen_query(int id, char cmd_char_1, char cmd_char_2, int32_t index)
	{
		can_msg_t msg;
		msg.id = id;
		msg.length = 4;
		msg.data[0] = cmd_char_1;
		msg.data[1] = cmd_char_2;
		msg.data[2] = index;
		msg.data[3] = (index >> 8) & 0x3F;  // The two MSB must be 0. Cf. DSP 301 Implementation guide p. 39.
		can_transmit(msg);
	}

	void canopen_set_int(const motor_t& motor, char cmd_char_1, char cmd_char_2, int32_t index, int32_t data)
	{
		canopen_set_int(motor.can_Rx_PDO2, cmd_char_1, cmd_char_2, index, data);
	}

	void canopen_set_int(int id, char cmd_char_1, char cmd_char_2, int32_t index, int32_t data)
	{
		can_msg_t msg;
		msg.id = id;
		msg.length = 8;
		msg.data[0] = cmd_char_1;
		msg.data[1] = cmd_char_2;
		msg.data[2] = index;
		msg.data[3] = (index >> 8) & 0x3F;  // The two MSB must be 0. Cf. DSP 301 Implementation guide p. 39.
		msg.data[4] = data;
		msg.data[5] = data >> 8;
		msg.data[6] = data >> 16;
		msg.data[7] = data >> 24;
		can_transmit(msg);
	}

	void canopen_SDO_download(const motor_t& motor, int32_t obj_index, int32_t obj_sub_index, int32_t data)
	{
		const int32_t ciInitDownloadReq = 0x20;
		const int32_t ciNrBytesNoData = 0x00;
		const int32_t ciExpedited = 0x02;
		const int32_t ciDataSizeInd = 0x01;

		can_msg_t msg;
		msg.id = motor.can_Rx_SDO;
		msg.length = 8;
		msg.data[0] = ciInitDownloadReq | (ciNrBytesNoData << 2) | ciExpedited | ciDataSizeInd;
		msg.data[1] = obj_index;
		msg.data[2] = obj_index >> 8;
		msg.data[3] = obj_sub_index;
		msg.data[4] = data;
		msg.data[5] = data >> 8;
		msg.data[6] = data >> 16;
		msg.data[7] = data >> 24;
		can_transmit(msg);
	}

	void can_transmit(const can_msg_t& msg)
	{
		::can_frame out = {};
		out.can_id = msg.id;
		out.can_dlc = msg.length;
		for(int i = 0; i < msg.length; ++i) {
			out.data[i] = msg.data[i];
		}

		// wait for socket to be ready for writing
		if(m_wait_for_can_sock) {
			std::unique_lock<std::mutex> lock(m_can_mutex);
			while(do_run && m_can_sock < 0) {
				m_can_condition.wait(lock);
			}
		}
		if(!do_run) {
			throw std::runtime_error("shutdown");
		}

		// send msg
		{
			const auto res = ::write(m_can_sock, &out, sizeof(out));
			if(res < 0) {
				throw std::runtime_error("write() failed with: " + std::string(strerror(errno)));
			}
			if(res != sizeof(out))
			{
				// re-open socket
				::shutdown(m_can_sock, SHUT_RDWR);
				throw std::logic_error("write() buffer overflow!");
			}
		}
	}

	/*
	 * Waits till all msgs are sent on the bus.
	 */
	void can_sync()
	{
		::usleep(10000);		// workaround, sleep for around 10 msgs
	}

	/*
	 * Processes incoming CAN msgs.
	 * Called by receive_loop() only!
	 */
	void handle(const can_msg_t& msg)
	{
		size_t num_motor_updates = 0;

		for(auto& wheel : m_wheels)
		{
			if(msg.id == wheel.drive.can_Tx_PDO1) {
				handle_PDO1(wheel.drive, msg);
			}
			if(msg.id == wheel.steer.can_Tx_PDO1) {
				handle_PDO1(wheel.steer, msg);
			}
			if(msg.id == wheel.drive.can_Tx_PDO2) {
				handle_PDO2(wheel.drive, msg);
			}
			if(msg.id == wheel.steer.can_Tx_PDO2) {
				handle_PDO2(wheel.steer, msg);
			}

			// re-compute wheel values
			if(wheel.drive.update_recv_time > m_last_sync_time)
			{
				wheel.curr_wheel_pos = calc_wheel_pos(wheel.drive);
				wheel.curr_wheel_vel = calc_wheel_vel(wheel.drive);
				num_motor_updates++;
			}
			if(wheel.steer.update_recv_time > m_last_sync_time)
			{
				wheel.curr_steer_pos = calc_wheel_pos(wheel.steer);
				wheel.curr_steer_vel = calc_wheel_vel(wheel.steer);
				num_motor_updates++;
			}
		}

		// check if we have all data for next update
		if(num_motor_updates >= m_wheels.size() * 2 && m_last_update_time < m_last_sync_time)
		{
			const ros::Time now = ros::Time::now();
			const ros::Time timestamp = m_last_sync_time + ros::Duration(m_motor_delay);
			publish_joint_states(timestamp);
			publish_joint_states_raw(timestamp);
			m_last_update_time = now;
		}
	}

	void publish_joint_states(ros::Time timestamp)
	{
		sensor_msgs::JointState::Ptr joint_state = boost::make_shared<sensor_msgs::JointState>();
		joint_state->header.stamp = timestamp;

		for(auto& wheel : m_wheels)
		{
			joint_state->name.push_back(wheel.drive.joint_name);
			joint_state->name.push_back(wheel.steer.joint_name);
			joint_state->position.push_back(wheel.curr_wheel_pos);
			joint_state->position.push_back(wheel.curr_steer_pos + wheel.home_angle);
			joint_state->velocity.push_back(wheel.curr_wheel_vel);
			joint_state->velocity.push_back(wheel.curr_steer_vel);
			joint_state->effort.push_back(wheel.drive.curr_torque);
			joint_state->effort.push_back(wheel.steer.curr_torque);
		}
		m_pub_joint_state.publish(joint_state);
	}

	void publish_joint_states_raw(ros::Time timestamp)
	{
		sensor_msgs::JointState::Ptr joint_state = boost::make_shared<sensor_msgs::JointState>();
		joint_state->header.stamp = timestamp;

		for(auto& wheel : m_wheels)
		{
			joint_state->name.push_back(wheel.drive.joint_name);
			joint_state->name.push_back(wheel.steer.joint_name);
			joint_state->position.push_back(wheel.drive.curr_enc_pos_inc);
			joint_state->position.push_back(wheel.steer.curr_enc_pos_inc);
			joint_state->velocity.push_back(wheel.drive.curr_enc_vel_inc_s);
			joint_state->velocity.push_back(wheel.steer.curr_enc_vel_inc_s);
			joint_state->effort.push_back(wheel.drive.curr_torque);
			joint_state->effort.push_back(wheel.steer.curr_torque);
		}
		m_pub_joint_state_raw.publish(joint_state);
	}

	double calc_wheel_pos(motor_t& motor) const
	{
		return 2 * M_PI * double(motor.rot_sign * motor.curr_enc_pos_inc)
				/ motor.enc_ticks_per_rev / motor.gear_ratio;
	}

	double calc_wheel_vel(motor_t& motor) const
	{
		return 2 * M_PI * double(motor.rot_sign * motor.curr_enc_vel_inc_s)
				/ motor.enc_ticks_per_rev / motor.gear_ratio;
	}

	int32_t read_int32(const can_msg_t& msg, int offset) const
	{
		if(offset < 0 || offset > 4) {
			throw std::logic_error("invalid offset");
		}
		int32_t value = 0;
		::memcpy(&value, msg.data + offset, 4);
		return value;
	}

	float read_float(const can_msg_t& msg, int offset) const
	{
		if(offset < 0 || offset > 4) {
			throw std::logic_error("invalid offset");
		}
		float value = 0;
		::memcpy(&value, msg.data + offset, 4);
		return value;
	}

	void handle_PDO1(motor_t& motor, const can_msg_t& msg)
	{
		motor.curr_enc_pos_inc = read_int32(msg, 0);
		motor.curr_enc_vel_inc_s = read_int32(msg, 4);
		motor.update_recv_time = ros::Time::now();
	}

	void handle_PDO2(motor_t& motor, const can_msg_t& msg)
	{
		if(msg.data[0] == 'S' && msg.data[1] == 'R')
		{
			const auto prev_status = motor.curr_status;
			motor.curr_status = read_int32(msg, 4);
			evaluate_status(motor, prev_status);
			motor.status_recv_time = ros::Time::now();
		}
		if(msg.data[0] == 'M' && msg.data[1] == 'F')
		{
			const auto prev_status = motor.curr_motor_failure;
			motor.curr_motor_failure = read_int32(msg, 4);
			evaluate_motor_failure(motor, prev_status);
		}
		if(msg.data[0] == 'H' && msg.data[1] == 'M')
		{
			if(msg.data[4] == 0)					// check if motor says homing finished
			{
				if(motor.homing_state == 0)
				{
					if((ros::Time::now() - motor.homing_start_time).toSec() > 0.5)
					{
						motor.homing_state = 1;		// only go to finish after active for some time
					}
					else {
						motor.homing_state = -2;	// restart, since it finished too soon
					}
				}
				else if(motor.homing_state == -1)
				{
					motor.homing_state = -2;		// restart, since it was never active
				}
			}
			else {
				motor.homing_state = 0;				// homing active
			}
		}
		if(msg.data[0] == 'I' && msg.data[1] == 'Q')
		{
			motor.curr_torque = read_float(msg, 4) * motor.torque_constant;
		}
	}

	void evaluate_status(motor_t& motor, int32_t prev_status)
	{
		if(motor.curr_status & 1)
		{
			if(motor.curr_status != prev_status)
			{
				if((motor.curr_status & 0xE) == 2) {
					ROS_ERROR_STREAM(motor.joint_name << ": drive error under voltage");
				}
				else if((motor.curr_status & 0xE) == 4) {
					ROS_ERROR_STREAM(motor.joint_name << ": drive error over voltage");
				}
				else if((motor.curr_status & 0xE) == 10) {
					ROS_ERROR_STREAM(motor.joint_name << ": drive error short circuit");
				}
				else if((motor.curr_status & 0xE) == 12) {
					ROS_ERROR_STREAM(motor.joint_name << ": drive error over-heating");
				}
				else {
					ROS_ERROR_STREAM(motor.joint_name << ": unknown failure: " << (motor.curr_status & 0xE));
				}
			}

			// request detailed description of failure
			canopen_query(motor, 'M', 'F', 0);

			motor.state = ST_MOTOR_FAILURE;
		}
		else if(motor.curr_status & (1 << 6))
		{
			// general failure
			if(motor.curr_status != prev_status)
			{
				ROS_ERROR_STREAM(motor.joint_name << ": failure latched");
			}

			// request detailed description of failure
			canopen_query(motor, 'M', 'F', 0);

			motor.state = ST_MOTOR_FAILURE;
		}
		else
		{
			// check if Bit 4 (-> Motor is ON) ist set
			if(motor.curr_status & (1 << 4))
			{
				if(motor.state != ST_OPERATION_ENABLED) {
					ROS_INFO_STREAM(motor.joint_name << ": operation enabled");
				}
				motor.state = ST_OPERATION_ENABLED;
			}
			else
			{
				if(motor.state != ST_OPERATION_DISABLED) {
					ROS_WARN_STREAM(motor.joint_name << ": operation disabled");
				}
				motor.state = ST_OPERATION_DISABLED;
			}
		}
	}

	void evaluate_motor_failure(motor_t& motor, int32_t prev_status)
	{
		if(motor.curr_motor_failure != prev_status)
		{
			if(motor.curr_motor_failure & (1 << 2)) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor failure: feedback loss");
			}
			else if(motor.curr_motor_failure & (1 << 3)) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor failure: peak current exceeded");
			}
			else if(motor.curr_motor_failure & (1 << 7)) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor failure: speed track error");
			}
			else if(motor.curr_motor_failure & (1 << 8)) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor failure: position track error");
			}
			else if(motor.curr_motor_failure & (1 << 17)) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor failure: speed limit exceeded");
			}
			else if(motor.curr_motor_failure & (1 << 21)) {
				ROS_ERROR_STREAM(motor.joint_name << ": motor failure: motor stuck");
			}
		}
	}

	void receive_loop()
	{
		bool is_error = false;

		while(do_run && ros::ok())
		{
			if(is_error || m_can_sock < 0)
			{
				std::lock_guard<std::mutex> lock(m_can_mutex);

				if(m_can_sock >= 0) {
					::close(m_can_sock);	// close first
				}
				if(is_error) {
					::usleep(1000 * 1000);	// in case of error sleep some time
					if(!do_run) {
						break;
					}
				}
				try {
					// open socket
					m_can_sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
					if(m_can_sock < 0) {
						throw std::runtime_error("socket() failed!");
					}
					// set can interface
					::ifreq ifr = {};
					::strncpy(ifr.ifr_name, m_can_iface.c_str(), IFNAMSIZ);
					if(::ioctl(m_can_sock, SIOCGIFINDEX, &ifr) < 0) {
						throw std::runtime_error("ioctl() failed!");
					}
					// bind to interface
					::sockaddr_can addr = {};
					addr.can_family = AF_CAN;
					addr.can_ifindex = ifr.ifr_ifindex;
					if(::bind(m_can_sock, (::sockaddr*)(&addr), sizeof(addr)) < 0) {
						throw std::runtime_error("bind() failed!");
					}
					ROS_INFO_STREAM("CAN interface '" << m_can_iface << "' opened successfully.");
				}
				catch(const std::exception& ex)
				{
					ROS_WARN_STREAM("Failed to open CAN interface '" << m_can_iface << "': "
							<< ex.what() << " (" << ::strerror(errno) << ")");
					is_error = true;
					continue;
				}
				is_error = false;
				m_can_condition.notify_all();	// notify that socket is ready
			}

			// read a frame
			::can_frame frame = {};
			const auto res = ::read(m_can_sock, &frame, sizeof(frame));
			if(res != sizeof(frame)) {
				if(do_run) {
					ROS_WARN_STREAM("read() failed with " << ::strerror(errno));
				}
				is_error = true;
				continue;
			}

			// convert frame
			can_msg_t msg;
			msg.id = frame.can_id & 0x1FFFFFFF;
			msg.length = frame.can_dlc;
			for(int i = 0; i < frame.can_dlc; ++i) {
				msg.data[i] = frame.data[i];
			}

			// process it
			{
				std::lock_guard<std::mutex> lock(m_node_mutex);

				m_wait_for_can_sock = false;		// disable waiting for transmit (avoid dead-lock)
				try {
					handle(msg);
				}
				catch(const std::exception& ex) {
					ROS_WARN_STREAM(ex.what());
				}
				m_wait_for_can_sock = true;			// enable waiting again
			}
		}

		// close socket
		{
			std::lock_guard<std::mutex> lock(m_can_mutex);
			if(m_can_sock >= 0) {
				::close(m_can_sock);
				m_can_sock = -1;
			}
			do_run = false;							// tell node that we are done
			m_can_condition.notify_all();			// notify that socket is closed
		}
	}

private:
	std::mutex m_node_mutex;

	ros::NodeHandle m_node_handle;

	ros::Publisher m_pub_joint_state;
	ros::Publisher m_pub_joint_state_raw;

	ros::Subscriber m_sub_joint_trajectory;
	ros::Subscriber m_sub_emergency_stop;
	ros::Subscriber m_sub_joy;

	int m_num_wheels = 0;
	std::vector<module_t> m_wheels;

	std::string m_can_iface;
	int m_motor_group_id = -1;
	int m_request_status_divider = 0;
	int m_heartbeat_divider = 0;
	double m_control_rate = 0;
	double m_motor_timeout = 0;
	double m_home_vel = 0;
	double m_steer_gain = 0;
	double m_steer_lookahead = 0;
	double m_steer_low_pass = 0;
	double m_max_steer_vel = 0;
	double m_drive_low_pass = 0;
	double m_motor_delay = 0;
	double m_trajectory_timeout = 0;
	bool m_auto_home = false;
	bool m_measure_torque = false;
	int m_homeing_button = -1;

	volatile bool do_run = true;
	bool is_homing_active = false;
	bool is_steer_reset_active = false;
	bool is_all_homed = false;
	bool is_em_stop = false;
	bool is_motor_reset = true;
	bool is_trajectory_timeout = false;
	bool is_stopped = true;

	uint64_t m_sync_counter = 0;
	ros::Time m_last_sync_time;
	ros::Time m_last_update_time;
	ros::Time m_last_trajectory_time;

	std::thread m_can_thread;
	std::mutex m_can_mutex;
	std::condition_variable m_can_condition;
	int m_can_sock = -1;
	bool m_wait_for_can_sock = true;

};


int main(int argc, char** argv)
{
	// initialize ROS
	ros::init(argc, argv, "neo_omnidrive_socketcan");

	ros::NodeHandle nh;

	double control_rate = 0;   // [1/s]
	nh.param("control_rate", control_rate, 50.0);

	// frequency of publishing states (cycle time)
	ros::Rate rate(control_rate);

	NeoSocketCanNode node;

	while(ros::ok())
	{
		try {
			node.initialize();
		}
		catch(std::exception& ex)
		{
			if(ros::ok())
			{
				ROS_ERROR_STREAM("NeoSocketCanNode: " << ex.what());
				::usleep(1000 * 1000);
				continue;
			}
		}
		break;
	}

	while(ros::ok())
	{
		ros::spinOnce();

		try {
			node.update();
		}
		catch(std::exception& ex)
		{
			if(ros::ok()) {
				ROS_ERROR_STREAM("NeoSocketCanNode: " << ex.what());
			}
		}

		rate.sleep();
	}

	node.shutdown();

	return 0;
}


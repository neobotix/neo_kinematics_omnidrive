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

#ifndef INCLUDE_OMNI_KINEMATICS_H_
#define INCLUDE_OMNI_KINEMATICS_H_

#include "OmniWheel.h"

#include <angles/angles.h>

#include <vector>
#include <math.h>


/*
 * Computes desired wheel angles and velocities based on commanded
 * platform velocity and yawrate.
 *
 * Takes into consideration current wheel angle and velocity, such as to
 * choose the closer solution which doesn't require reversing the velocity.
 */
void compute_kinematics(std::vector<OmniWheel>& wheels, double move_vel_x, double move_vel_y, double move_yawrate)
{
	for(auto& wheel : wheels)
	{
		const double wheel_pos_radius = wheel.get_wheel_pos_radius();		// wheel position in polar coords [m]
		const double wheel_pos_angle = wheel.get_wheel_pos_angle();			// wheel position in polar coords [rad]
		const double tangential = wheel_pos_radius * move_yawrate;					// tangential velocity
		const double vel_x = move_vel_x + tangential * -sin(wheel_pos_angle);		// tangential is 90 deg rotated (ie. in y direction at phi=0)
		const double vel_y = move_vel_y + tangential * cos(wheel_pos_angle);

		double new_wheel_angle = ::atan2(vel_y, vel_x);
		double new_wheel_vel = ::hypot(vel_x, vel_y);

		if(fabs(angles::shortest_angular_distance(new_wheel_angle, wheel.wheel_angle)) > M_PI / 2)
		{
			new_wheel_angle = angles::normalize_angle(new_wheel_angle + M_PI);
			new_wheel_vel = -1 * new_wheel_vel;
		}

		wheel.wheel_angle = new_wheel_angle;
		wheel.wheel_vel = new_wheel_vel;
	}
}




#endif // INCLUDE_OMNI_KINEMATICS_H_

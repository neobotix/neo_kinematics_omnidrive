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

#ifndef INCLUDE_OMNI_WHEEL_H_
#define INCLUDE_OMNI_WHEEL_H_

#include <math.h>
#include <string>


/*
 *
 * Note: lever_arm points to y axis, so a wheel angle of zero moves the wheel to the left.
 */
class OmniWheel {
public:
	double center_pos_x = 0;			// steering axis location relative to base_link [m]
	double center_pos_y = 0;			// steering axis location relative to base_link [m]
	double lever_arm = 0;				// distance between steering axis and wheel center [m]
	double home_angle = 0;				// wheel angle after homing [rad]

	double wheel_pos_x = 0;				// wheel position relative to base_link [m]
	double wheel_pos_y = 0;				// wheel position relative to base_link [m]

	double wheel_angle = 0;				// current wheel angle relative to base_link [rad]
	double wheel_vel = 0;				// current wheel velocity between ground and wheel_link [m/s]

	std::string drive_joint_name;
	std::string steer_joint_name;

	OmniWheel()
	{
	}

	OmniWheel(	double center_pos_x_, double center_pos_y_, double lever_arm_, double home_angle_,
				double wheel_angle_ = 0, double wheel_vel_ = 0)
		:	center_pos_x(center_pos_x_),
			center_pos_y(center_pos_y_),
			lever_arm(lever_arm_),
			home_angle(home_angle_)
	{
		set_wheel_angle(wheel_angle_);
		wheel_vel = wheel_vel_;
	}

	/*
	 * Computes new wheel position + sets new angle.
	 */
	void set_wheel_angle(double wheel_angle_)
	{
		wheel_angle = wheel_angle_;
		wheel_pos_x = center_pos_x + lever_arm * -sin(wheel_angle_);
		wheel_pos_y = center_pos_y + lever_arm * cos(wheel_angle_);
	}

	/*
	 * Returns wheel radius from center. (polar coords)
	 */
	double get_wheel_pos_radius() const {
		return ::hypot(wheel_pos_x, wheel_pos_y);
	}

	/*
	 * Returns wheel position angle relative to X axis. (polar coords)
	 */
	double get_wheel_pos_angle() const {
		return ::atan2(wheel_pos_y, wheel_pos_x);
	}

};


#endif // INCLUDE_OMNI_WHEEL_H_

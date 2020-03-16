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

#include <vector>
#include <array>
#include <string>
#include <iostream>

#include <ginac/ginac.h>

using namespace GiNaC;


/*
 * Computes theoretical linear friction for a wheel.
 */
std::array<ex, 2> wheel_friction_eq(ex wheel_angle, ex wheel_vel, ex move_vel_x, ex move_vel_y)
{
	ex wheel_vel_x = wheel_vel * cos(wheel_angle);
	ex wheel_vel_y = wheel_vel * sin(wheel_angle);
	return {move_vel_x - wheel_vel_x, move_vel_y - wheel_vel_y};
}

/*
 * Computes rigid body point velocities at a certain position.
 */
std::array<ex, 2> calc_move_vel_at_pos(ex wheel_pos_radius, ex wheel_pos_angle, ex move_vel_x, ex move_vel_y, ex move_yawrate)
{
	ex tangential = wheel_pos_radius * move_yawrate;				// tangential velocity
	ex vel_x = move_vel_x + tangential * -sin(wheel_pos_angle);		// tangential is 90 deg rotated (ie. in y direction at phi=0)
	ex vel_y = move_vel_y + tangential * cos(wheel_pos_angle);
	return {vel_x, vel_y};
}


int main ()
{
	symbol move_vel_x("move_vel_x");
	symbol move_vel_y("move_vel_y");
	symbol move_yawrate("move_yawrate");

	symbol wheel_pos_radius("wheel_pos_radius");
	symbol wheel_pos_angle("wheel_pos_angle");
	symbol wheel_angle("wheel_angle");
	symbol wheel_vel("wheel_vel");

	auto move_vel_wheel = calc_move_vel_at_pos(	wheel_pos_radius, wheel_pos_angle,
												move_vel_x, move_vel_y, move_yawrate);

	auto residual = wheel_friction_eq(wheel_angle, wheel_vel, move_vel_wheel[0], move_vel_wheel[1]);

	std::cout << csrc << "Rx = " << residual[0] << std::endl << std::endl;
	std::cout << csrc << "Ry = " << residual[1] << std::endl << std::endl;
	std::cout << csrc << "d_Rx_vel_x = " << diff(residual[0], move_vel_x) << std::endl << std::endl;
	std::cout << csrc << "d_Ry_vel_x = " << diff(residual[1], move_vel_x) << std::endl << std::endl;
	std::cout << csrc << "d_Rx_vel_y = " << diff(residual[0], move_vel_y) << std::endl << std::endl;
	std::cout << csrc << "d_Ry_vel_y = " << diff(residual[1], move_vel_y) << std::endl << std::endl;
	std::cout << csrc << "d_Rx_yawrate = " << diff(residual[0], move_yawrate) << std::endl << std::endl;
	std::cout << csrc << "d_Ry_yawrate = " << diff(residual[1], move_yawrate) << std::endl << std::endl;
}

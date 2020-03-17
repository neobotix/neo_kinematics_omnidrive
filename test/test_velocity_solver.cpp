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

#include "../include/VelocitySolver.h"

#include <iostream>


int main()
{
	std::vector<OmniWheel> wheels(4);

	wheels[0] = OmniWheel(0.4, 0.3, 0, 0);
	wheels[1] = OmniWheel(-0.4, 0.3, 0, 0);
	wheels[2] = OmniWheel(-0.4, -0.3, 0, 0);
	wheels[3] = OmniWheel(0.4, -0.3, 0, 0);

	VelocitySolver solver(4);

	for(auto& wheel : wheels)
	{
		wheel.set_wheel_angle(0);
		wheel.wheel_vel = 0;
	}

	solver.move_vel_x = 1;
	solver.move_vel_y = 1;
	solver.move_yawrate = 1;
	solver.solve(wheels);

	std::cout << "Test 0: " << solver.move_vel_x << ", " << solver.move_vel_y << ", " << solver.move_yawrate << " (R_norm = " << solver.R_norm  << ")" << std::endl;

	for(auto& wheel : wheels)
	{
		wheel.set_wheel_angle(0);
		wheel.wheel_vel = 1;
	}

	solver.move_vel_x = 0;
	solver.move_vel_y = 0;
	solver.move_yawrate = 0;
	solver.solve(wheels);

	std::cout << "Test 1: " << solver.move_vel_x << ", " << solver.move_vel_y << ", " << solver.move_yawrate << " (R_norm = " << solver.R_norm  << ")" << std::endl;

	for(auto& wheel : wheels)
	{
		wheel.set_wheel_angle(0);
		wheel.wheel_vel = 1;
	}
	wheels[0].set_wheel_angle(0.1);

	solver.move_vel_x = 0;
	solver.move_vel_y = 0;
	solver.move_yawrate = 0;
	solver.solve(wheels);

	std::cout << "Test 2: " << solver.move_vel_x << ", " << solver.move_vel_y << ", " << solver.move_yawrate << " (R_norm = " << solver.R_norm  << ")" << std::endl;

	for(auto& wheel : wheels)
	{
		wheel.set_wheel_angle(0);
		wheel.wheel_vel = 1;
	}
	wheels[0].wheel_vel = 0.9;

	solver.move_vel_x = 0;
	solver.move_vel_y = 0;
	solver.move_yawrate = 0;
	solver.solve(wheels);

	std::cout << "Test 3: " << solver.move_vel_x << ", " << solver.move_vel_y << ", " << solver.move_yawrate << " (R_norm = " << solver.R_norm  << ")" << std::endl;

	for(auto& wheel : wheels)
	{
		wheel.set_wheel_angle(0);
		wheel.wheel_vel = 1;
	}
	wheels[0].set_wheel_angle(1.57);
	wheels[0].wheel_vel = 1;

	solver.move_vel_x = 0;
	solver.move_vel_y = 0;
	solver.move_yawrate = 0;
	solver.solve(wheels);

	std::cout << "Test 4: " << solver.move_vel_x << ", " << solver.move_vel_y << ", " << solver.move_yawrate << " (R_norm = " << solver.R_norm  << ")" << std::endl;

}


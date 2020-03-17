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

#include <iostream>

void print_wheels(const std::vector<OmniWheel>& wheels)
{
	int i = 0;
	for(auto& wheel : wheels) {
		std::cout << "Wheel " << i++ << ": target_angle=" << wheel.wheel_angle << ", target_vel=" << wheel.wheel_vel << std::endl;
	}
}


int main()
{
	std::vector<OmniWheel> wheels(4);

	wheels[0] = OmniWheel(0.4,  0.3,  0.1, 0, 0, 0);			// front left
	wheels[1] = OmniWheel(-0.4, 0.3,  0.1, 0, 0, 0);			// back left
	wheels[2] = OmniWheel(-0.4, -0.3, 0.1, M_PI, M_PI, 0);		// back right
	wheels[3] = OmniWheel(0.4,  -0.3, 0.1, M_PI, M_PI, 0);		// front right

	OmniKinematics kinematics(4);

	{
		auto result = kinematics.compute(wheels, 0, 0, 0);
		std::cout << "Test 0:" << std::endl;
		print_wheels(result);
		std::cout << std::endl;
	}
	{
		auto result = kinematics.compute(wheels, 1, 0, 0);
		std::cout << "Test 1:" << std::endl;
		print_wheels(result);
		std::cout << std::endl;
	}
	{
		auto result = kinematics.compute(wheels, 0, 1, 0);
		std::cout << "Test 2:" << std::endl;
		print_wheels(result);
		std::cout << std::endl;
	}
	{
		auto result = kinematics.compute(wheels, 0, 0, 1);
		std::cout << "Test 3:" << std::endl;
		print_wheels(result);
		std::cout << std::endl;
	}
	{
		auto tmp = wheels;
		tmp[0].wheel_angle = 0;
		tmp[1].wheel_angle = 0;
		tmp[2].wheel_angle = M_PI;
		tmp[3].wheel_angle = M_PI;
		tmp[0].wheel_vel = 1;
		tmp[1].wheel_vel = 1;
		tmp[2].wheel_vel = -1;
		tmp[3].wheel_vel = -1;

		auto result = kinematics.compute(wheels, 0, -1, 0);
		std::cout << "Test 4:" << std::endl;
		print_wheels(result);
		std::cout << std::endl;
	}
	{
		auto tmp = wheels;
		tmp[0].wheel_angle = 0;
		tmp[1].wheel_angle = 0;
		tmp[2].wheel_angle = M_PI;
		tmp[3].wheel_angle = M_PI;
		tmp[0].wheel_vel = 1;
		tmp[1].wheel_vel = 1;
		tmp[2].wheel_vel = -1;
		tmp[3].wheel_vel = -1;

		auto result = kinematics.compute(wheels, -1, 0, 0);
		std::cout << "Test 5:" << std::endl;
		print_wheels(result);
		std::cout << std::endl;
	}

}


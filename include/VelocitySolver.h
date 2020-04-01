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

#ifndef INCLUDE_VELOCITY_SOLVER_H_
#define INCLUDE_VELOCITY_SOLVER_H_

#include "OmniWheel.h"

#include <neo_common/MatrixX.h>

#include <math.h>
#include <stdexcept>


/*
 * Computes platform velocity + yawrate based on a number of omni-drive wheels
 * and their given position, velocity and steering orientation.
 *
 * See omni_equations.cpp for the source of equations below.
 */
class VelocitySolver {
public:
	double R_norm = 0;				// solution error
	double move_vel_x = 0;			// solution [m/s]
	double move_vel_y = 0;			// solution [m/s]
	double move_yawrate = 0;		// solution [rad/s]

	VelocitySolver(int num_wheels_)
		:	num_wheels(num_wheels_),
			M(num_wheels_ * 2)
	{
		R.resize(M, 1);
		J.resize(M, 3);
	}

	void solve(const std::vector<OmniWheel>& wheels)
	{
		if(wheels.size() != num_wheels) {
			throw std::logic_error("wheels.size() != num_wheels");
		}

		// make two iterations to get final R_norm
		for(int iter = 0; iter < 2; ++iter)
		{
			J.fill(0);		// unset J values should be zero

			for(int i = 0; i < num_wheels; ++i)
			{
				const double wheel_pos_radius = wheels[i].get_wheel_pos_radius();	// wheel position in polar coords [m]
				const double wheel_pos_angle = wheels[i].get_wheel_pos_angle();		// wheel position in polar coords [rad]
				const double wheel_angle = wheels[i].wheel_angle;
				const double wheel_vel = wheels[i].wheel_vel;

				R[i * 2 + 0] = move_vel_x-wheel_vel*cos(wheel_angle)-wheel_pos_radius*sin(wheel_pos_angle)*move_yawrate;
				R[i * 2 + 1] = -wheel_vel*sin(wheel_angle)+wheel_pos_radius*cos(wheel_pos_angle)*move_yawrate+move_vel_y;

				J(i * 2 + 0, 0) = 1;
				J(i * 2 + 1, 1) = 1;
				J(i * 2 + 0, 2) = -wheel_pos_radius*sin(wheel_pos_angle);
				J(i * 2 + 1, 2) = wheel_pos_radius*cos(wheel_pos_angle);
			}
			R_norm = R.norm();

			// solve Gauss-Newton step
			const Matrix<double, 3, 3> H(J.transpose() * J);
			const Matrix<double, 3, 1> X = H.inverse() * Matrix<double, 3, 1>(J.transpose() * R);

			move_vel_x -= X[0];
			move_vel_y -= X[1];
			move_yawrate -= X[2];
		}
	}

private:
	const int num_wheels = 0;
	const int M = 0;

	MatrixX<double> R;			// residual vector
	MatrixX<double> J;			// jacobian matrix

};


#endif // INCLUDE_VELOCITY_SOLVER_H_

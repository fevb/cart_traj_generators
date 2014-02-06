/*
 *  CircleTrajGenerator.cpp
 *
 *
 *  Created on: Feb 5, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cart_traj_generators/CircleTrajGenerator.h>


CircleTrajGenerator::CircleTrajGenerator(const KDL::Frame &F_init) : CartTrajGenerator(F_init)
{

}

CircleTrajGenerator::~CircleTrajGenerator()
{

}

void CircleTrajGenerator::setRadius(double radius)
{
	m_radius = radius;
}

void CircleTrajGenerator::setPeriod(double period)
{
	m_period = period;
}

void CircleTrajGenerator::getSetPoint(double time, KDL::Frame &F, KDL::Twist &v)
{
	F.M = m_F_init.M;
	v.rot = KDL::Vector::Zero();

	double r = m_radius;
	double f = 1/(m_period);

	KDL::Vector circle;

	if(time>m_duration)
	{
		circle(0) = r*sin(2*M_PI*f*m_duration);
		circle(1) = -r*cos(2*M_PI*f*m_duration) + r;
		circle(2) = 0.0; //-0.01*sin(2*M_PI*2*f*t)-0.04; // make the z component a bit beneath the table surface
		F.p = m_F_init.p + circle;

		v = KDL::Twist::Zero();
		return;
	}


	circle(0) = r*sin(2*M_PI*f*time);
	circle(1) = -r*cos(2*M_PI*f*time) + r;
	circle(2) = 0.0; //-0.01*sin(2*M_PI*2*f*t)-0.04; // make the z component a bit beneath the table surface

	F.p = m_F_init.p + circle;

	v.vel(0) = r*2*M_PI*f*cos(2*M_PI*f*time);
	v.vel(1) = r*2*M_PI*f*sin(2*M_PI*f*time);
	v.vel(2) = 0.0;

	// linearly increase velocity from 0 during first second of the trajectory
	if(time<1.0)
	{
		v.vel = v.vel*time;
	}

	else if(time>m_duration-1.0)
	{
		v.vel = v.vel*(m_duration-time);
	}

}




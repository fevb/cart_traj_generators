/*
 *  LinePeriodicTrajGenerator.h
 *
 *
 *  Created on: May 5, 2014
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

#include <cart_traj_generators/LinePeriodicTrajGenerator.h>

LinePeriodicTrajGenerator::LinePeriodicTrajGenerator() : LineTrajGenerator()
{


}

LinePeriodicTrajGenerator::~LinePeriodicTrajGenerator()
{

}

void LinePeriodicTrajGenerator::setPeriod(double period)
{
	m_period = period;
}

void LinePeriodicTrajGenerator::getSetPoint(double time, KDL::Frame &F, KDL::Twist &v)
{
	double t = fmod(time, m_period);

	F = m_F_init;
	// set zero rotational velocity
	v.rot = KDL::Vector::Zero();

	KDL::Vector line_pos;

	KDL::Vector sine_wave_pos;
	v.vel = m_normal_direction*(2.0*M_PI)*m_sine_amplitude*sin(2.0*M_PI*time);
	// add a small offset to the sine wave
	sine_wave_pos = -m_sine_amplitude*cos(2.0*M_PI*time)*m_normal_direction
	- m_sine_amplitude*m_normal_direction - 0.01*m_normal_direction;

	KDL::Vector max_vel = m_tangential_direction*m_vel;

	if(t>(m_period/2.0))
	{
		t = fmod(t,(m_period/2.0));
		F.p = F.p + max_vel/2.0 + max_vel*(m_period/2.0-2.0) + max_vel/2.0;
		max_vel = max_vel*-1;
	}


	if(t<1.0)
	{
		v.vel = v.vel + max_vel*t;
		line_pos = max_vel*pow(t, 2.0)/2.0;
		F.p = F.p + line_pos + sine_wave_pos;
		return;
	}

	else if(t >= 1.0 && t <= m_period/2.0-1.0)
	{
		v.vel = v.vel + max_vel;
		line_pos = max_vel/2.0 + max_vel*(t-1.0);
		F.p = F.p + line_pos + sine_wave_pos;
		return;
	}

	else if(t>=m_period/2.0-1.0 && t<=m_period/2.0)
	{
		v.vel = v.vel + max_vel*(m_period/2.0-t);
		line_pos = max_vel/2.0 + max_vel*(m_period/2.0-2.0) + max_vel*pow(m_period/2.0-t, 2.0)/2.0;
		F.p = F.p + line_pos + sine_wave_pos;
	}

	if(time>=m_duration)
	{
		v.vel = KDL::Vector::Zero();
	}

}

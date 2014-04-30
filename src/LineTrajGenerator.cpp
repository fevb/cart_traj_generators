/*
 *  LineTrajGenerator.cpp
 *
 *
 *  Created on: Apr 30, 2014
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

#include <cart_traj_generators/LineTrajGenerator.h>
#include <math.h>

LineTrajGenerator::LineTrajGenerator() : CartTrajGenerator()
{

}

LineTrajGenerator::~LineTrajGenerator()
{

}

void LineTrajGenerator::setTangentialDirection(KDL::Vector tangential_direction)
{
	m_tangential_direction = tangential_direction;
	m_tangential_direction.Normalize();
}

void LineTrajGenerator::setNormalDirection(KDL::Vector normal_direction)
{
	m_normal_direction = normal_direction;
	m_normal_direction.Normalize();
}

void LineTrajGenerator::setVel(double vel)
{
	m_vel = vel;
}

void LineTrajGenerator::setSineAmplitude(double sine_amplitude)
{
	m_sine_amplitude = fabs(sine_amplitude);
}

void LineTrajGenerator::getSetPoint(double time, KDL::Frame& F, KDL::Twist& v)
{
	F = m_F_init;
	// set zero rotational velocity
	v.rot = KDL::Vector::Zero();

	KDL::Vector line_pos;

	KDL::Vector sine_wave_pos;
	v.vel = m_normal_direction*(2.0*M_PI)*m_sine_amplitude*sin(2.0*M_PI*time);
	// add a small offset to the sine wave
	sine_wave_pos = -m_sine_amplitude*cos(2.0*M_PI*time)*m_normal_direction
	- m_sine_amplitude*m_normal_direction - 0.002*m_normal_direction;

	KDL::Vector max_vel = m_tangential_direction*m_vel;

	if(time<1.0)
	{
		v.vel = v.vel + max_vel*time;
		line_pos = max_vel*pow(time, 2.0)/2.0;
		F.p = F.p + line_pos + sine_wave_pos;
		return;
	}

	else if(time >= 1.0 && time <= m_duration-1.0)
	{
		v.vel = v.vel + max_vel;
		line_pos = max_vel/2.0 + max_vel*(time-1.0);
		F.p = F.p + line_pos + sine_wave_pos;
		return;
	}

	else if(time>=m_duration-1.0 && time<=m_duration)
	{
		v.vel = v.vel + max_vel*(m_duration-time);
		line_pos = max_vel/2.0 + max_vel*(m_duration-2.0) + max_vel*pow(m_duration-time, 2.0)/2.0;
		F.p = F.p + line_pos + sine_wave_pos;
	}

	else
	{
		v.vel = KDL::Vector::Zero();
		line_pos = max_vel + max_vel*(m_duration-2.0);
		F.p = F.p + line_pos;
	}

}



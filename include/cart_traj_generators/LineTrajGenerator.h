/*
 *  LineTrajGenerator.h
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

#ifndef LINETRAJGENERATOR_H_
#define LINETRAJGENERATOR_H_

#include <cart_traj_generators/CartTrajGenerator.h>

class LineTrajGenerator : public CartTrajGenerator
{
public:
	LineTrajGenerator();
	virtual ~LineTrajGenerator();

	// direction in which to move in a straight line
	void setTangentialDirection(KDL::Vector tangential_direction);

	// normal direction on which to apply sine wave
	void setNormalDirection(KDL::Vector normal_direction);

	// reference velocity of the line trajectory
	void setVel(double vel);

	// amplitude of the sine wave in the normal direction
	void setSineAmplitude(double sine_amplitude);

	virtual void getSetPoint(double time, KDL::Frame &F, KDL::Twist &v);

protected:

	double m_vel;
	KDL::Vector m_tangential_direction;
	KDL::Vector m_normal_direction;

	double m_sine_amplitude;

};

#endif /* LINETRAJGENERATOR_H_ */

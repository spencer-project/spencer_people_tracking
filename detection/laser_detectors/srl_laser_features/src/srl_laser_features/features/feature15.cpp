/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  Copyright (c) 2006-2012, Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
*    Oscar Martinez, Autonomous Intelligent Systems, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <srl_laser_features/features/feature15.h>

#ifdef HAVE_LIB_FFTW
#include <fftw3.h>
#endif


namespace srl_laser_features {

Feature15::Feature15() : Feature()
{
	pthread_mutex_init(&m_mutex, NULL);
}

Feature15::~Feature15()
{
	pthread_mutex_destroy(&m_mutex);
}

void Feature15::evaluate(const Segment& segment, Eigen::VectorXd& result) const
{
	#ifndef HAVE_LIB_FFTW
		result = Eigen::VectorXd::Zero(0);
	#else
		result = Eigen::VectorXd::Zero(6);

		const size_t numPoints = segment.points.size();
		if (numPoints > 1) {
			fftw_complex* in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numPoints);
			fftw_complex* out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * numPoints);

			pthread_mutex_lock(&m_mutex);
			fftw_plan plan = fftw_plan_dft_1d(numPoints, in, out, -1/*FFTW_FORWARD*/, 1U << 6/*FFTW_ESTIMATE*/);
			pthread_mutex_unlock(&m_mutex);

			// copy
			for (size_t pIndex = 0; pIndex < numPoints; ++pIndex) {
				in[pIndex][0] = segment.points[pIndex](0);
				in[pIndex][1] = segment.points[pIndex](1);
			}

			fftw_execute(plan); /* repeat as needed */

			// output
			result(0) = out[0][0];
			result(1) = out[0][1];
			result(2) = out[1][0];
			result(3) = out[1][1];
			result(4) = out[2][0];
			result(5) = out[2][1];

			// check illegal result
			if (isnan(result(4)) || result(4) == DBL_MAX) {
				result(4) = -1.0;
			}
			if (isnan(result(5)) || result(5) == DBL_MAX) {
				result(5) = -1.0;
			}

			fftw_destroy_plan(plan);
			fftw_free(in);
			fftw_free(out);
		}
	#endif
}

} // end of namespace srl_laser_features

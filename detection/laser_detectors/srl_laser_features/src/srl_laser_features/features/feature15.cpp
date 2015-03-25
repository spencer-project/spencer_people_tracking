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

/*!
 \brief Header file of the Feature15 class.
 \author Matthias Luber, Luciano Spinello and Kai O. Arras, Social Robotics Laboratory and
 Oscar Martinez, Autonomous Intelligent Systems, Albert-Ludwigs-University of Freiburg, Germany
 */
#ifndef SRL_LASER_FEATURES_FEATURE15_H_
#define SRL_LASER_FEATURES_FEATURE15_H_

#include <srl_laser_features/features/feature.h>
#include <cstdio>


namespace srl_laser_features {

/**
 * First six Fourier coefficients.
 *
 * @author Matthias Luber, Luciano Spinello, Oscar Martinez, Kai O. Arras
 */
class Feature15: public Feature
{
public: 
	Feature15();
	virtual ~Feature15();

	inline virtual const FeatureDimension getDescription(unsigned int dimension) const
	{
		assert(dimension < getNDimensions());

		char text[128];
		sprintf(text, "Fourier coefficient %d.", dimension);
		return text;
	}

	inline unsigned int getNDimensions() const
	{
		#ifndef HAVE_LIB_FFTW
			return 0;
		#else
			return 6;
		#endif
	}

	virtual void evaluate(const Segment& segment, Eigen::VectorXd& result) const;

private:
	/// fftw_plan_dft_1d is not thread safe. See http://www.fftw.org/fftw3_doc/Thread-safety.html 
	pthread_mutex_t m_mutex;
};

} // end of namespace srl_laser_features

#endif // SRL_LASER_FEATURES_FEATURE15_H_
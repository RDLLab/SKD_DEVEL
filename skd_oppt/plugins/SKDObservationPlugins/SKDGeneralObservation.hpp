#ifndef _NCAP_COLLISION_OBSERVATION_HPP_
#define _NCAP_COLLISION_OBSERVATION_HPP_
#include <oppt/robotHeaders/Observation.hpp>
#include <limits>

using std::cout;
using std::endl;


namespace oppt {


// ENUMERATED DISCRETE CAR_INTENTIONS
enum OBSERVATION_INFO{REL_LONGIT, REL_HORIZONTAL}; 



class SKDGeneralObservation: public VectorObservation {
public:
	SKDGeneralObservation(VectorFloat& observationVec):
		VectorObservation(observationVec) {

	}


	virtual ~SKDGeneralObservation() {}

	virtual FloatType distanceTo(const Observation& otherObservation) const override {
		// Very high number to make it a different observation by default
		FloatType dist = 100000; 
		const FloatType LONGIT_OFFSET = 2.5;
		const FloatType HORIZONTAL_OFFSET = 2;
		const FloatType LONGIT_DISCRETE = 3.125;
		const FloatType HORIZONTAL_DISCRETE = 2;
		VectorFloat otherObsVec = static_cast<const SKDGeneralObservation &>(otherObservation).asVector();
		FloatType thisObsBinLongit = std::ceil(
			(observationVec_[OBSERVATION_INFO::REL_LONGIT] + LONGIT_OFFSET) // Offset to positive bins
			 	/ LONGIT_DISCRETE);

		FloatType thisObsBinHorizontal = std::ceil(
			(observationVec_[OBSERVATION_INFO::REL_HORIZONTAL] + HORIZONTAL_OFFSET) // Offset to positive bins
			 	/ HORIZONTAL_DISCRETE);

		// Comparison observation
		FloatType otherBinLongit = std::ceil(
			(otherObsVec[OBSERVATION_INFO::REL_LONGIT] + LONGIT_OFFSET)
				 / LONGIT_DISCRETE);

		FloatType otherBinHorizontal = std::ceil(
			(otherObsVec[OBSERVATION_INFO::REL_HORIZONTAL] + HORIZONTAL_OFFSET)
				/ HORIZONTAL_DISCRETE);


		// Add the L2-distance between observations
 		if((thisObsBinLongit == otherBinLongit) && (thisObsBinHorizontal == otherBinHorizontal)){
			// Same discretized bin is observed. Consider same observation
			dist = 0;
		}


		//Return big distance for different observations (bins). Otherwise, observation is the same
		return dist;
	}


};
}

#endif
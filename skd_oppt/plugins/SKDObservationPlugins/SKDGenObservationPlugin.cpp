/**
 * Copyright 2017
 * 
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the 
 * GNU General Public License published by the Free Software Foundation, 
 * either version 2 of the License, or (at your option) any later version.
 * 
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OPPT. 
 * If not, see http://www.gnu.org/licenses/.
 */

#ifndef _SKD_KAMIKAZE_OBSERVATION_PLUGIN_HPP_
#define _SKD_KAMIKAZE_OBSERVATION_PLUGIN_HPP_



#include "oppt/plugin/Plugin.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "SKDGeneralObservation.hpp"
#include "../SKDDefines/KamikazeTrajGenGeneralUtils.hpp"



namespace oppt
{
class SKDGenObservationPlugin: public ObservationPlugin
{
public :
    SKDGenObservationPlugin():
        ObservationPlugin() {
    }

    virtual ~SKDGenObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        // Parse options files
        parseOptions_<KamikazeTrajGenGeneralOptions>(optionsFile);
        KamikazeTrajGenGeneralOptions* generalOptions = static_cast<KamikazeTrajGenGeneralOptions*>(options_.get());

        // Left and right
        carDimensions_ = generalOptions->carDimensions;

        // Generate uniform distribution for errors with respect to the intention
        if(!makeErrorDistribution())
            ERROR("Unable to generate error distribution");

        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        

        // Container for the results information
        ObservationResultSharedPtr observationResult = std::make_shared<ObservationResult>();
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        auto robot = robotEnvironment_->getRobot();

        // Retrieve state information from the observation request
        VectorFloat stateVector = observationRequest->currentState->as<VectorState>()->asVector();

        // Create observation based on state information
        VectorFloat observationVector(2, 0);

        // Estimate rel_x, and rel_y components based on the informaiton in the state vector
        observationVector[OBSERVATION_INFO::REL_LONGIT] = stateVector[STATE_INFO::PED_LONGIT] - stateVector[STATE_INFO::CAR_LONGIT];
        observationVector[OBSERVATION_INFO::REL_HORIZONTAL] = stateVector[STATE_INFO::PED_HORIZONTAL] - stateVector[STATE_INFO::CAR_HORIZONTAL];

        // Double check that this method is called approapiately
        VectorFloat errorVector = sampleErrorVector();

        // Add noise to the observation based on sampled error vector
        for(size_t i = 0; i < 1; ++i){
            observationVector[i] += errorVector[i];
        }

        
        // create an observation based on the observation vector
        observationResult->observation = std::make_shared<SKDGeneralObservation>(observationVector);
        //printVector(observationVector, "obsVec");


        // Add the observation error to the observationVector
        return observationResult;
    }

    virtual FloatType calcLikelihood(const RobotStateSharedPtr& state,
                                     const Action *action,
                                     const Observation *observation) const  override{
        // Get the nominal observation
        VectorFloat stateVector = state->as<VectorState>()->asVector();
        VectorFloat obsVector = observation->as<VectorObservation>()->asVector();


        // Check that the state and observation vectors are the correct size
        if(stateVector.size() != 6){
             ERROR("STATE VECTOR wrong size on observation plugin. Please check state vector is approapiate");
        }
        
       
        // Check that both state and observation are same size
        if(obsVector.size() != 2){
            ERROR("OBS VECTOR wrong size on observation plugin. Please check obs vector is approapiate");
        }


        // Construct estimate of the relative positioning between agents in the environment based on state vector
        VectorFloat actualRel;

        // Push back estimate of the relative position on x
        actualRel.push_back(stateVector[STATE_INFO::PED_LONGIT] - stateVector[STATE_INFO::CAR_LONGIT]);
        actualRel.push_back(stateVector[STATE_INFO::PED_HORIZONTAL] - stateVector[STATE_INFO::CAR_HORIZONTAL]);

        // Diff vector
        VectorFloat diffVec;
        for(size_t i = 0; i < actualRel.size(); ++i){
            diffVec.push_back(obsVector[i] - actualRel[i]);
        }


        // Calculate probability of seeing the difference between the actual and observed information        
        FloatType pdfVal = errorDistribution_->pdf(diffVec);        

        return pdfVal;
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }



    

private:
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

    VectorFloat carDimensions_;



private:
    // Create error distribution for sensor noise in car's intention
    bool makeErrorDistribution() {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();

        // The mean vector for the 2D multivariate normal distribution
        Matrixdf mean = Matrixdf::Zero(2, 1);
        Matrixdf covarianceMatrix = Matrixdf::Identity(2, 2);
        //Populate the covariance matrix 
        covarianceMatrix(0, 0) = 1;
        covarianceMatrix(1,1) = 1;

        errorDistribution_ = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        errorDistribution_->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
        return true;
    }



    // Sample an error vector from the current distribution
    VectorFloat sampleErrorVector() const{
        VectorFloat errorVec;
        Vectordf sampledVector = getErrorDistribution()->sample(1);
        for(size_t elemNum = 0; elemNum < sampledVector.size() ; ++elemNum){
            errorVec.push_back(sampledVector[elemNum]);
        }
        return errorVec;
    }


    // Clamp a value
    FloatType clamp(const FloatType val, const FloatType bound1, const FloatType bound2) const{
        FloatType min = bound1;
        FloatType max = bound2;

        // Check for max and min bounds
        if(min >= max){
            // Reverse bounds
            min = bound2;
            max = bound1;
        }

        if(val < min){
            return min;
        } else if (val > max){
            return max;
        }

        // Does not exceed bounds
        return val;
    }


};

OPPT_REGISTER_OBSERVATION_PLUGIN(SKDGenObservationPlugin)

}

#endif

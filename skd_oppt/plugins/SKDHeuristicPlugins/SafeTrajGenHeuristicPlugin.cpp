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
#ifndef _SAFE_TRAJ_GEN_HEURISTIC_PLUGIN_HPP_
#define _SAFE_TRAJ_GEN_HEURISTIC_PLUGIN_HPP_
#include "oppt/plugin/Plugin.hpp"
#include "../SKDDefines/SafeTrajGenGeneralUtils.hpp"

namespace oppt
{
class SafeTrajGenHeuristicPlugin: public HeuristicPlugin
{
public:
    SafeTrajGenHeuristicPlugin():
        HeuristicPlugin() {

    }

    virtual ~SafeTrajGenHeuristicPlugin() {

    }

    virtual bool load(const std::string& optionsFile) override {        
        optionsFile_ = optionsFile;
        parseOptions_<SafeTrajGenGeneralOptions>(optionsFile);
        generalOptions_ = static_cast<SafeTrajGenGeneralOptions*>(options_.get());



        return true;
    }

  virtual FloatType getHeuristicValue(const HeuristicInfo* heuristicInfo) const override {   
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
       // Retrieve information from the state
        VectorFloat stateVec = heuristicInfo->currentState->as<VectorState>()->asVector();
        // User data associated with the state
        auto nextStateUserData = 
            static_cast<SKDUserData*>(heuristicInfo->currentState->getUserData().get());

        // Further information from general options
        FloatType stepPenalty_ = generalOptions_->stepPenalty;
        FloatType stepTime_ = generalOptions_->fixedStepTime;
        VectorFloat carDimensions_ = generalOptions_->carDimensions;

        // Extract location information from state
        VectorFloat pedLocation{stateVec[STATE_INFO::PED_LONGIT], stateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{stateVec[STATE_INFO::CAR_LONGIT], stateVec[STATE_INFO::CAR_HORIZONTAL]};

        // Calculate relative distance on both dimensions
        VectorFloat goalArea = generalOptions_->safetyGoal;
        FloatType transverseDist = stateVec[STATE_INFO::PED_HORIZONTAL] - (goalArea[GOAL_AREAS::GOAL_HOZ]) ; 
        FloatType longitDist = stateVec[STATE_INFO::PED_LONGIT] - (goalArea[GOAL_AREAS::GOAL_LONGIT]) ; 

       
        // // Steps until longitudinal margin
        FloatType goalEstimatedStepsLongit = std::abs(longitDist) / (MAX_SPEED * stepTime_);
        // Approximated steps till hitting the car from left side
        FloatType goalEstimatedStepsHoz = std::abs(transverseDist) / (MAX_SPEED * stepTime_);


        // Approximated values is approximated steps until goal and approximated future penalties incurred from current state
        FloatType heuristicVal = generalOptions_->goalReward - (2 * goalEstimatedStepsHoz  * stepPenalty_) - (goalEstimatedStepsLongit * stepPenalty_);

       
        return heuristicVal;
    }

   
private:
    // Pointer to structure with parsed options values
    SafeTrajGenGeneralOptions* generalOptions_;
    std::string optionsFile_;


};

OPPT_REGISTER_HEURISTIC_PLUGIN(SafeTrajGenHeuristicPlugin)

}

#endif

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
#include "oppt/plugin/Plugin.hpp"
#include "../SKDDefines/SafeTrajGenGeneralUtils.hpp"
#include <cmath> 

namespace oppt
{
class SafeTrajGenRewardPlugin: public RewardPlugin
{
public :
    SafeTrajGenRewardPlugin():
        RewardPlugin() {

    }

    virtual ~SafeTrajGenRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        robotEnvironment_ = robotEnvironment_;
        parseOptions_<SafeTrajGenGeneralOptions>(optionsFile);
        generalOptions_ = static_cast<SafeTrajGenGeneralOptions*>(options_.get());
        carDimensions_  = generalOptions_->carDimensions;

        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();

        // If there is a collision, then the penalty is the negative of the reward in the collision case
        if (propagationResult->collisionReport->collides) {
            return -(generalOptions_->goalReward);
            // return generalOptions_->goalReward;
        } else if (robotEnvironment_->isTerminal(propagationResult)){
            // Terminal because the pedestrian reached the goal (since the other case is checked above)
            return generalOptions_->goalReward;
        }

        return -generalOptions_->stepPenalty;
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-generalOptions_->terminalPenalty,
                              generalOptions_->goalReward * 2);
    }

private:
    SafeTrajGenGeneralOptions* generalOptions_;
    VectorFloat carDimensions_;

};

OPPT_REGISTER_REWARD_PLUGIN(SafeTrajGenRewardPlugin)

}

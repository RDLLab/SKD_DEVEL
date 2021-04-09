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
#include "../SKDDefines/KamikazeTrajGenGeneralUtils.hpp"
#include <cmath> 

using std::endl;
using std::cout;

namespace oppt
{
class KamkazeTrajGenRewardPlugin: public RewardPlugin
{
public :
    KamkazeTrajGenRewardPlugin():
        RewardPlugin() {

    }

    virtual ~KamkazeTrajGenRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<KamikazeTrajGenGeneralOptions>(optionsFile);
        generalOptions_ = static_cast<KamikazeTrajGenGeneralOptions*>(options_.get());
        carDimensions_  = generalOptions_->carDimensions;

        return true;
    }

    virtual FloatType getReward(const PropagationResultSharedPtr& propagationResult) const override {
        // Constant for problem
        const FloatType MAX_SPEED = 2.5;
        // Retrieve next state (s') from the propagation result
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
        // Retrieve the user data associated with the next state
        auto nextStateUserData =
            static_cast<SKDUserData*>(propagationResult->nextState->getUserData().get());
        FloatType currentVisitIndex = nextStateUserData->visitIndex;


        // Retrieve safe trajectory and step time
        TrajData pedSafeTraj = nextStateUserData->safeTrajectory;
        TrajPoint currentSafeTrajPoint = pedSafeTraj[currentVisitIndex];
        FloatType stepTime = generalOptions_->fixedStepTime;

        // Ensure trajectories are of the same size
        if(currentVisitIndex >= pedSafeTraj.size()){
            ERROR("VISIT INDEX EXCEEDS SAFE TRAJ SIZE");
        }


        // Calculate distance between ped and horizontal line
        FloatType distanceFromSafetLongit = std::abs(stateVec[STATE_INFO::PED_LONGIT] - currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_LONGIT]);
        FloatType distanceFromSafeHoz = 
            std::abs(stateVec[STATE_INFO::PED_HORIZONTAL] - currentSafeTrajPoint[SAFE_TRAJ::SAFE_PED_HOZ]);


        
        // Initialise reward with step penalty and accumulate according to future scenarios
        FloatType reward = -generalOptions_->stepPenalty;

        // Append penalty from deviating from safe traj vertically
        reward += (distanceFromSafeHoz/(MAX_SPEED * stepTime)) * (2 * -generalOptions_->stepPenalty);

        // Append penaly from distance to safe traj
        reward += (distanceFromSafetLongit/(MAX_SPEED * stepTime)) * (-generalOptions_->stepPenalty);

     
        if (!propagationResult)
            ERROR("Propagation report is null");

        /** Make sure we have some collision information */
        if (!propagationResult->collisionReport) {
            oppt::CollisionReportSharedPtr collisionReport =
                robotEnvironment_->getRobot()->makeDiscreteCollisionReport(propagationResult->nextState);
            propagationResult->collisionReport = collisionReport;            
        }

        // If a collision was achieved, return the success reward
        if (propagationResult->collisionReport->collides) {
                reward += generalOptions_->goalReward;
        } else if (robotEnvironment_->isTerminal(propagationResult)){
            // Terminal but not due to collision (car has safely avoided pedestrian)
            reward -= generalOptions_->terminalPenalty;
        }




        if(robotEnvironment_->isExecutionEnvironment()){
            cout << "DEBUGGING REWARD PLUGIN" << endl;
            cout << "SAGE PED KEY INDEX " << generalOptions_->safeTrajKey;
            cout << "CURRENT VISIT INDEX " << currentVisitIndex;
            printVector(currentSafeTrajPoint, "CURRENT SAFE PED POINT");
        }


        return reward;


        
    }

    virtual std::pair<FloatType, FloatType> getMinMaxReward() const override {
        return std::make_pair(-generalOptions_->terminalPenalty,
                              generalOptions_->goalReward * 2);
    }

private:
    KamikazeTrajGenGeneralOptions* generalOptions_;
    VectorFloat carDimensions_;

};

OPPT_REGISTER_REWARD_PLUGIN(KamkazeTrajGenRewardPlugin)

}

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
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "../SKDDefines/KamikazeTrajGenGeneralUtils.hpp"

namespace oppt
{
class KamikazeTrajGenTerminalPlugin: public TerminalPlugin
{
public :
    KamikazeTrajGenTerminalPlugin():
        TerminalPlugin() {

    }

    virtual ~KamikazeTrajGenTerminalPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<KamikazeTrajGenGeneralOptions>(optionsFile);
        // Save pointer to options file
        generalOptions_ = static_cast<KamikazeTrajGenGeneralOptions*>(options_.get());
        robotEnvironment_ = robotEnvironment_;
        return true;
    }

    virtual ValidityReportSharedPtr isValid(const PropagationResultSharedPtr& propagationResult) override {
        ValidityReportSharedPtr validityReport(new ValidityReport(propagationResult->nextState));
        validityReport->satisfiesConstraints = true;
        validityReport->isValid = true;
       

        return validityReport;
    }

    virtual bool isTerminal(const PropagationResultSharedPtr& propagationResult) override {  
        VectorFloat nextStateVector = propagationResult->nextState.get()->as<VectorState>()->asVector();
        auto userData = static_cast<SKDUserData*>(propagationResult->nextState->getUserData().get());
        VectorFloat carDimensions_ = generalOptions_->carDimensions;

        // // Terminal state if a collision happens
        if(userData->collisionReport->collides){

            return true;
        }

        /* Terminal state if the car is no longer reachable for the pedestrian.
        This is, if the longitudinal position of the car is already above the pedestrian */
        if(nextStateVector[STATE_INFO::CAR_LONGIT] > (nextStateVector[STATE_INFO::PED_LONGIT] + carDimensions_[0]/2)){
            //std::cout << "AVOIDED" << std::endl;
            return true;
        }


        return false;
    }

private:
    FloatType avoidedDist_;

    KamikazeTrajGenGeneralOptions* generalOptions_;

private:
  

};

OPPT_REGISTER_TERMINAL_PLUGIN(KamikazeTrajGenTerminalPlugin)

}





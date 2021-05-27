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
#include "../SKDDefines/SafeTrajGenGeneralUtils.hpp"

namespace oppt
{
class SafeTrajGenTerminalPlugin: public TerminalPlugin
{
public :
    SafeTrajGenTerminalPlugin():
        TerminalPlugin() {

    }

    virtual ~SafeTrajGenTerminalPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<SafeTrajGenGeneralOptions>(optionsFile);
        // Save pointer to options file
        generalOptions_ = static_cast<SafeTrajGenGeneralOptions*>(options_.get());
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
        

        // Retrieve goal bounds
        VectorFloat goalBounds = generalOptions_->goalBounds;

         // Terminal state if a collision happens
        if(userData->collisionReport->collides){
            return true;
        }
        
        // Terminal if in the vicinity of the goal area
        FloatType maxBoundLongit = goalBounds[GOAL_BOUNDS::GOAL_LONGIT_MAX];
        FloatType minBoundLongit = goalBounds[GOAL_BOUNDS::GOAL_LONGIT_MIN];
        FloatType maxBoundHoz = goalBounds[GOAL_BOUNDS::GOAL_HOZ_MAX];
        FloatType minBoundHoz = goalBounds[GOAL_BOUNDS::GOAL_HOZ_MIN];

        bool withinLongitMargin = (nextStateVector[STATE_INFO::PED_LONGIT] <= maxBoundLongit) && (nextStateVector[STATE_INFO::PED_LONGIT] >= minBoundLongit);
        bool withinHozMargin = (nextStateVector[STATE_INFO::PED_HORIZONTAL] <= maxBoundHoz) && (nextStateVector[STATE_INFO::PED_HORIZONTAL] >= minBoundHoz);
        
        // Check if within goal
        if(withinLongitMargin && withinHozMargin){
            return true;
        }


        // Terminal cases not met, return false 
        return false;
    }

private:
    FloatType avoidedDist_;

    SafeTrajGenGeneralOptions* generalOptions_;

private:
  

};

OPPT_REGISTER_TERMINAL_PLUGIN(SafeTrajGenTerminalPlugin)

}





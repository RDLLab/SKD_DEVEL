




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
#ifndef _SAFE_TRAJ_GEN_GENERAL_UTILS_HPP_
#define _SAFE_TRAJ_GEN_GENERAL_UTILS_HPP_
#include "oppt/plugin/PluginOptions.hpp"

namespace oppt
{


typedef VectorFloat TrajPoint;
typedef std::vector<TrajPoint> TrajData;

// Enumerations of the different kind of vectors infor used. 
// THE ORDERED ADOPTED HERE WILL BE USED AS THE CONVENTIONS THROUGHOUT THE PLUGINS

// VARIABLE ORDERS IN STATE VECTORS
enum STATE_INFO{PED_LONGIT, PED_HORIZONTAL, CAR_LONGIT, CAR_HORIZONTAL, CAR_SPEED, INTENTION};

// VARIABLE ORDERS IN CAR STATE VECTOR
enum CAR_STATE_INFO{CAR_POS_LONGIT, CAR_SPEED_LONGIT, CAR_STATE_INTENTION};

enum INTENTION_VEC_INFO{INTENTION_REL_LONGIT, INTENTION_REL_HORIZONTAL, INTENTION_VAL};

// ENUMERATED DISCRETE CAR_INTENTIONS
enum CAR_INTENTIONS{HAZARD_STOP = 0, CRUISING = 3}; 

// ENUMERATED CAR DIMENSIONS
enum CAR_DIMENSIONS{CAR_LENGTH, CAR_WIDTH, CAR_HEIGHT};

// ENUMERATED PED DIMENSIONS
enum PED_DIMENSIONS{PED_RADIUS, PED_HEIGHT};

// ENUMERATED GOAL ARES
enum GOAL_AREAS{GOAL_LONGIT, GOAL_HOZ};

// ENUMERATED POINTS FOR LOADED SAFE TRAJS
enum SAFE_TRAJ{SAFE_PED_LONGIT, SAFE_PED_HOZ};




// Options object to parse information from configuration file
class SafeTrajGenGeneralOptions: public PluginOptions
{
public:
    SafeTrajGenGeneralOptions() = default;

    virtual ~SafeTrajGenGeneralOptions() = default;


    // General variables
    std::string carLinkName = "";
    std::string pedLinkName = "";
    VectorFloat carDimensions;
    VectorFloat pedDimensions;
    VectorFloat pedSafeTrajLongit;
    VectorFloat pedSafeTrajHoz;
    VectorFloat intentionDiscretization;
    VectorFloat intentionDiscretizationLower;
    VectorFloat intentionDiscretizationUpper;


    // Goal location for the pedeinitialVelocitystrian for the plugins in the "safety case"
    VectorFloat safetyGoal;
    VectorFloat goalMargins;

    // Initial belief variables
    VectorFloat upperBound;
    VectorFloat lowerBound;
    
    // Transition variables
    std::string intentionModelFile;
    std::string dynamicsModelFile;

    FloatType processError = 0.0;
    FloatType fixedStepTime = 0.3;
    VectorUInt actionSpaceDiscretization;

    // Variables for basic controller transition
    FloatType brakingDeceleration = -3.0;
    FloatType controllerMultiplier = 1;
    FloatType fixedVelocity = 8.33;



    // Observation variables
    FloatType carObsError = 0.0;

    // Reward variables
    FloatType goalReward = 0.0;
    FloatType terminalPenalty = 0.0;
    FloatType stepPenalty = 0.0; 


    // Terminal variables
    FloatType avoidedDistance = 0.0;


    static std::unique_ptr<options::OptionParser> makeParser() {
        std::unique_ptr<options::OptionParser> parser =
            PluginOptions::makeParser();
        addGeneralPluginOptions(parser.get());
        return std::move(parser);
    }

    // Add the transition plugin options
    static void addGeneralPluginOptions(options::OptionParser* parser) {

    	/*** General options ***/
        // Typical recommended deceleration rates
        parser->addOption<FloatType>("generalOptions",
                                       "brakingDeceleration",
                                   &SafeTrajGenGeneralOptions::brakingDeceleration);

        // Controller type multiplier
        parser->addOption<FloatType>("generalOptions",
                                       "controllerMultiplier",
                                   &SafeTrajGenGeneralOptions::controllerMultiplier);

        parser->addOption<VectorFloat>("generalOptions",
                                       "goalMargins",
                                   &SafeTrajGenGeneralOptions::goalMargins);
    
        parser->addOption<std::string>("generalOptions",
                                       "carLinkName",
                                   &SafeTrajGenGeneralOptions::carLinkName);

        parser->addOption<std::string>("generalOptions",
                                       "pedLinkName",
                                   &SafeTrajGenGeneralOptions::pedLinkName);

        parser->addOption<VectorFloat>("generalOptions",
                                     "carDimensions",
                                     &SafeTrajGenGeneralOptions::carDimensions);

        parser->addOption<VectorFloat>("generalOptions",
                                     "pedDimensions",
                                     &SafeTrajGenGeneralOptions::pedDimensions);

        // Parse safety goal location
        parser->addOption<VectorFloat>("generalOptions",
                                     "safetyGoalArea",
                                     &SafeTrajGenGeneralOptions::safetyGoal);


        parser->addOption<VectorFloat>("generalOptions",
                                    "intentionDiscretization",
                                    &SafeTrajGenGeneralOptions::intentionDiscretization);

        parser->addOption<VectorFloat>("generalOptions",
                                    "intentionDiscretizationUpper",
                                    &SafeTrajGenGeneralOptions::intentionDiscretizationUpper);
        parser->addOption<VectorFloat>("generalOptions",
                                    "intentionDiscretizationLower",
                                    &SafeTrajGenGeneralOptions::intentionDiscretizationLower);


        parser->addOption<std::string>("generalOptions",
                                    "intentionModelFile",
                                    &SafeTrajGenGeneralOptions::intentionModelFile);

        parser->addOption<std::string>("generalOptions",
                                    "intentionModelFile",
                                    &SafeTrajGenGeneralOptions::intentionModelFile);



    

        /*** Initial belief options ***/
        // Lower starting bound
        parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "lowerBound",
                                       &SafeTrajGenGeneralOptions::lowerBound);
        // Upper starting bound
	     parser->addOption<VectorFloat>("initialBeliefOptions",
                                       "upperBound",
                                       &SafeTrajGenGeneralOptions::upperBound);

        /*** Transition options ***/
        parser->addOption<FloatType>("transitionPluginOptions",
                                     "processError",
                                     &SafeTrajGenGeneralOptions::processError);
	
    	parser->addOption<FloatType>("transitionPluginOptions",
                                         "fixedStepTime",
                                         &SafeTrajGenGeneralOptions::fixedStepTime);

        // Action discretization options
        VectorUInt defVec;
        parser->addOptionWithDefault<VectorUInt>("ABT",
                                                "actionDiscretization", 
                                                &SafeTrajGenGeneralOptions::actionSpaceDiscretization, defVec);


        /*** Observation Plugin options ***/
        parser->addOption<FloatType>("observationPluginOptions",
                                     "carObsError",
                                     &SafeTrajGenGeneralOptions::carObsError);



        /*** Reward Plugin options ***/
        parser->addOption<FloatType>("rewardPluginOptions",
                                         "goalReward",
                                         &SafeTrajGenGeneralOptions::goalReward);

        parser->addOption<FloatType>("rewardPluginOptions",
                                         "terminalPenalty",
                                         &SafeTrajGenGeneralOptions::terminalPenalty);

        parser->addOption<FloatType>("rewardPluginOptions",
                                         "stepPenalty",
                                         &SafeTrajGenGeneralOptions::stepPenalty);


        /*** Terminal Plugin options ***/
        parser->addOption<FloatType>("terminalPluginOptions",
                                         "avoidedDistance",
                                         &SafeTrajGenGeneralOptions::avoidedDistance);

    }

};






class SKDUserData: public RobotStateUserData {
public:
    SKDUserData():
        RobotStateUserData() {
    }

    virtual ~SKDUserData() = default;


    CollisionReportSharedPtr collisionReport = nullptr;
    size_t visitIndex = 0;
    TrajData safeTrajectory;
};









/*** HELPER FUNCTIONS ***/
// Computes the magnitude of the given vector
inline FloatType getMagnitude(VectorFloat& vec) {
    FloatType result = 0.0;

    for(auto& component : vec){
        result += (component*component);
    }
    return sqrt(result);
}



/*** Function to retrieve either the scoped(0) or unscoped(1) part of a link or collision object name ***/
std::string getScopingIndexName(std::string name, int index){
    std::string result;
        if (name.find("::") != std::string::npos) {
            VectorString nameElems;
            split(name, "::", nameElems);
            // result = nameElems[index];
            std::vector<std::string> list;
            list.push_back(nameElems[0]);
            list.push_back(nameElems[1]);

            std::string joined = boost::algorithm::join(list, "::");

            result = joined;
        }
    return result;
}


















}

#endif


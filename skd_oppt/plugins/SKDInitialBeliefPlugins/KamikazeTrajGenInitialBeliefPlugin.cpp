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
#ifndef _SKD_KAMIKAZE_INITIAL_BELIEF_PLUGIN_HPP_
#define _SKD_KAMIKAZE_INITIAL_BELIEF_PLUGIN_HPP_


#include "oppt/plugin/Plugin.hpp"
#include "../SKDDefines/KamikazeTrajGenGeneralUtils.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "oppt/opptCore/Distribution.hpp"
#include "../SKDDefines/json.hpp"

using json = nlohmann::json;



namespace oppt
{
class KamikazeTrajGenInitialBeliefPlugin: public InitialBeliefPlugin
{
public:
    KamikazeTrajGenInitialBeliefPlugin():
        InitialBeliefPlugin(),
        initialStateVec_() {
    }

    virtual ~KamikazeTrajGenInitialBeliefPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<KamikazeTrajGenGeneralOptions>(optionsFile);

        // Retrieve pointer to options file
        KamikazeTrajGenGeneralOptions* generalOptions = static_cast<KamikazeTrajGenGeneralOptions*>(options_.get());
        carLinkName_ = generalOptions->carLinkName;
        pedLinkName_ = generalOptions->pedLinkName;

        unsigned int numDimensions = robotEnvironment_->getRobot()->getStateSpace()->getNumDimensions();
        cout << "Num dim = " << numDimensions << endl;
        cout << "lower bound size " << generalOptions->lowerBound.size() << endl;
        if (generalOptions->lowerBound.size() != numDimensions)
            ERROR("Lower bound for the uniform distribution doesn't match state space dimension");
        if (generalOptions->upperBound.size() != numDimensions)
            ERROR("Upper bound for the uniform distribution doesn't match state space dimension");
        for (size_t i = 0; i != generalOptions->lowerBound.size(); ++i) {
            if (generalOptions->lowerBound[i] > generalOptions->upperBound[i])
                ERROR("Lower bound for initial belief must be smaller than upper bound");
        }

        // Random engine for distributions
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        uniformDistribution_ =
            std::make_unique<UniformDistribution<FloatType>>(generalOptions->lowerBound, generalOptions->upperBound, randomEngine);


        // Load safe trajectories from file
        std::ifstream safeTrajFile(generalOptions->safeTrajFilePath);
        json safeTrajsJson = json::parse(safeTrajFile);
        safeTraj_ = safeTrajsJson[generalOptions->safeTrajKey].get<TrajData>();

        for(auto& point : safeTraj_){
            printVector(point, "CURRENT POINT");
        }
    
        // populate link map for agents in the environment
        populateLinkMap();

        return true;
    }

    virtual RobotStateSharedPtr sampleAnInitState() override {
         // Sample an initial state vector
        //VectorFloat initialStateVec = toStdVec<FloatType>(uniformDistribution_->sample(1).col(0));
        const FloatType initialIntention = CAR_INTENTIONS::CRUISING;
        const FloatType initialSpeed = 0;
        // Initial locations
        VectorFloat pedInit = safeTraj_[0];
        // Fix an initial state according to the real life data estimate
        VectorFloat initialStateVec{pedInit[REAL_TRAJ_DATA::TRAJ_LONGIT], pedInit[REAL_TRAJ_DATA::TRAJ_HOZ],
                                100, -1.7, 
                                initialSpeed, initialIntention};

        // Set the corresponding pose for the links in the environment according to state vector
        gazebo::physics::Link* carLink = linkMapInit_.at(carLinkName_);
        gazebo::physics::Link* pedLink = linkMapInit_.at(pedLinkName_);
        
        #ifdef GZ_GT_7
        GZPose carPose = carLink->WorldPose();
        GZPose pedPose = pedLink->WorldPose();
        // set the pose structure with updated information
        carPose.Pos().X(initialStateVec[STATE_INFO::CAR_LONGIT]);
        carPose.Pos().Y(initialStateVec[STATE_INFO::CAR_HORIZONTAL]);

        pedPose.Pos().X(initialStateVec[STATE_INFO::PED_LONGIT]);
        pedPose.Pos().Y(initialStateVec[STATE_INFO::PED_HORIZONTAL]);

        #else
        GZPose carPose = carLink->GetWorldPose();
        GZPose pedPose = pedLink->GetWorldPose();
        // set the pose structure with updated information
        carPose.pos.x = initialStateVec[STATE_INFO::CAR_LONGIT];
        carPose.pos.y = initialStateVec[STATE_INFO::CAR_HORIZONTAL];

        pedPose.pos.x = initialStateVec[STATE_INFO::PED_LONGIT];
        pedPose.pos.y = initialStateVec[STATE_INFO::PED_HORIZONTAL];
        #endif

        // update gazebo pose for both links
        carLink->SetWorldPose(carPose);
        pedLink->SetWorldPose(pedPose);


        // First construct an intial world state based in the intial state vector
        robotEnvironment_->getGazeboInterface()->makeInitialWorldState(initialStateVec, false);

        // Then get the initial world state
        GazeboWorldStatePtr initialWorldState_ = robotEnvironment_->getGazeboInterface()->getInitialWorldState();


        RobotStateSharedPtr initialState = std::make_shared<VectorState>(initialStateVec);
        initialState->setGazeboWorldState(initialWorldState_);


        // Populate with user data
        RobotStateUserDataSharedPtr userData(new SKDUserData());

        // User Data
        auto nextUserData = static_cast<SKDUserData*>(userData.get());
        // Default collision report for init states
        nextUserData->collisionReport =
            std::make_shared<CollisionReport>();

        // Set starting visit index
        nextUserData->visitIndex = 0;

        // Set safe trajectory
        nextUserData->safeTrajectory = safeTraj_;

        initialState->setUserData(userData);

        return initialState;
    }

private:


    // Helper functions
    // Create a map to the different useful joints of the robot arms and populate the list
    void populateLinkMap() {
        // Populate the joint map with their correspoding pointers
        std::vector<gazebo::physics::Link*> links = robotEnvironment_->getGazeboInterface()->getLinks();
        for (auto& currentLink : links) {
            linkMapInit_.insert(std::pair<std::string, gazebo::physics::Link*>(currentLink->GetName(), currentLink));
        }
    }


private:

    VectorFloat initialStateVec_;

    std::unique_ptr<Distribution<FloatType>> uniformDistribution_;

    // Unordered map of links
    std::unordered_map<std::string, gazebo::physics::Link*> linkMapInit_;

    // Car Link name
    std::string carLinkName_;
    std::string pedLinkName_;

    // Safe Traj
    TrajData safeTraj_;

};

OPPT_REGISTER_INITIAL_BELIEF_PLUGIN(KamikazeTrajGenInitialBeliefPlugin)

}

#endif

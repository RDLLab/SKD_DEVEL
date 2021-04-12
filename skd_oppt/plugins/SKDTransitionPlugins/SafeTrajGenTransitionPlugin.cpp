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
#include "oppt/opptCore/Distribution.hpp"
#include "oppt/gazeboInterface/GazeboInterface.hpp"
#include "../SKDDefines/SafeTrajGenGeneralUtils.hpp"
#include "SKDActionSpaceDiscretizer.hpp"
#include <boost/timer.hpp>
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"
#include <random>
#include <mlpack/core.hpp>
#include <mlpack/methods/neighbor_search/neighbor_search.hpp>
#include <chrono>


using std::cout;
using std::endl;
using namespace mlpack;
using namespace mlpack::neighbor; // NeighborSearch and NearestNeighborSort
using namespace mlpack::metric; // ManhattanDistance


namespace oppt
{
class SafeTrajGenTransitionPlugin: public TransitionPlugin
{
public:
    SafeTrajGenTransitionPlugin():
         TransitionPlugin() {
    }

    virtual ~SafeTrajGenTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        // Parse options files
        parseOptions_<SafeTrajGenGeneralOptions>(optionsFile);
        generalOptions_ = static_cast<SafeTrajGenGeneralOptions*>(options_.get());

        // Std deviation from gaussian acceleration 
        processError_  = generalOptions_->processError;
        fixedStepTime_ = generalOptions_->fixedStepTime;
        carLinkName_ = generalOptions_->carLinkName;
        pedLinkName_ = generalOptions_->pedLinkName;
        carDimensions_ = generalOptions_->carDimensions;
        pedDimensions_ = generalOptions_->pedDimensions;  


        // // Overwrite action space with custom one for individual actions
        auto actionSpace = robotEnvironment_->getRobot()->getActionSpace();
        auto actionSpaceDiscretization = generalOptions_->actionSpaceDiscretization;
        if (actionSpaceDiscretization.size() == 0)
            ERROR("Size of action space discretization must be greater than 0");
        std::shared_ptr<ActionSpaceDiscretizer> skdActionSpaceDiscretizer(new SKDActionSpaceDiscretizer(actionSpace, actionSpaceDiscretization));
        actionSpace->setActionSpaceDiscretizer(skdActionSpaceDiscretizer);


        // Populate map and generate error distributions
        populateLinkMap();
       

        // Setup knn structure to lookup intentions
        std::string intentions_model_file = generalOptions_->intentionModelFile;
        std::string transition_dynamics_file = generalOptions_->dynamicsModelFile;
        std::cout << "PRINTING PATHS" << std::endl;
        std::cout << intentions_model_file << std::endl;
        std::cout << transition_dynamics_file << std::endl;
    
        data::Load(intentions_model_file, intentionDatabase_, true);
        // Train on subdatabase
        auto& intentionDBSizes = size(intentionDatabase_);
        arma::mat subIntention = intentionDatabase_.submat(0,0, intentionDBSizes[0]-2, intentionDBSizes[1] - 1);
        //Use templates to specify that we want a NeighborSearch object which uses Manhattan Dist
        intentionNN_ = NeighborSearch<NearestNeighborSort, ManhattanDistance>(subIntention);


        // Setup knn structure to lookup dynamics
        data::Load(transition_dynamics_file, dynamicsDatabase_, true);
        // Train on subdatabase
        auto& dynamicsDBSizes = size(dynamicsDatabase_);
        arma::mat subDynamics = dynamicsDatabase_.submat(0,0, dynamicsDBSizes[0]-3, dynamicsDBSizes[1] - 1);
        dynamicsNN_ = NeighborSearch<NearestNeighborSort, ManhattanDistance>(subDynamics);

       
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // Copy information from propagationRequest to propagationResult
        PropagationResultSharedPtr propagationResult(new PropagationResult());
        propagationResult->previousState = propagationRequest->currentState.get();
        propagationResult->action = propagationRequest->action;
        // User Data
        auto currentStateUserData =
            static_cast<SKDUserData*>(propagationRequest->currentState->getUserData().get());
       
        // Extract information from propagationRequest as vectors
        VectorFloat actionApplied = propagationRequest->action->as<VectorAction>()->asVector();
        VectorFloat stateVector = propagationRequest->currentState->as<VectorState>()->asVector();



        // Create current car state vector
        VectorFloat currentIntentionVec{
            stateVector[STATE_INFO::PED_LONGIT] - stateVector[STATE_INFO::CAR_LONGIT], 
            stateVector[STATE_INFO::PED_HORIZONTAL] - stateVector[STATE_INFO::CAR_HORIZONTAL]};
        
        // Infer current intention of the controller
        FloatType currentCarIntention = estimateCurrentCarIntention(currentIntentionVec);

        //Current state vector of the car
        VectorFloat currentCarState{stateVector[STATE_INFO::CAR_LONGIT], stateVector[STATE_INFO::CAR_SPEED], currentCarIntention};

        // Propagate car to the next state
        VectorFloat nextCarState = propagateCarInfo(currentCarState);
        

        // // Propagate for two more steps to simulate a single plannin step time in transition
        for(size_t propagations = 0; propagations < 2; ++propagations){
            nextCarState = propagateCarInfo(nextCarState);
        }


        // Container to store result
        VectorFloat resultingState(stateVector);

        // Update components of the resulting vector
        // Sample from uniform distribution to make the transition on the intention value
        unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed1); //gen(time(NULL))
       // std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(-1.6, -1.8);

        resultingState[STATE_INFO::PED_LONGIT] = stateVector[STATE_INFO::PED_LONGIT] + (actionApplied[STATE_INFO::PED_LONGIT] * fixedStepTime_);
        // Clamp position of pedestrian on horizontal axis
        resultingState[STATE_INFO::PED_HORIZONTAL] = 
                    clamp(stateVector[STATE_INFO::PED_HORIZONTAL] + (actionApplied[STATE_INFO::PED_HORIZONTAL] * fixedStepTime_), 
                        3.75, -4.5);
        resultingState[STATE_INFO::CAR_LONGIT] = nextCarState[CAR_STATE_INFO::CAR_POS_LONGIT];

        resultingState[STATE_INFO::CAR_HORIZONTAL] = (FloatType) distribution(generator);
        resultingState[STATE_INFO::CAR_SPEED] = nextCarState[CAR_STATE_INFO::CAR_SPEED_LONGIT];

        // Change the car intention in the end
        VectorFloat nextIntentionVec{
            resultingState[STATE_INFO::PED_LONGIT] - resultingState[STATE_INFO::CAR_LONGIT], 
            resultingState[STATE_INFO::PED_HORIZONTAL] - resultingState[STATE_INFO::CAR_HORIZONTAL]};
        resultingState[STATE_INFO::INTENTION] = estimateCurrentCarIntention(nextIntentionVec);


        // Update the link poses in the gazebo world
        updateGazeboPoses(resultingState);
        
        // Populate the rest of the result structure
        // Create a robotState object from resulting state
        RobotStateSharedPtr nextRobotState = std::make_shared<oppt::VectorState>(resultingState);
        propagationResult->nextState = nextRobotState;

        // Save the current states in gazebo
        robotEnvironment_->getGazeboInterface()->setStateVector(resultingState);
        auto newWorldState = robotEnvironment_->getGazeboInterface()->getWorldState();   
        propagationResult->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));

        // Save user data
        // // Check for collision between the robot (DUBIN) and the pedestrian (Cylinder)
        propagationResult->collisionReport = std::make_shared<CollisionReport>();
        propagationResult->collisionReport->collides = checkRoadCollision(resultingState);
        // Populate with user data
        RobotStateUserDataSharedPtr userData(new SKDUserData());
        
        auto nextStateUserData = static_cast<SKDUserData*>(userData.get());
        // Default collision report for init states
        nextStateUserData->collisionReport = std::make_shared<CollisionReport>();
        nextStateUserData->collisionReport->collides = checkRoadCollision(resultingState);
        nextStateUserData->visitIndex = 
            // Cap index to last trajectory point since its a terminal point
            clamp(currentStateUserData->visitIndex + 1, 0, ((currentStateUserData->safeTrajectory).size() - 1));

        propagationResult->nextState->setUserData(userData);

        //std::cout << "visit index in this query was " << nextStateUserData->visitIndex << std::endl;
  
        return propagationResult;
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }





private:

    // Updates the gazebo poses of the links according to the informaiton in the state vec
    void updateGazeboPoses(const VectorFloat& stateVector) const{
        // Set the corresponding pose for the links in the environment according to state vector
        gazebo::physics::Link* carLink = linkMapTrans_.at(carLinkName_);
        gazebo::physics::Link* pedLink = linkMapTrans_.at(pedLinkName_);

        #ifdef GZ_GT_7
        GZPose carPose = carLink->WorldPose();
        GZPose pedPose = pedLink->WorldPose();

        // set the pose structure with updated information
        carPose.Pos().X(stateVector[STATE_INFO::CAR_LONGIT]);
        carPose.Pos().Y(stateVector[STATE_INFO::CAR_HORIZONTAL]);

        pedPose.Pos().X(stateVector[STATE_INFO::PED_LONGIT]);
        pedPose.Pos().Y(stateVector[STATE_INFO::PED_HORIZONTAL]);

        #else
        GZPose carPose = carLink->GetWorldPose();
        GZPose pedPose = pedLink->GetWorldPose();

        // set the pose structure with updated information
        carPose.pos.x = stateVector[STATE_INFO::CAR_LONGIT];
        carPose.pos.y = stateVector[STATE_INFO::CAR_HORIZONTAL];

        pedPose.pos.x = (stateVector[STATE_INFO::PED_LONGIT]);
        pedPose.pos.y = (stateVector[STATE_INFO::PED_HORIZONTAL]);
        #endif

        // update gazebo pose for both links
        carLink->SetWorldPose(carPose);
        pedLink->SetWorldPose(pedPose);
    }


    // Create a map to the different useful joints of the robot arms and populate the lsits of joint limits
    void populateLinkMap() {
        // Populate the joint map with their correspoding pointers
        std::vector<gazebo::physics::Link*> links = robotEnvironment_->getGazeboInterface()->getLinks();
        for (auto& currentLink : links) {
            linkMapTrans_.insert(std::pair<std::string, gazebo::physics::Link*>(currentLink->GetName(), currentLink));
        }
    }





    VectorFloat propagateCarInfo(const VectorFloat& carState) const {
        double throttleVal = 0.8;
        double brakeVal = 0.0;
        double carStepTime = 0.1;

        // Check if intention is hazard stop
        if(carState[CAR_STATE_INFO::CAR_STATE_INTENTION] == CAR_INTENTIONS::HAZARD_STOP){
            throttleVal = 0.0;
            brakeVal = 1.0;
        }


        // Add error to the speed
        FloatType currentSpeed =  carState[CAR_STATE_INFO::CAR_SPEED_LONGIT];
        // Update components of the resulting vector
        // Sample from uniform distribution to make the transition on the intention value
        unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator1(seed1); //gen(time(NULL))
       // std::default_random_engine generator;
        std::uniform_real_distribution<double> queryVelDist(currentSpeed * -0.2, 0);
        FloatType speedError = queryVelDist(generator1);


        // Query database to test
        // Create the object we will store the nearest neighbors in.
        arma::Mat<size_t> neighbors;
        arma::mat distances; // We need to store the distance too.
        std::vector<double> nextDynamics{double(carState[CAR_STATE_INFO::CAR_SPEED_LONGIT] + speedError), throttleVal, brakeVal};

        //printVector(nextDynamics, "QUERY VEC");
        arma::mat queryVec(nextDynamics);
        arma::mat dynamicsDatabaseT = dynamicsDatabase_.t();

       
        // Compute the nearest neighbour
        const_cast<NeighborSearch<NearestNeighborSort, ManhattanDistance>*>(&dynamicsNN_)->Search(queryVec, 1, neighbors, distances);
        // std::cout << "QUERYING FOR DYNAMICS" << std::endl;
        // queryVec.t().print("B");
       

        // Copy best Match to vector
        arma::mat matchingLookup = dynamicsDatabaseT.row(neighbors[0]);

        // Add the predicted difference to the current one
        FloatType deltaCarLongit =  matchingLookup[3];
        FloatType deltaCarVel = matchingLookup[4];
        //std::cout << "answer was " << matchingLookup << std::endl;
        // getchar();
        // std::cout << "PRO WAS" << nextVelVal << std::endl;


        FloatType nextCarLongit = carState[CAR_STATE_INFO::CAR_POS_LONGIT] + deltaCarLongit;
        FloatType nextCarVel = clamp(carState[CAR_STATE_INFO::CAR_SPEED_LONGIT] + deltaCarVel, 0, 8.33);



        // Create vector for result                                
        VectorFloat nextCarState{nextCarLongit, nextCarVel, carState[CAR_STATE_INFO::CAR_STATE_INTENTION]};

        // // Debug here
        

        return nextCarState;

    }



    FloatType estimateCurrentCarIntention(const VectorFloat& intentionStateVec) const{
        // Estimate intention as cruising by default
        FloatType currentIntentionEstimate = CAR_INTENTIONS::HAZARD_STOP;
        VectorFloat intentionDisc = generalOptions_->intentionDiscretization;
        VectorFloat intentionDiscUpper = generalOptions_->intentionDiscretizationUpper;
        VectorFloat intentionDiscLower = generalOptions_->intentionDiscretizationLower;
        FloatType relLongitBinSize = std::abs(intentionDiscUpper[0] - intentionDiscLower[0])/intentionDisc[0];
        FloatType relHozBinSize = std::abs(intentionDiscUpper[1] - intentionDiscLower[1])/intentionDisc[1];
        // Transpose of nearest neighbour for intention data
        arma::mat intentionDataT = intentionDatabase_.t();
        // Perform binning of the currnet statevec
        FloatType stateRelLongitudinal = std::floor(
                (intentionStateVec[INTENTION_VEC_INFO::INTENTION_REL_LONGIT] - intentionDiscLower[0]) 
                    / relLongitBinSize);
        FloatType stateRelHorizontal = std::floor(
                (intentionStateVec[INTENTION_VEC_INFO::INTENTION_REL_HORIZONTAL] - intentionDiscLower[1]) 
                    / relHozBinSize);

        // Query database to test
        // Create the object we will store the nearest neighbors in.
        arma::Mat<size_t> cruisingNeighbours;
        arma::mat cruisingDistances; 
        // We need to store the distance too.
        std::vector<double> cruisingVec{stateRelLongitudinal, stateRelHorizontal, CAR_INTENTIONS::CRUISING};
        arma::mat cruisingQuery(cruisingVec);

        // Create the object we will store the nearest neighbors in.
        arma::Mat<size_t> hazardNeighbours;
        arma::mat hazardDistances; 
        // We need to store the distance too.
        std::vector<double> hazardVec{stateRelLongitudinal, stateRelHorizontal, CAR_INTENTIONS::HAZARD_STOP};
        arma::mat hazardQuery(hazardVec);
       


       
        // Compute the nearest neighbour
        const_cast<NeighborSearch<NearestNeighborSort, ManhattanDistance>*>(&intentionNN_)->Search(cruisingQuery, 1, cruisingNeighbours, cruisingDistances);
        const_cast<NeighborSearch<NearestNeighborSort, ManhattanDistance>*>(&intentionNN_)->Search(hazardQuery, 1, hazardNeighbours, hazardDistances);
        // std::cout << "QUERYING FOR INTENTION" << std::endl;
        // queryVec.t().print("A");
       

        // Copy best Match to vector
        arma::mat matchingLookupCruising = intentionDataT.row(cruisingNeighbours[0]);
        arma::mat matchingLookupHazard = intentionDataT.row(hazardNeighbours[0]);

        // Add the predicted difference to the current one
        FloatType cruisinIntentionProbability =  matchingLookupCruising[3];
        FloatType hazardIntentionProbability =  matchingLookupHazard[3];

        // // Sample
        // unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
        // std::default_random_engine generator(seed1); //gen(time(NULL))
        // // std::default_random_engine generator;
        // std::uniform_real_distribution<double> distribution(0, 1);
        // FloatType sample = distribution(generator);


        // if(hazardIntentionProbability > 0.1){
        //     std::cout << "CHANGING TO DANGER "  << hazardIntentionProbability << " "  << cruisinIntentionProbability << std::endl;
        //     printVector(intentionStateVec, "DANGER VEC");
        //     currentIntentionEstimate = CAR_INTENTIONS::HAZARD_STOP;
        // } else
        if (cruisinIntentionProbability > 0.8) {
            currentIntentionEstimate = CAR_INTENTIONS::CRUISING;
            //std::cout << "CHANGING TO CRUISING" << std::endl;
        }

        return currentIntentionEstimate;
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



    // Local collision checking function to bypass time incur in Gazebo and fcl api calls
    bool checkRoadCollision(const VectorFloat& checkStateVec) const{
        VectorFloat pedLocation{checkStateVec[STATE_INFO::PED_LONGIT], checkStateVec[STATE_INFO::PED_HORIZONTAL]};
        VectorFloat carLocation{checkStateVec[STATE_INFO::CAR_LONGIT], checkStateVec[STATE_INFO::CAR_HORIZONTAL]};

        // Relative distances on both directions
        FloatType longitudinalDist = checkStateVec[STATE_INFO::PED_LONGIT] - (checkStateVec[STATE_INFO::CAR_LONGIT]);
        FloatType transverseDist = checkStateVec[STATE_INFO::PED_HORIZONTAL] - (checkStateVec[STATE_INFO::CAR_HORIZONTAL]);

        // Get closest vertical edge of car to pedestrian
        FloatType closestCarLongit = clamp(checkStateVec[STATE_INFO::CAR_LONGIT] + longitudinalDist,
                                checkStateVec[STATE_INFO::CAR_LONGIT] - carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2,
                                checkStateVec[STATE_INFO::CAR_LONGIT] + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2);

        // Get closest horizontal edge of car to pedestrian
        FloatType closestCarHoz = clamp(checkStateVec[STATE_INFO::CAR_HORIZONTAL] + transverseDist,
                                checkStateVec[STATE_INFO::CAR_HORIZONTAL] - carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2,
                                checkStateVec[STATE_INFO::CAR_HORIZONTAL] + carDimensions_[CAR_DIMENSIONS::CAR_WIDTH]/2);


        // Get closest point within car to center of pedestrian
        VectorFloat closestPoint{closestCarLongit, closestCarHoz};
        VectorFloat diffVec{checkStateVec[STATE_INFO::PED_LONGIT] - closestCarLongit, 
                            checkStateVec[STATE_INFO::PED_HORIZONTAL] - closestCarHoz};

        FloatType distanceS = 0;
        for(auto& val : diffVec){
            distanceS += (val * val);
        } 


        // Check if norm of diffVec is greater than radius
        if(sqrt(distanceS) <= pedDimensions_[PED_DIMENSIONS::PED_RADIUS]){
            return true;
        }


        // If greater than radius, no collision
        return false;
    }

private:
    // Member variables
    std::unique_ptr<Distribution<FloatType>> errorDistribution_;
    // General options variables
    SafeTrajGenGeneralOptions* generalOptions_;

    FloatType processError_ = 0.0;

    FloatType fixedStepTime_ = 0.0;

    FloatType carFOV_ = 0.0;

    FloatType dangerThresh_ = 0.0;

    FloatType speedLimit_ = 0.0;

    FloatType emergencyRate_ = 0.0;

    FloatType accelerationRate_ = 0.0;

    FloatType slowDownRate_ = 0.0;

    std::string carLinkName_ ;
    std::string pedLinkName_ ;

    VectorFloat carDimensions_;
    VectorFloat pedDimensions_;

    // Variables for intention database
    arma::mat intentionDatabase_;

    // Variable for dynamics database
    arma::mat dynamicsDatabase_;

    // Intention database
    NeighborSearch<NearestNeighborSort, ManhattanDistance> intentionNN_;
    NeighborSearch<NearestNeighborSort, ManhattanDistance> dynamicsNN_;

    std::unordered_map<std::string, gazebo::physics::Link*> linkMapTrans_;
    // Unorder map collision Objects associated with each link
    std::unordered_map<std::string, OpptCollisionObject*> collisionLinkMap_;





    


};

OPPT_REGISTER_TRANSITION_PLUGIN(SafeTrajGenTransitionPlugin)

}

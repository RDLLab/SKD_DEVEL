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
#include "../SKDDefines/KamikazeTrajGenGeneralUtils.hpp"
#include "SKDActionSpaceDiscretizer.hpp"
#include <boost/timer.hpp>
#include "oppt/opptCore/CollisionObject.hpp"
#include "oppt/opptCore/CollisionRequest.hpp"
#include <random>
#include <chrono>


using std::cout;
using std::endl;


namespace oppt
{
class KamikazeTrajGenTransitionPlugin: public TransitionPlugin
{
public:
    KamikazeTrajGenTransitionPlugin():
         TransitionPlugin() {
    }

    virtual ~KamikazeTrajGenTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        // Parse options files
        parseOptions_<KamikazeTrajGenGeneralOptions>(optionsFile);
        generalOptions_ = static_cast<KamikazeTrajGenGeneralOptions*>(options_.get());

        // Calculate relevant information for specified controller
        brakingDeceleration_ = generalOptions_->brakingDeceleration;
        controllerMultiplier_ = generalOptions_->controllerMultiplier;

        cout << "################### Controller multiplier value was ###############" << controllerMultiplier_ << endl;
        fixedVelocity_ = generalOptions_->fixedVelocity;

         // Std deviation from gaussian acceleration 
        processError_  = generalOptions_->processError;
        fixedStepTime_ = generalOptions_->fixedStepTime;
        carLinkName_ = generalOptions_->carLinkName;
        pedLinkName_ = generalOptions_->pedLinkName;
        carDimensions_ = generalOptions_->carDimensions;   
        pedDimensions_ = generalOptions_->pedDimensions;    
        
        // Compute threshold distance at which the controller will decide to break
        FloatType stoppingTime = std::abs(fixedVelocity_ / brakingDeceleration_);

        // This is the base (multiplier == 1) desired stopping distance described in # Car_BEHAVIOUR.md
        baseStoppingDist_ = 
            // Stopping distance for car
            (fixedVelocity_ * stoppingTime) + (0.5 * brakingDeceleration_ * stoppingTime * stoppingTime) 
            // Added padding to avoid collision in the absence of control error
            + carDimensions_[CAR_DIMENSIONS::CAR_LENGTH]/2  + (pedDimensions_[PED_DIMENSIONS::PED_RADIUS])
            //whole step padding
            + (fixedStepTime_ * fixedVelocity_);



        // // Overwrite action space with custom one for individual actions
        auto actionSpace = robotEnvironment_->getRobot()->getActionSpace();
        auto actionSpaceDiscretization = generalOptions_->actionSpaceDiscretization;
        if (actionSpaceDiscretization.size() == 0)
            ERROR("Size of action space discretization must be greater than 0");
        std::shared_ptr<ActionSpaceDiscretizer> skdActionSpaceDiscretizer(new SKDActionSpaceDiscretizer(actionSpace, actionSpaceDiscretization));
        actionSpace->setActionSpaceDiscretizer(skdActionSpaceDiscretizer);

        // Populate map and generate error distributions
        populateLinkMap();

        
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // Constant 
        FloatType const MAX_SPEED = 2.5;
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


        // Construct resultingvector based on previous vector
        VectorFloat resultingState(stateVector);

        // Populate resulting state for information from the next state
        // Get the current car intention based on the current distance between the pedestian and the controller
        CAR_INTENTIONS currentIntention = getCurrentIntention(stateVector);
         // Cache the value of the acceleration for now ( default to constant velocity)
        FloatType currentAcceleration = 0;

        // Update the kinematics information of the resulting state based on intention
        if(currentIntention == CAR_INTENTIONS::HAZARD_STOP){
            FloatType brakingError = 0;
            currentAcceleration = brakingDeceleration_;


            // // Add error if execution component to create 20% error in transition
            if(robotEnvironment_->isExecutionEnvironment()){
               // Sample an acceleration error
                brakingError = getUniformDistSample(-0.1 * std::abs(brakingDeceleration_), 0.1 * std::abs(brakingDeceleration_));
            }

            currentAcceleration += brakingError;

            // Change current velocity value according to vf = vo + a*t
            resultingState[STATE_INFO::CAR_SPEED] = clamp(stateVector[STATE_INFO::CAR_SPEED] + (currentAcceleration * fixedStepTime_), 0, fixedVelocity_) ;
        } else{
            // Keep driving at a constant speed
            resultingState[STATE_INFO::CAR_SPEED] = fixedVelocity_;
            currentAcceleration = 0;
        }

        // Add error to the car speed
        FloatType speedError = getUniformDistSample(-0.05 * resultingState[STATE_INFO::CAR_SPEED], 0.05 * resultingState[STATE_INFO::CAR_SPEED]);
        //FloatType speedError = 0;
        resultingState[STATE_INFO::CAR_SPEED] = clamp(resultingState[STATE_INFO::CAR_SPEED] + speedError, 0, fixedVelocity_) ;


    
        // Clip the speed and deceleration to 0
        if(stateVector[STATE_INFO::CAR_SPEED] <= 0){
            currentAcceleration = 0;
        }
 
        // Update car longitudinal location based on the velocity ( have to decide if current one or lag one step.)
        // Use the currrent speed of the car to update its postion according to the kinematics  dS=  vot + 1/2 a(t^2)
        resultingState[STATE_INFO::CAR_LONGIT] = stateVector[STATE_INFO::CAR_LONGIT]
                                                    + (stateVector[STATE_INFO::CAR_SPEED] * fixedStepTime_)
                                                        + (0.5 * fixedStepTime_ * fixedStepTime_ * currentAcceleration);


        //Update pedestrian location according to best action taken 
        resultingState[STATE_INFO::PED_LONGIT] = stateVector[STATE_INFO::PED_LONGIT] + actionApplied[STATE_INFO::PED_LONGIT] * fixedStepTime_;
        resultingState[STATE_INFO::PED_HORIZONTAL] = stateVector[STATE_INFO::PED_HORIZONTAL] + actionApplied[STATE_INFO::PED_HORIZONTAL] * fixedStepTime_;
      
        // Update the intention information from the resulting state vector
        resultingState[STATE_INFO::INTENTION] = currentIntention;
        
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
        // Check for collision between the robot (DUBIN) and the pedestrian (Cylinder)
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
        nextStateUserData->safeTrajectory = currentStateUserData->safeTrajectory;


        propagationResult->nextState->setUserData(userData);
  
        return propagationResult;
    }

    virtual Distribution<FloatType>* const getErrorDistribution() const override {
        return errorDistribution_.get();
    }


private:

    std::unique_ptr<Distribution<FloatType>> errorDistribution_;

    // General options variables
    KamikazeTrajGenGeneralOptions* generalOptions_;

    FloatType processError_ = 0.0;

    FloatType fixedStepTime_ = 0.0;

    FloatType baseStoppingDist_ = 0.0;

    FloatType brakingDeceleration_ = 0.0;

    FloatType controllerMultiplier_ = 1.0;

    FloatType fixedVelocity_ = 0.0;
    // Stopping distance of this contorller base on controller multiplier
    FloatType stoppingDistance = 0.0;

    std::string carLinkName_ ;
    std::string pedLinkName_ ;

    VectorFloat carDimensions_;
    VectorFloat pedDimensions_;

    std::unordered_map<std::string, gazebo::physics::Link*> linkMapTrans_;
    // Unorder map collision Objects associated with each link
    std::unordered_map<std::string, OpptCollisionObject*> collisionLinkMap_;

    // Function pointers to custom defined collision functions of interest
    /**
     * @brief Function that is used to determine if the knuckle link collides
     * with the object we want to grasp. This is a requirement for a successful grasp
     */
    std::function<bool ()> carCollisionFunction_ = nullptr;





private:

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


    // Get the current intention of the car
    CAR_INTENTIONS getCurrentIntention(const VectorFloat& stateVector) const{
        // Default intention result. Keep Driving
        CAR_INTENTIONS resultedIntention =  CAR_INTENTIONS::CRUISING;
        // Check the current relative positioning between the car and the pedestrian to chekc if 
        // stopiing distance has been met.
        FloatType longDist = stateVector[STATE_INFO::PED_LONGIT] - stateVector[STATE_INFO::CAR_LONGIT];
        FloatType stoppingThreshDist = baseStoppingDist_ * controllerMultiplier_;


        // // Start breaking only after longDist is smaller
        if(longDist <= stoppingThreshDist){
            resultedIntention = CAR_INTENTIONS::HAZARD_STOP;
        }


        return resultedIntention;
    }


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


    // Gets a sample from a uniform distribution with the support [lower, upper]
    FloatType getUniformDistSample(FloatType lower, FloatType upper) const{

         // Update components of the resulting vector
            // Sample from uniform distribution to make the transition on the intention value
            unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed1); //gen(time(NULL))
           // std::default_random_engine generator;
            std::uniform_real_distribution<FloatType> distribution(lower, upper);
            FloatType sample = (FloatType) distribution(generator);

            // std::cout << " SAMPLE FOR LOWER " << lower << "AND UPPER" << upper << "WAS " << sample << std::endl;
            return sample;
    }





};

OPPT_REGISTER_TRANSITION_PLUGIN(KamikazeTrajGenTransitionPlugin)

}

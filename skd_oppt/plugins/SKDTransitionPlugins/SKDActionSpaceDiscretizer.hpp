#ifndef _SKD_ACTION_SPACE_DISCRETIZER_
#define _SKD_ACTION_SPACE_DISCRETIZER_
#include "oppt/robotHeaders/ActionSpaceDiscretizer.hpp"

namespace oppt {

class SKDActionSpaceDiscretizer: public CustomActionSpaceDiscretizer {
public:
	SKDActionSpaceDiscretizer(ActionSpaceSharedPtr &actionSpace,
	                           const std::vector<unsigned int>& discretization):
		CustomActionSpaceDiscretizer(actionSpace, discretization) {
			LOGGING("Inside constructor");

	}

	virtual ~SKDActionSpaceDiscretizer() {}

	virtual std::vector<ActionSharedPtr> getAllActionsInOrder(const unsigned int& numStepsPerDimension) const {		
		std::vector<ActionSharedPtr> allActionsOrdered_;
		const float MAX_SPEED = 2.5; // Max speed a pedestrian can attain
		int DISCRETIZATION_NUM = 1;
		VectorFloat directions({-1.0, 1.0});
		long code = 0;


		// Append single horizontal action
		VectorFloat westActionVals(2,0);
		westActionVals[0] = (MAX_SPEED) * directions[0];
		ActionSharedPtr westAction(new DiscreteVectorAction(westActionVals));
		westAction->as<DiscreteVectorAction>()->setBinNumber(code);
		code++;
		allActionsOrdered_.push_back(westAction);

		// Populate all the vertical direction actions
		for(int speedDivider = 1 ; speedDivider <= DISCRETIZATION_NUM; ++speedDivider){
			for(auto& direction : directions){
				VectorFloat actionVals(2,0);
				actionVals[1] = (MAX_SPEED/speedDivider) * direction;
				ActionSharedPtr newAction(new DiscreteVectorAction(actionVals));
				newAction->as<DiscreteVectorAction>()->setBinNumber(code);
				code++;
				allActionsOrdered_.push_back(newAction);

			}
		}


		// Populate diagonal movements to the right of the environment
		const float MAX_DIAGONAL_SIDES = 1.77; // sin(45 deg) * MAX_SPEED = 2.5
		// Append north west action 
		VectorFloat northWestVals(2,0);
		northWestVals[0] = MAX_DIAGONAL_SIDES * directions[1];
		northWestVals[1] = MAX_DIAGONAL_SIDES * directions[0];
		ActionSharedPtr northWest(new DiscreteVectorAction(northWestVals));
		northWest->as<DiscreteVectorAction>()->setBinNumber(code);
		code++;
		allActionsOrdered_.push_back(northWest);

		// Append south west action according to gazebo system
		VectorFloat southWestVals(2,0);
		southWestVals[0] = MAX_DIAGONAL_SIDES * directions[0];
		southWestVals[1] = MAX_DIAGONAL_SIDES * directions[0];
		ActionSharedPtr southWest(new DiscreteVectorAction(southWestVals));
		southWest->as<DiscreteVectorAction>()->setBinNumber(code);
		code++;
		allActionsOrdered_.push_back(southWest);



		// Add the no movement action for the pedestrian
		VectorFloat noMovementActionVals(2,0);
		ActionSharedPtr noMovementAction(new DiscreteVectorAction(noMovementActionVals));
		noMovementAction->as<DiscreteVectorAction>()->setBinNumber(code);
		code++;
		allActionsOrdered_.push_back(noMovementAction);	



		// Print out list of actions to verify correctness
		// // Print actions
		for(auto actions : allActionsOrdered_){
			actions->print(cout);
			std::cout << std::endl;
		}
		return allActionsOrdered_;
	}
};

}

#endif
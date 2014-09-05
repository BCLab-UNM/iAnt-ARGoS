#include "iAnt_loop_functions.h"

// (constructor) initialize class variables, Init() function contains further setup
iAnt_loop_functions::iAnt_loop_functions() :
	floorEntity(NULL),
	RNG(NULL),
	tick(0)
{}

void iAnt_loop_functions::Init(TConfigurationNode& node) {
	// initialize iAntData to values set in the XML configuration file
	iAntData.Init(node);

	// initialize floor object where food and nest objects are drawn
    floorEntity = &GetSpace().GetFloorEntity();

    size_t foodItemCount;

    // set XML parameters to variables
	GetNodeAttribute(GetNode(node, "simulation_settings"), "foodItemCount", foodItemCount);

    // create a random number generator for random food item placement
    RNG = CRandom::CreateRNG("argos");

    /*
     * TODO
     * Implement various other ways to distribute food onto the arena floor, e.g. power law, etc.
     *
     */
    // set the positions for all food items to be drawn later
    for(UInt32 i = 0; i < foodItemCount; ++i) {
    	iAntData.food.foodPositions.push_back(CVector2(RNG->Uniform(iAntData.navigation.forageRangeX),
    			                                       RNG->Uniform(iAntData.navigation.forageRangeY)));
    }

    // get the footbot entities
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    // and set the footbot's nest location and search target values
    for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& controller = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        controller.iAntData.Init(node);
        controller.iAntData.food.foodPositions = iAntData.food.foodPositions;

        // implement this later -> controller.iAntData = iAntData;
    }
}

CColor iAnt_loop_functions::GetFloorColor(const CVector2& position) {
    // set the pheromone marker
	/*
	if(targetSeeking) {
        if((position - searchTarget).SquareLength() < foodRadiusSquared) {
            return CColor::GRAY40;
        }
    }
    */

    // food items are black discs with radius set in XML
    // check the positions of all food items
    for(UInt32 i = 0; i < iAntData.food.foodPositions.size(); ++i) {
        // if we are in the bounds of a food item, paint it black
		if((position - iAntData.food.foodPositions[i]).SquareLength() < iAntData.food.foodRadiusSquared) {
			return CColor::BLACK;
		}
	}

	// nest area is grey disc with radius set in XML
    // if we are in the bounds of the nest, paint it grey
    if((position - iAntData.navigation.nestPosition).SquareLength() < iAntData.navigation.nestRadiusSquared) {
        return CColor::GRAY80;
    }

    // default arena color when otherwise not the nest, food item, or pheromone
	return CColor::WHITE;
}

// this function is called BEFORE the ControlStep() function in the controller class
void iAnt_loop_functions::PreStep() {
	++tick; // increment the frame variable

	// container for all available foot-bot controller objects
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    // loop through all of the foot-bot controller objects
	for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
	    // variable for the current foot-bot
	    CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
	    // variable for the current foot-bot's controller object
		iAnt_controller& controller = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

		// set the current foot-bot's position (used in controller class for positioning)
		controller.iAntData.navigation.position.Set(footBot.GetEmbodiedEntity().GetPosition().GetX(),
				                                    footBot.GetEmbodiedEntity().GetPosition().GetY());

		if(controller.hasFoundFood() && !controller.iAntData.food.isHoldingFoodItem) {
			bool done = false;
			vector<CVector2>::iterator i;

			// check each food item position to see if the foot-bot found food
            for(i = iAntData.food.foodPositions.begin(); i != iAntData.food.foodPositions.end() && !done; i++) {
                // if the foot-bot is within range of a food item
                if((controller.iAntData.navigation.position - *i).SquareLength() < iAntData.food.foodRadiusSquared) {
        	    	controller.iAntData.food.isHoldingFoodItem = true;

                    // pick up the food item and update foodData variables
                    floorEntity->SetChanged();
                    done = true;
                }
            }

            // erase the foodPosition from the vector, it's no longer needed
            if(done) {
            	iAntData.food.foodPositions.erase(--i); // remember that the for loop applies i++ ONCE too far! go back!!!
            }

            controller.iAntData.food.foodPositions = iAntData.food.foodPositions;
	    } else if(controller.isInTheNest() && controller.iAntData.food.isHoldingFoodItem) {
	    	controller.iAntData.food.isHoldingFoodItem = false;
	    }
	}
}

void iAnt_loop_functions::PostStep() {
    // nothing... yet.
}

void iAnt_loop_functions::Reset() {
	for(UInt32 i = 0; i < iAntData.food.foodPositions.size(); ++i) {
		iAntData.food.foodPositions[i].Set(RNG->Uniform(iAntData.navigation.forageRangeX), RNG->Uniform(iAntData.navigation.forageRangeY));
	}

	tick = 0;
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

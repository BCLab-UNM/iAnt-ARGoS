#include "iAnt_loop_functions.h"

// (constructor) initialize class variables, Init() function contains further setup
iAnt_loop_functions::iAnt_loop_functions() :
	floorEntity(NULL),
	RNG(NULL)//,
	//foodItemReset(false)//,
	//foodRadiusSquared(0.0)
{}

void iAnt_loop_functions::Init(TConfigurationNode& node) {
	iAntData.Init(node);

	// floor object where food and nest objects are drawn
    floorEntity = &GetSpace().GetFloorEntity();

    // use to initialize class objects and variables
    UInt32 foodItemCount;

    // set XML parameters to variables
	GetNodeAttribute(GetNode(node, "simulation_settings"), "foodItemCount", foodItemCount);
    GetNodeAttribute(GetNode(node, "simulation_settings"), "foodRadius", simulation_settings.foodRadiusSquared);
    GetNodeAttribute(GetNode(node, "simulation_settings"), "foodItemReset", simulation_settings.foodItemReset);

    // do this here so the user doesn't have to pre-calculate in the XML file
    simulation_settings.foodRadiusSquared *= simulation_settings.foodRadiusSquared;

    // create a random number generator for random food item placement
    RNG = CRandom::CreateRNG("argos");

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
        // implement this later -> controller.iAntData = iAntData;
        controller.iAntData.navigation.nestPosition = iAntData.navigation.nestPosition;
        controller.iAntData.navigation.arenaSize = iAntData.navigation.arenaSize;
        controller.iAntData.navigation.forageRangeX = iAntData.navigation.forageRangeX;
        controller.iAntData.navigation.forageRangeY = iAntData.navigation.forageRangeY;
        controller.iAntData.food.foodPositions = iAntData.food.foodPositions;
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
		if((position - iAntData.food.foodPositions[i]).SquareLength() < simulation_settings.foodRadiusSquared) {
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

		/*
		LOG << endl << controller.GetId() << endl
			<< controller.iAntData.navigation.arenaSize << endl
			<< controller.iAntData.navigation.angleTolerance << endl
			<< controller.iAntData.navigation.distanceTolerance << endl
			<< controller.iAntData.CPFA.state << endl
			<< controller.iAntData.CPFA.siteFidelityRate << endl;
			*/

		if(controller.hasFoundFood() && !controller.iAntData.food.isHoldingFoodItem) {
			bool done = false;
			vector<CVector2>::iterator i;

			// check each food item position to see if the foot-bot found food
            for(i = iAntData.food.foodPositions.begin(); i != iAntData.food.foodPositions.end() && !done; i++) {
                // if the foot-bot is within range of a food item
                if((controller.iAntData.navigation.position - *i).SquareLength() < simulation_settings.foodRadiusSquared) {
        	    	controller.iAntData.food.isHoldingFoodItem = true;
                    if(simulation_settings.foodItemReset) {
                        // move to a new place!
                        i->Set(RNG->Uniform(iAntData.navigation.forageRangeX), RNG->Uniform(iAntData.navigation.forageRangeY));
                    }

                    // pick up the food item and update foodData variables
                    floorEntity->SetChanged();
                    done = true;
                }
            }

            // erase the foodPosition from the vector if it's no longer needed
            if(done && !simulation_settings.foodItemReset) {
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
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

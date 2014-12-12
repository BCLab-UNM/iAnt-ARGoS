#include "iAnt_loop_functions.h"

// (constructor) initialize class variables, Init() function contains further setup
iAnt_loop_functions::iAnt_loop_functions():
	floorEntity(NULL),
	RNG(NULL),
	tick(0),
	foodItemCount(0),
	foodRadiusSquared(0.0),
	nestRadiusSquared(0.0),
	foodDistribution(0),
	updateFoodOnly(false)
{}

void iAnt_loop_functions::Init(TConfigurationNode& node) {
	// initialize floor object where food and nest objects are drawn
    floorEntity = &GetSpace().GetFloorEntity();

    // used to build forageRangeX and forageRangeY
    CVector2 rangeX, rangeY;

    Real foodRadius;

    // set XML parameters to variables
	GetNodeAttribute(GetNode(node, "simulation_settings"), "foodItemCount"   , foodItemCount    );
	GetNodeAttribute(GetNode(node, "simulation_settings"), "foodDistribution", foodDistribution );
	GetNodeAttribute(GetNode(node, "navigation")         , "forageRangeX"    , rangeX           );
	GetNodeAttribute(GetNode(node, "navigation")         , "forageRangeY"    , rangeY           );
	GetNodeAttribute(GetNode(node, "navigation")         , "nestPosition"    , nestPosition     );
	GetNodeAttribute(GetNode(node, "navigation")         , "nestRadius"      , nestRadiusSquared);
	GetNodeAttribute(GetNode(node, "food")               , "foodRadius"      , foodRadius);

	nestRadiusSquared *= nestRadiusSquared;
	foodRadiusSquared = foodRadius * foodRadius;
    forageRangeX.Set(rangeX.GetX(), rangeX.GetY());
    forageRangeY.Set(rangeY.GetX(), rangeY.GetY());

    // create a random number generator for random food item placement
    RNG = CRandom::CreateRNG("argos");

    // food distribution can = "random", "power law", "cluster"
    // random distribution
    switch(foodDistribution) {
    	case 1: { // cluster
    		int clusterSize = (foodItemCount / 4);
    		int clusterWidth = (int)sqrt(round((double)clusterSize));
    		CVector2 placementPosition;

    		for(int i = 0; i < clusterSize; i++) {
    			placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));

    			for(int j = 0; j < clusterWidth; j++) {
    				for(int k = 0; k < clusterWidth; k++) {
    	    			foodPositions.push_back(placementPosition);
    	    			placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0));
    				}

    				placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0 * clusterWidth));
    				placementPosition.SetY(placementPosition.GetY() + (foodRadius * 3.0));
    			}
    		}

    		break;
    	}
    	case 2: { // power law
    		int clusterNumber = 4;
    		int clusterSize = foodItemCount;
    		int clusterWidth = 0;
    		CVector2 placementPosition;

    		for(int i = 0; i < clusterNumber; i++) {
    			placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
    			clusterSize = (int)round((double)clusterSize / 2.0);
        		clusterWidth = (int)sqrt(round((double)clusterSize));

    			for(int j = 0; j < clusterWidth; j++) {
    				for(int k = 0; k < clusterWidth; k++) {
    	    			foodPositions.push_back(placementPosition);
    	    			placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0));
    				}

    				placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0 * clusterWidth));
    				placementPosition.SetY(placementPosition.GetY() + (foodRadius * 3.0));
    			}
    		}

    		break;
    	}
    	case 0: // random
    		for(UInt32 i = 0; i < foodItemCount; ++i) {
    			foodPositions.push_back(CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY)));
    		}
    		break;
    	default:
    		LOGERR << "Invalid food distribution in XML! Using default random distribution\n";
    		for(UInt32 i = 0; i < foodItemCount; ++i) {
    			foodPositions.push_back(CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY)));
    		}
    }

    // get the footbot entities
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    // and set the footbot's nest location and search target values
    for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        c.UpdateFoodList(foodPositions);
    }
}

CColor iAnt_loop_functions::GetFloorColor(const CVector2& position) {
	// TODO enhance the efficiency of this code!
    // food items are black discs with radius set in XML
    // check the positions of all food items
	for(UInt32 i = 0; i < foodPositions.size(); i++) {
		// if we are in the bounds of a food item, paint it black
		if((position - foodPositions[i]).SquareLength() < foodRadiusSquared) {
			return CColor::BLACK;
		}
	}

	//if(updateFoodOnly == false) {
		for(UInt32 i = 0; i < pheromoneList.size() && foodPositions.size() > 0; i++) {
			// if we are in the bounds of a food item, paint it black
			if(pheromoneList[i].IsActive() && (position - pheromoneList[i].Location()).SquareLength() < foodRadiusSquared) {
				return CColor::RED;
			}
		}
	//}

	// nest area is grey disc with radius set in XML
    // if we are in the bounds of the nest, paint it grey
    if((position - nestPosition).SquareLength() < nestRadiusSquared) {
        return CColor::GRAY80;
    }

    // default arena color when otherwise not the nest, food item, or pheromone
	return CColor::WHITE;
}

// this function is called BEFORE the ControlStep() function in the controller class
void iAnt_loop_functions::PreStep() {
	bool done = false;
	bool pheromoneSet = false;
	// vector<CVector2> tempMarkerPositions;

	// container for all available foot-bot controller objects
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    tick++; // increment the frame variable

    // loop through all of the foot-bot controller objects
	for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
	    // variable for the current foot-bot
	    CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
	    // variable for the current foot-bot's controller object
		iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());
		// set the current foot-bot's position (used in controller class for positioning)
		c.UpdatePosition(CVector2(footBot.GetEmbodiedEntity().GetPosition().GetX(),
				                  footBot.GetEmbodiedEntity().GetPosition().GetY()));
		c.UpdateTime(tick);

		// if the robot has found food and isn't already holding food
		if(c.IsFindingFood() && !c.IsHoldingFood()) {
			/*****/ // bool done = false;
			vector<CVector2>::iterator i;

			// check each food item position to see if the foot-bot found food
            for(i = foodPositions.begin(); i != foodPositions.end() && !done; i++) {
                // if the foot-bot is within range of a food item
                if((c.Position() - *i).SquareLength() < foodRadiusSquared) {
                    // pick up the food item and update foodData variables
        	    	c.PickupFood();
        	    	if(c.GetTargetPheromone().IsActive() == true) pheromoneList.push_back(c.GetTargetPheromone());
                    done = true;
                }
            }

            // erase the foodPosition from the vector, it's no longer needed
            if(done) {
            	foodPositions.erase(--i); // remember that the for loop applies i++ ONCE too far! go back!!!
            	updateFoodOnly = true;
            	/*****/ //floorEntity->SetChanged();
            }

            c.UpdateFoodList(foodPositions);
	    } else if(c.IsInTheNest() && c.IsHoldingFood()) {
	    	/*****/ // bool pheromoneSet = false;
	    	c.DropOffFood();

	    	/* needs to be weighted random selection from pheromone */
	    	double maxStrength = 0.0;

	    	for(unsigned int i = 0; i < pheromoneList.size(); i++) {
	    		pheromoneList[i].Update(tick);

	    		if(pheromoneList[i].IsActive() == true) maxStrength += pheromoneList[i].Weight();
	    	}

	    	double randomWeight = RNG->Uniform(CRange<double>(0.0, maxStrength));

			vector<iAnt_pheromone>::iterator i;

	    	for(i = pheromoneList.begin(); i != pheromoneList.end() && !pheromoneSet; i++) {
	    		if(randomWeight < i->Weight() && i->IsActive() == true) {
	    			c.SetTargetPheromone(*i);
	    			pheromoneSet = true;
	    		}

	    		randomWeight -= i->Weight();
	    	}

	    	updateFoodOnly = false;
	    	// if(pheromoneSet) floorEntity->SetChanged();
	    }
	}

	if(done || pheromoneSet) floorEntity->SetChanged();
}

void iAnt_loop_functions::PostStep() {
    // nothing... yet
}

void iAnt_loop_functions::Reset() {
	foodPositions.clear();

	for(UInt32 i = 0; i < foodItemCount; ++i) {
    	foodPositions.push_back(CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY)));
	}

	floorEntity->SetChanged();
	tick = 0;
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

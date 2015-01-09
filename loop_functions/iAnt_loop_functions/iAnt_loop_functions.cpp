#include "iAnt_loop_functions.h"

// (constructor) initialize class variables, Init() function contains further setup
iAnt_loop_functions::iAnt_loop_functions():
	floorEntity(NULL),
	RNG(NULL),
	simTime(0),
	foodItemCount(0),
	foodRadiusSquared(0.0),
	nestRadiusSquared(0.0),
	foodDistribution(0)
{}

iAnt_loop_functions::~iAnt_loop_functions() {}

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

    SetFoodDistribution();

    // food distribution can = "random", "power law", "cluster"
    /* TODO: move this into its own function */
/*
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
            // old method
    		//int clusterNumber = 4;
    		//int clusterSize = foodItemCount;
    		//int clusterWidth = 0;
    		//CVector2 placementPosition;

    		//for(int i = 0; i < clusterNumber; i++) {
    			//placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
    			//clusterSize = (int)round((double)clusterSize / 2.0);
        		//clusterWidth = (int)sqrt(round((double)clusterSize));

    			//for(int j = 0; j < clusterWidth; j++) {
    				//for(int k = 0; k < clusterWidth; k++) {
    	    			//foodPositions.push_back(placementPosition);
    	    			//placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0));
    				//}

    				//placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0 * clusterWidth));
    				//placementPosition.SetY(placementPosition.GetY() + (foodRadius * 3.0));
    			//}
    		//}

    		CVector2 placementPosition(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
    		CVector2 startPosition = placementPosition;
            CRange<Real> cluster64x, cluster64y;
            CRange<Real> cluster16x, cluster16y;
            CRange<Real> cluster4x, cluster4y;
            int foodCount = foodItemCount;

            if(foodCount >= 64) {
                for(int i = 0; i < 8; i++) {
                    for(int j = 0; j < 8; j++) {
    	    			foodPositions.push_back(placementPosition);
    	    			placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0));
    	    			//placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0));
    				}

    				placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0 * 8.0));
    				placementPosition.SetY(placementPosition.GetY() - (foodRadius * 3.0));
    				//placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0 * 8.0));
    				//placementPosition.SetY(placementPosition.GetY() + (foodRadius * 3.0));
                }

                cluster64x.Set(startPosition.GetX() - foodRadius - (foodRadius * 3.0 * 4.0),
                               startPosition.GetX() + (foodRadius * 3.0 * 8.0));
                cluster64y.Set(startPosition.GetY() - foodRadius - (foodRadius * 3.0 * 4.0),
                               startPosition.GetY() + (foodRadius * 3.0 * 8.0));
                foodCount -= 64;
            }

            placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));

            while(cluster64x.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetX()) &&
                  cluster64y.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetY())) {
                placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
            }
            startPosition = placementPosition;

            if(foodCount >= 16) {
                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
    	    			foodPositions.push_back(placementPosition);
    	    			placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0));
    	    			//placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0));
    				}

    				placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0 * 4.0));
    				placementPosition.SetY(placementPosition.GetY() - (foodRadius * 3.0));
    				//placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0 * 4.0));
    				//placementPosition.SetY(placementPosition.GetY() + (foodRadius * 3.0));
                }

                cluster16x.Set(startPosition.GetX() - foodRadius - (foodRadius * 3.0 * 2.0),
                               startPosition.GetX() + (foodRadius * 3.0 * 4.0));
                cluster16y.Set(startPosition.GetY() - foodRadius - (foodRadius * 3.0 * 2.0),
                               startPosition.GetY() + (foodRadius * 3.0 * 4.0));
                foodCount -= 16;
            }

            placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));

            while((cluster64x.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetX()) &&
                  cluster64y.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetY())) ||
                  (cluster16x.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetX()) &&
                  cluster16y.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetY()))) {
                placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
            }
            startPosition = placementPosition;

            if(foodCount >= 4) {
                for(int i = 0; i < 2; i++) {
                    for(int j = 0; j < 2; j++) {
    	    			foodPositions.push_back(placementPosition);
    	    			placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0));
    	    			//placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0));
    				}

    				placementPosition.SetX(placementPosition.GetX() + (foodRadius * 3.0 * 2.0));
    				placementPosition.SetY(placementPosition.GetY() - (foodRadius * 3.0));
    				//placementPosition.SetX(placementPosition.GetX() - (foodRadius * 3.0 * 2.0));
    				//placementPosition.SetY(placementPosition.GetY() + (foodRadius * 3.0));
                }

                cluster4x.Set(startPosition.GetX() - foodRadius - (foodRadius * 3.0),
                              startPosition.GetX() + (foodRadius * 3.0 * 2.0));
                cluster4y.Set(startPosition.GetY() - foodRadius - (foodRadius * 3.0),
                              startPosition.GetY() + (foodRadius * 3.0 * 2.0));
                foodCount -= 4;
            }

            placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));

            while(foodCount > 0) {
                while((cluster64x.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetX()) &&
                   cluster64y.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetY())) ||
                   (cluster16x.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetX()) &&
                   cluster16y.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetY())) ||
                   (cluster4x.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetX()) &&
                   cluster4y.WithinMinBoundIncludedMaxBoundIncluded(placementPosition.GetY()))) {
                    placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
                }

    			foodPositions.push_back(placementPosition);
                foodCount--;
                placementPosition = CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
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
*/

    // get the footbot entities
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    // and set the footbot's nest location and food item positions
    for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());
        c.UpdateFoodList(foodPositions);
        c.SetNestPosition(nestPosition);
        c.SetNestRadiusSquared(nestRadiusSquared);
        c.SetForageRange(forageRangeX, forageRangeY);
        c.SetFoodRadiusSquared(foodRadiusSquared);
    }
}

CColor iAnt_loop_functions::GetFloorColor(const CVector2& position) {
    return CColor::WHITE;
}

// this function is called BEFORE the ControlStep() function in the controller class
void iAnt_loop_functions::PreStep() {
    bool setChanged = false;

	// container for all available foot-bot controller objects
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    simTime++; // increment the frame variable

	for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
	    // variable for the current foot-bot
	    CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);

	    // variable for the current foot-bot's controller object
		iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

		// set the current foot-bot's position (used in controller class for positioning)
		c.UpdatePosition(CVector2(footBot.GetEmbodiedEntity().GetPosition().GetX(),
				                  footBot.GetEmbodiedEntity().GetPosition().GetY()));
		c.UpdateTime(simTime);
        c.UpdateFoodList(foodPositions);
        c.UpdateFidelityList(fidelityPositions);

		// if the robot has found food and isn't already holding food
		if(c.IsFindingFood() && !c.IsHoldingFood()) {
			vector<CVector2>::iterator i;

			// check each food item position to see if the foot-bot found food
            for(i = foodPositions.begin(); i != foodPositions.end(); i++) {
                // if the foot-bot is within range of a food item
                if((c.Position() - *i).SquareLength() < foodRadiusSquared) {
                    // pick up the food item and update foodData variables
        	    	c.PickupFood();
        	    	if(c.GetTargetPheromone().IsActive() == true) {

                        // clean pheromone placement
                        //pheromoneList.push_back(iAnt_pheromone((*i),
                        //                        c.GetTargetPheromone().LastUpdated(),
                        //                        c.GetTargetPheromone().DecayRate(),
                        //                        c.GetTargetPheromone().Weight()));

                        // messy pheromone placement
                        pheromoneList.push_back(c.GetTargetPheromone());

                        fidelityPositions.push_back(c.GetTargetPheromone().Location());
                    }
                    foodPositions.erase(i);
                    setChanged = true;
                    break;
                }
            }
	    } else if(c.IsInTheNest() && c.IsHoldingFood()) {
	    	c.DropOffFood();

	    	/* needs to be weighted random selection from pheromone */
	    	double maxStrength = 0.0;

	    	for(unsigned int i = 0; i < pheromoneList.size(); i++) {
                pheromoneList[i].Update(simTime);
                if(pheromoneList[i].IsActive() == true) {
                    maxStrength += pheromoneList[i].Weight();
                }
	    	}

	    	double randomWeight = RNG->Uniform(CRange<double>(0.0, maxStrength));

			vector<iAnt_pheromone>::iterator i;

            for(i = pheromoneList.begin(); i != pheromoneList.end(); i++) {
	    		if(randomWeight < i->Weight() && i->IsActive() == true) {
	    			c.SetTargetPheromone(*i);
                    setChanged = true;
	    			break;
	    		}

	    		randomWeight -= i->Weight();
	    	}
        }

        for(int i = 0; i < pheromoneList.size(); i++) {
            if(pheromoneList[i].IsActive() == true) {
                pheromonePositions.push_back(pheromoneList[i].Location());
            }
        }

        c.UpdatePheromoneList(pheromonePositions);
    }

    fidelityPositions.clear();
    pheromonePositions.clear();

    if(setChanged == true) floorEntity->SetChanged();
}

void iAnt_loop_functions::PostStep() {
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

	for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); ++it) {
	    // variable for the current foot-bot
	    CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);

	    // variable for the current foot-bot's controller object
		iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        c.UpdateFoodList(foodPositions);
    }

    if(foodPositions.size() == 0) {
        pheromoneList.clear();
    }
}

void iAnt_loop_functions::Reset() {
	foodPositions.clear();

	for(UInt32 i = 0; i < foodItemCount; ++i) {
    	foodPositions.push_back(CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY)));
	}

	floorEntity->SetChanged();
	simTime = 0;
}

/*
 * foodDistribution is set in the XML file
 * random    = 0
 * cluster   = 1
 * power law = 2
 *
 */
void iAnt_loop_functions::SetFoodDistribution() {
    switch(foodDistribution) {
        case 0:
            RandomFoodDistribution();
            break;
        case 1:
            ClusterFoodDistribution();
            break;
        case 2:
            PowerLawFoodDistribution();
            break;
        default:
            LOGERR << "ERROR: Invalid food distribution in XML file.\n";
    }
}

void iAnt_loop_functions::RandomFoodDistribution() {
    for(UInt32 i = 0; i < foodItemCount; ++i) {
        foodPositions.push_back(CVector2(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY)));
    }
}

void iAnt_loop_functions::ClusterFoodDistribution() {
    double foodRadius = sqrt(foodRadiusSquared);
    int clusterSize = 4; (foodItemCount / 4);
    int clusterWidth = (int)sqrt(round((double)(foodItemCount/clusterSize)));
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
}

void iAnt_loop_functions::PowerLawFoodDistribution() {
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

#include "iAnt_loop_functions.h"

// (constructor) initialize class variables, Init() function contains further setup
iAnt_loop_functions::iAnt_loop_functions():
	floorEntity(NULL),
	RNG(NULL),
	simTime(0),
    simCounter(0),
	foodItemCount(0),
    ticks_per_second(0),
    ticks_per_simulation(0),
    random_seed(0),
	foodRadiusSquared(0.0),
	nestRadiusSquared(0.0),
    variableSeed(false),
	foodDistribution(0),
    numberOfClusters(0),
    clusterWidthX(0),
    clusterLengthY(0),
    powerRank(0)
{}

iAnt_loop_functions::~iAnt_loop_functions() {}

void iAnt_loop_functions::Init(TConfigurationNode& node) {
    initNode = &node;

	// initialize floor object where food and nest objects are drawn
    floorEntity = &GetSpace().GetFloorEntity();
    arenaSize = GetSpace().GetArenaSize();

    // used to build forageRangeX and forageRangeY
    CVector2 rangeX(-arenaSize.GetX()/2.0, arenaSize.GetX()/2.0),
             rangeY(-arenaSize.GetY()/2.0, arenaSize.GetY()/2.0);

    // use these to get simulation data from other parts of the XML file
    // outside of the loop_functions scope
    CSimulator      *sim     = &GetSimulator();
    CPhysicsEngine  *pEngine = &sim->GetPhysicsEngine("default");

    Real   foodRadius;
    size_t variableSeedInt = 0;

    /*
     * simulator.h, space.h, and loop_functions.h, (among others) have useful
     * functions for getting at data from other parts of the XML file outside
     * of the node's branch
     */
    ticks_per_simulation = sim->GetMaxSimulationClock();
    random_seed          = sim->GetRandomSeed();
    ticks_per_second     = pEngine->GetInverseSimulationClockTick();

    TConfigurationNode simSettings = GetNode(node, "simulation_settings"),
                       cluster     = GetNode(node, "cluster_distribution_1"),
                       powerLaw    = GetNode(node, "powerLaw_distribution_2");

    // set XML parameters to variables
    GetNodeAttribute(simSettings, "variableSeed"    , variableSeedInt  );
    GetNodeAttribute(simSettings, "foodItemCount"   , foodItemCount    );
    GetNodeAttribute(simSettings, "foodDistribution", foodDistribution );
    GetNodeAttribute(simSettings, "nestPosition"    , nestPosition     );
    GetNodeAttribute(simSettings, "nestRadius"      , nestRadiusSquared);
    GetNodeAttribute(simSettings, "foodRadius"      , foodRadius       );
    GetNodeAttribute(cluster    , "numberOfClusters", numberOfClusters );
    GetNodeAttribute(cluster    , "clusterWidthX"   , clusterWidthX    );
    GetNodeAttribute(cluster    , "clusterLengthY"  , clusterLengthY   );
    GetNodeAttribute(powerLaw   , "powerRank"       , powerRank        );

    variableSeed = (variableSeedInt == 1) ? (true) : (false);
	nestRadiusSquared *= nestRadiusSquared;
	foodRadiusSquared = foodRadius * foodRadius;
    forageRangeX.Set(rangeX.GetX(), rangeX.GetY());
    forageRangeY.Set(rangeY.GetX(), rangeY.GetY());
    RNG = CRandom::CreateRNG("argos");

    SetFoodDistribution();

    // get the footbot entities
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

    // and set the footbot's nest location and food item positions
    for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        c.SetFoodPositions(foodPositions);
        c.SetNestPosition(nestPosition);
        c.SetNestRadiusSquared(nestRadiusSquared);
        c.SetForageRange(forageRangeX, forageRangeY);
        c.SetFoodRadiusSquared(foodRadiusSquared);
        c.SetLoopFunctions(this);
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

	for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
	    // variable for the current foot-bot
	    CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);

	    // variable for the current foot-bot's controller object
		iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

		c.SetTime(simTime);
        c.SetFoodPositions(foodPositions);
        //c.SetFidelityPositions(fidelityPositions);

		// if the robot has found food and isn't already holding food
		if(c.IsFindingFood() && !c.IsHoldingFood()) {
			vector<CVector2>::iterator i;

			// check each food item position to see if the foot-bot found food
            for(i = foodPositions.begin(); i != foodPositions.end(); i++) {
                // if the foot-bot is within range of a food item
                if((c.GetPosition() - *i).SquareLength() < foodRadiusSquared) {
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

	    	for(size_t i = 0; i < pheromoneList.size(); i++) {
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

        for(size_t i = 0; i < pheromoneList.size(); i++) {
            if(pheromoneList[i].IsActive() == true) {
                pheromonePositions.push_back(pheromoneList[i].Location());
            }
        }

        c.SetPheromonePositions(pheromonePositions);
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

        c.SetFoodPositions(foodPositions);
    }

    if(foodPositions.size() == 0) {
        pheromoneList.clear();
    }
}


/**
 * Returns <tt>true</tt> if the experiment is finished.
 * This method allows the user to specify experiment-specific ending conditions.
 * The default implementation of this method returns always <tt>false</tt>.
 * This means that the only ending conditions for an experiment are time limit
 * expiration or GUI shutdown.
 * @return <tt>true</tt> if the experiment is finished.
 * @see CSimulator::IsExperimentFinished()
 */
bool iAnt_loop_functions::IsExperimentFinished() {
    if(foodPositions.size() == 0) {
        return true;
    }

    return false;
}

/**
 * Executes user-defined logic when the experiment finishes.
 * This method is called within CSimulator::IsExperimentFinished()
 * as soon as its return value evaluates to <tt>true</tt>. This
 * method is executed before Destroy().
 * You can use this method to perform final calculations at the
 * end of an experiment.
 * The default implementation of this method does nothing.
 */
void iAnt_loop_functions::PostExperiment() {
    ofstream dataOutput("iAntTagData.txt", ios::app);
    size_t time_in_minutes = floor(floor(simTime / ticks_per_second) / 60);

    if(dataOutput.tellp() == 0) {
        dataOutput << "tags_collected, time_in_minutes, random_seed\n";
    }

    dataOutput << foodItemCount - foodPositions.size() << ", ";
    dataOutput << time_in_minutes << ", " << random_seed << endl;
    dataOutput.close();

    if(simCounter == 0) {
        LOG << "\ntags_collected, time_in_minutes, random_seed\n";
        LOG << foodItemCount - foodPositions.size() << ", ";
        LOG << time_in_minutes << ", " << random_seed << endl;
    } else {
        ifstream dataInput("iAntTagData.txt");
        string s;

        while(getline(dataInput, s)) {
            LOG << s << endl;
        }

        dataInput.close();
    }

    simCounter++;
    foodPositions.clear();
    pheromoneList.clear();
    if(variableSeed) GetSimulator().SetRandomSeed(++random_seed);
}


void iAnt_loop_functions::Reset() {
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    simTime = 0;
    SetFoodDistribution();

    for(CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        c.SetFoodPositions(foodPositions);
        c.SetPheromonePositions(pheromonePositions);
        //c.SetFidelityPositions(fidelityPositions);
        c.SetForageRange(forageRangeX, forageRangeY);
        c.Reset();
    }

    floorEntity->SetChanged();
}

vector<CVector2> iAnt_loop_functions::GetFidelityPositions() {
    return fidelityPositions;
}

vector<CVector2> iAnt_loop_functions::GetFoodPositions() {
    return foodPositions;
}

vector<CVector2> iAnt_loop_functions::GetPheromonePositions() {
    return pheromonePositions;
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
    CVector2 placementPosition;

    for(size_t i = 0; i < foodItemCount; i++) {
        placementPosition.Set(RNG->Uniform(forageRangeX),
                              RNG->Uniform(forageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(forageRangeX),
                                  RNG->Uniform(forageRangeY));
        }

        foodPositions.push_back(placementPosition);
    }
}

void iAnt_loop_functions::RandomFoodDistribution(size_t f) {
    CVector2 placementPosition;

    for(size_t i = 0; i < f; i++) {
        placementPosition.Set(RNG->Uniform(forageRangeX),
                              RNG->Uniform(forageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(forageRangeX),
                                  RNG->Uniform(forageRangeY));
        }

        foodPositions.push_back(placementPosition);
    }
}

void iAnt_loop_functions::ClusterFoodDistribution() {
    Real   foodOffset  = 3.0 * GetFoodRadius();
    size_t foodToPlace = numberOfClusters * clusterWidthX * clusterLengthY;

    CVector2 placementPosition;

    if(foodToPlace < foodItemCount) {
        foodToPlace = foodItemCount;
    } else {
        foodItemCount = foodToPlace;
    }

    for(size_t i = 0; i < numberOfClusters; i++) {
        if(foodToPlace == 0) break;

        placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));

        while(IsOutOfBounds(placementPosition, clusterLengthY, clusterWidthX)) {
            placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
        }

        for(size_t j = 0; j < clusterLengthY; j++) {
            for(size_t k = 0; k < clusterWidthX; k++) {
                foodToPlace--;                
                foodPositions.push_back(placementPosition);
                placementPosition.SetX(placementPosition.GetX() + foodOffset);
            }

            placementPosition.SetX(placementPosition.GetX() - (clusterWidthX * foodOffset));
            placementPosition.SetY(placementPosition.GetY() + foodOffset);
        }
    }

    if(foodItemCount > foodToPlace) RandomFoodDistribution(foodItemCount - foodToPlace);
    if(foodToPlace > foodItemCount) foodItemCount = foodToPlace;
}

void iAnt_loop_functions::PowerLawFoodDistribution() {
    Real   foodOffset     = 3.0 * GetFoodRadius();
    size_t foodPlaced     = 0;
    size_t powerLawLength = 1;

    vector<size_t> powerLawClusters;
    vector<size_t> clusterSides;
    CVector2       placementPosition;

    for(size_t i = 0; i < powerRank; i++) {
        powerLawClusters.push_back(powerLawLength * powerLawLength);
        powerLawLength *= 2;
    }

    for(size_t i = 0; i < powerRank; i++) {
        powerLawLength /= 2;
        clusterSides.push_back(powerLawLength);
    }

    for(size_t h = 0; h < powerLawClusters.size(); h++) {
        for(size_t i = 0; i < powerLawClusters[h]; i++) {
            placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));

            while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
                placementPosition.Set(RNG->Uniform(forageRangeX), RNG->Uniform(forageRangeY));
            }

            for(size_t j = 0; j < clusterSides[h]; j++) {
                for(size_t k = 0; k < clusterSides[h]; k++) {
                    foodPlaced++;
                    foodPositions.push_back(placementPosition);
                    placementPosition.SetX(placementPosition.GetX() + foodOffset);
                }

                placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
                placementPosition.SetY(placementPosition.GetY() + foodOffset);
            }
        }
    }

    if(foodPlaced < foodItemCount) RandomFoodDistribution(foodItemCount - foodPlaced);
    if(foodItemCount < foodPlaced) foodItemCount = foodPlaced;
}

bool iAnt_loop_functions::IsCollidingWithNest(CVector2 p) {
    Real nestRadiusPlusBuffer = GetNestRadius() + GetFoodRadius();
    Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    return ((p - nestPosition).SquareLength() < NRPB_squared);
}

bool iAnt_loop_functions::IsCollidingWithFood(CVector2 p) {
    Real foodRadiusPlusBuffer = 2.0 * GetFoodRadius();
    Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

    for(size_t i = 0; i < foodPositions.size(); i++) {
        if((p - foodPositions[i]).SquareLength() < FRPB_squared) return true;
    }

    return false;
}

bool iAnt_loop_functions::IsOutOfBounds(CVector2 p, size_t length, size_t width) {
    CVector2 placementPosition = p;

    Real foodOffset   = 3.0 * GetFoodRadius();
    Real widthOffset  = 3.0 * GetFoodRadius() * (Real)width;
    Real lengthOffset = 3.0 * GetFoodRadius() * (Real)length;

    Real x_min = p.GetX() - GetFoodRadius();
    Real x_max = p.GetX() + GetFoodRadius() + widthOffset;

    Real y_min = p.GetY() - GetFoodRadius();
    Real y_max = p.GetY() + GetFoodRadius() + lengthOffset;

    if((x_min < (forageRangeX.GetMin() + GetFoodRadius())) ||
       (x_max > (forageRangeX.GetMax() - GetFoodRadius())) ||
       (y_min < (forageRangeY.GetMin() + GetFoodRadius())) ||
       (y_max > (forageRangeY.GetMax() - GetFoodRadius()))) {
        return true;
    }

    for(size_t j = 0; j < length; j++) {
        for(size_t k = 0; k < width; k++) {
            if(IsCollidingWithFood(placementPosition)) return true;
            if(IsCollidingWithNest(placementPosition)) return true;
            placementPosition.SetX(placementPosition.GetX() + foodOffset);
        }

        placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
        placementPosition.SetY(placementPosition.GetY() + foodOffset);
    }

    return false;
}

Real iAnt_loop_functions::GetNestRadius() {
    return sqrt(nestRadiusSquared);
}

Real iAnt_loop_functions::GetFoodRadius() {
    return sqrt(foodRadiusSquared);
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

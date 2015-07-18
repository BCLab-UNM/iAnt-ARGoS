#include "iAnt_loop_functions.h"

/******************************************************************************/
/* Constructor */
/******************************************************************************/
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
    outputData(false),
	foodDistribution(0),
    numberOfClusters(0),
    clusterWidthX(0),
    clusterLengthY(0),
    powerRank(0)
{}

/******************************************************************************/
/* Destructor */
/******************************************************************************/
iAnt_loop_functions::~iAnt_loop_functions() {}

/******************************************************************************/
/* Initialization Function */
/******************************************************************************/
void iAnt_loop_functions::Init(TConfigurationNode& node) {
    initNode    = &node;
    floorEntity = &GetSpace().GetFloorEntity();
    arenaSize   =  GetSpace().GetArenaSize();

    CVector2 rangeX =  CVector2(-arenaSize.GetX()/2.0, arenaSize.GetX()/2.0);
    CVector2 rangeY =  CVector2(-arenaSize.GetY()/2.0, arenaSize.GetY()/2.0);

    CSimulator           *sim             = &GetSimulator();
    CPhysicsEngine       *pEngine         = &sim->GetPhysicsEngine("default");
    Real                  foodRadius      =  0.0;
    size_t                variableSeedInt =  0;
    size_t                outputDataInt   =  0;

    ticks_per_simulation = sim->GetMaxSimulationClock();
    random_seed          = sim->GetRandomSeed();
    ticks_per_second     = pEngine->GetInverseSimulationClockTick();

    TConfigurationNode simSettings = GetNode(node, "simulation_settings");
    TConfigurationNode cluster     = GetNode(node, "cluster_distribution_1");
    TConfigurationNode powerLaw    = GetNode(node, "powerLaw_distribution_2");

    GetNodeAttribute(simSettings, "variableSeed"    , variableSeedInt  );
    GetNodeAttribute(simSettings, "outputData"      , outputDataInt    );
    GetNodeAttribute(simSettings, "foodItemCount"   , foodItemCount    );
    GetNodeAttribute(simSettings, "foodDistribution", foodDistribution );
    GetNodeAttribute(simSettings, "nestPosition"    , nestPosition     );
    GetNodeAttribute(simSettings, "nestRadius"      , nestRadiusSquared);
    GetNodeAttribute(simSettings, "foodRadius"      , foodRadius       );
    GetNodeAttribute(cluster    , "numberOfClusters", numberOfClusters );
    GetNodeAttribute(cluster    , "clusterWidthX"   , clusterWidthX    );
    GetNodeAttribute(cluster    , "clusterLengthY"  , clusterLengthY   );
    GetNodeAttribute(powerLaw   , "powerRank"       , powerRank        );

    RNG                = CRandom::CreateRNG("argos");
    variableSeed       = (variableSeedInt == 1) ? (true) : (false);
    outputData         = (outputDataInt == 1)   ? (true) : (false);
	nestRadiusSquared *= nestRadiusSquared;
	foodRadiusSquared  = foodRadius * foodRadius;

    forageRangeX.Set(rangeX.GetX(), rangeX.GetY());
    forageRangeY.Set(rangeY.GetX(), rangeY.GetY());

    SetFoodDistribution();

    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>
                             (footBot.GetControllableEntity().GetController());

        c.SetFoodPositions(foodPositions);
        c.SetNestPosition(nestPosition);
        c.SetNestRadiusSquared(nestRadiusSquared);
        c.SetForageRange(forageRangeX, forageRangeY);
        c.SetFoodRadiusSquared(foodRadiusSquared);
        c.SetFramesPerSecond(ticks_per_second);
        c.SetLoopFunctions(this);

        iAnts.push_back(&c);
    }
}

CColor iAnt_loop_functions::GetFloorColor(const CVector2& position) {
    return CColor::WHITE;
}

void iAnt_loop_functions::PreStep() {
    size_t framesPerHalfSecond = (ticks_per_second / 2);
    simTime++;

    for(size_t i = 0; i < iAnts.size(); i++) {
		iAnts[i]->SetTime(simTime);
        iAnts[i]->SetFoodPositions(foodPositions);

        if(simTime % framesPerHalfSecond == 0) {
            string CPFA_ID = iAnts[i]->Get_CPFA_ID();

		    if(iAnts[i]->IsFindingFood() && !iAnts[i]->IsHoldingFood()) {
                if(CPFA_ID == "SEARCHING") {
                    // commenting this line out will prevent food from being removed from the map
                    //     food remains on the map, food item count un-edited
                    //     ants will not use food pickup behavior
                    foodPositions = UpdateFoodPositions(iAnts[i]);
                }
	        }
            /* when implementing food monitoring (i.e. food collection w/o nest return)
               this conditional must be changed to preserve pheromone behavior

               change to:
                   else if(iAnts[i]->IsHoldingFood()) {

            */
            else if(iAnts[i]->IsInTheNest() && iAnts[i]->IsHoldingFood()) {
                pheromoneList = UpdatePheromoneList(iAnts[i]);
                pheromonePositions = UpdatePheromonePositions(iAnts[i]);
            }
        }
    }

    fidelityPositions = UpdateFidelityPositions();
}

void iAnt_loop_functions::PostStep() {
    // ;-(
}

bool iAnt_loop_functions::IsExperimentFinished() {
    if(foodPositions.size() > 0) return false;

    for(size_t i = 0; i < iAnts.size(); i++) {
        if(iAnts[i]->IsHoldingFood() == true) return false;
        if(iAnts[i]->IsInTheNest() == false) return false;
    }

    return true;
}

void iAnt_loop_functions::PostExperiment() {
    // This variable is set in XML
    if(outputData == false) return;

    // This file is created in the directory where you run ARGoS
    // it is always created and/or appended to, never overwritten, i.e. ios::app
    ofstream dataOutput("iAntTagData.txt", ios::app);
    size_t time_in_minutes = floor(floor(simTime / ticks_per_second) / 60);

    // output to file
    if(dataOutput.tellp() == 0) {
        dataOutput << "tags_collected, time_in_minutes, random_seed\n";
    }

    dataOutput << foodItemCount - foodPositions.size() << ", ";
    dataOutput << time_in_minutes << ", " << random_seed << endl;
    dataOutput.close();

    // output to ARGoS GUI
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
}


void iAnt_loop_functions::Reset() {
    if(variableSeed) GetSimulator().SetRandomSeed(++random_seed);

    simTime = 0;
    foodPositions.clear();
    pheromoneList.clear();
    pheromonePositions.clear();
    SetFoodDistribution();

    for(size_t i = 0; i < iAnts.size(); i++) {
        iAnts[i]->SetFoodPositions(foodPositions);
        iAnts[i]->SetPheromonePositions(pheromonePositions);
        iAnts[i]->SetForageRange(forageRangeX, forageRangeY);
        iAnts[i]->Reset();
    }
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

/*******************************************************************************
* foodDistribution is set in the XML file
* random = 0, cluster = 1, power law = 2
*******************************************************************************/
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

    if(foodToPlace > 0) RandomFoodDistribution(foodToPlace);
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

vector<iAnt_pheromone> iAnt_loop_functions::UpdatePheromoneList(iAnt_controller *c) {
    vector<iAnt_pheromone> newPheromoneList;
	double                 maxStrength      = 0.0;

    c->DropOffFood();

	for(size_t i = 0; i < pheromoneList.size(); i++) {
        pheromoneList[i].Update(simTime);

        if(pheromoneList[i].IsActive() == true) {
            maxStrength += pheromoneList[i].Weight();
            newPheromoneList.push_back(pheromoneList[i]);
        }
    }

    double randomWeight = RNG->Uniform(CRange<double>(0.0, maxStrength));

    vector<iAnt_pheromone>::iterator i;

    for(i = pheromoneList.begin(); i != pheromoneList.end(); i++) {
	    if(randomWeight < i->Weight() && i->IsActive() == true) {
	    	c->SetTargetPheromone(*i);
	    	break;
	    }

        randomWeight -= i->Weight();
    }

    return newPheromoneList;
}

vector<CVector2> iAnt_loop_functions::UpdatePheromonePositions(iAnt_controller *c) {
    vector<CVector2> newPheromones;
    CVector2 p;

    for(size_t i = 0; i < pheromoneList.size(); i++) {
        p = pheromoneList[i].Location();

        if(pheromoneList[i].IsActive() == true) {
            bool duplicate = false;

            for(size_t j = 0; j < newPheromones.size(); j++) {
                if(newPheromones[j] == p) duplicate = true;
            }

            if(duplicate == false) newPheromones.push_back(p);
        }
    }

    c->SetPheromonePositions(pheromonePositions);

    return newPheromones;
}

vector<CVector2> iAnt_loop_functions::UpdateFoodPositions(iAnt_controller *c) {
    vector<CVector2> newFoodPositions;
    CVector2         p = c->GetPosition();
    CVector2         f;

    for(size_t i = 0; i < foodPositions.size(); i++) {
        f = foodPositions[i];

        if((p - f).SquareLength() < foodRadiusSquared) {
            c->PickupFood();
        }
        else newFoodPositions.push_back(f);
    }

    if(c->GetSharedPheromone().IsActive() == true) {
        pheromoneList.push_back(c->GetSharedPheromone());
    }

    return newFoodPositions;
}

vector<CVector2> iAnt_loop_functions::UpdateFidelityPositions() {
    vector<CVector2> newFidelityPositions;

    for(size_t i = 0; i < iAnts.size(); i++) {
        newFidelityPositions.push_back(iAnts[i]->GetFidelityPosition());
    }

    return newFidelityPositions;
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

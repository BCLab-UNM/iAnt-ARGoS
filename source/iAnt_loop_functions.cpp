#include "iAnt_loop_functions.h"

iAnt_loop_functions::iAnt_loop_functions() :
    SimTime(0),
    MaxSimTime(0),
    TicksPerSecond(16),
    ResourceDensityDelay(0),
    RandomSeed(1337),
    SimCounter(0),
    MaxSimCounter(1),
    VariableSeed(0),
    OutputData(0),
    TrailDensityRate(6),

    DrawTrails(0),
    DrawTargetRays(0),

    FoodDistribution(0),
    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),

    ProbabilityOfSwitchingToSearching(0.99999),
    ProbabilityOfReturningToNest(0.00001),
    UninformedSearchVariation(0.231256126),
    RateOfInformedSearchDecay(0.05),
    RateOfSiteFidelity(10.0),
    RateOfLayingPheromone(10.0),
    RateOfPheromoneDecay(0.05),

    NestRadius(0.25),
    NestRadiusSquared(0.0625),
    NestElevation(0.01),

    SearchRadius(0.0),
    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),

    ForageRangeX(-10.0, 10.0),
    ForageRangeY(-10.0, 10.0)
{}

/*****
 * Required by ARGoS. This function initializes global variables using
 * values from the XML configuration file supplied when ARGoS is run.
 *****/
void iAnt_loop_functions::Init(TConfigurationNode& node) {

    /* Temporary variables. */
    CSimulator     *simulator     = &GetSimulator();
    CPhysicsEngine *physicsEngine = &simulator->GetPhysicsEngine("default");
    CVector3        ArenaSize     = GetSpace().GetArenaSize();
    CVector2        rangeX        = CVector2(-ArenaSize.GetX()/2.0, ArenaSize.GetX()/2.0);
    CVector2        rangeY        = CVector2(-ArenaSize.GetY()/2.0, ArenaSize.GetY()/2.0);
    CDegrees        USV_InDegrees;

    /* Get each tag in the loop functions section of the XML file. */
    TConfigurationNode CPFA     = GetNode(node, "CPFA");
    TConfigurationNode simNode  = GetNode(node, "simulation");
    TConfigurationNode random   = GetNode(node, "_0_FoodDistribution_Random");
    TConfigurationNode cluster  = GetNode(node, "_1_FoodDistribution_Cluster");
    TConfigurationNode powerLaw = GetNode(node, "_2_FoodDistribution_PowerLaw");

    /* Initialize all loop functions variables from the XML file. */
    GetNodeAttribute(CPFA,     "ProbabilityOfSwitchingToSearching", data.ProbabilityOfSwitchingToSearching);
    GetNodeAttribute(CPFA,     "ProbabilityOfReturningToNest",      data.ProbabilityOfReturningToNest);
    GetNodeAttribute(CPFA,     "UninformedSearchVariation",         USV_InDegrees);
    GetNodeAttribute(CPFA,     "RateOfInformedSearchDecay",         data.RateOfInformedSearchDecay);
    GetNodeAttribute(CPFA,     "RateOfSiteFidelity",                data.RateOfSiteFidelity);
    GetNodeAttribute(CPFA,     "RateOfLayingPheromone",             data.RateOfLayingPheromone);
    GetNodeAttribute(CPFA,     "RateOfPheromoneDecay",              data.RateOfPheromoneDecay);
    GetNodeAttribute(simNode,  "MaxSimCounter",                     data.MaxSimCounter);
    GetNodeAttribute(simNode,  "MaxSimTime",                        data.MaxSimTime);
    GetNodeAttribute(simNode,  "VariableSeed",                      data.VariableSeed);
    GetNodeAttribute(simNode,  "OutputData",                        data.OutputData);
    GetNodeAttribute(simNode,  "NestPosition",                      data.NestPosition);
    GetNodeAttribute(simNode,  "NestRadius",                        data.NestRadius);
    GetNodeAttribute(simNode,  "FoodRadius",                        data.FoodRadius);
    GetNodeAttribute(simNode,  "FoodDistribution",                  data.FoodDistribution);
    GetNodeAttribute(random,   "FoodItemCount",                     data.FoodItemCount);
    GetNodeAttribute(cluster,  "NumberOfClusters",                  data.NumberOfClusters);
    GetNodeAttribute(cluster,  "ClusterWidthX",                     data.ClusterWidthX);
    GetNodeAttribute(cluster,  "ClusterLengthY",                    data.ClusterLengthY);
    GetNodeAttribute(powerLaw, "PowerRank",                         data.PowerRank);

    /* Convert and calculate additional values. */
    data.MaxSimTime                = data.MaxSimTime * data.TicksPerSecond;
    data.RandomSeed                = simulator->GetRandomSeed();
    data.TicksPerSecond            = physicsEngine->GetInverseSimulationClockTick();
    data.UninformedSearchVariation = ToRadians(USV_InDegrees);
    data.NestRadiusSquared         = (data.NestRadius) * (data.NestRadius);
    data.FoodRadiusSquared         = (data.FoodRadius + 0.04) * (data.FoodRadius + 0.04);
    data.SearchRadius              = (4.0 * data.FoodRadiusSquared);

    data.ForageRangeX.Set(rangeX.GetX() + (2.0 * data.FoodRadius), rangeX.GetY() - (2.0 * data.FoodRadius));
    data.ForageRangeY.Set(rangeY.GetX() + (2.0 * data.FoodRadius), rangeY.GetY() - (2.0 * data.FoodRadius));

    RNG = CRandom::CreateRNG("argos");
    data.RNG = RNG;

    /* Store the iAnts in a more friendly, human-readable structure. */
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        //iAnts.push_back(&c);
        c.SetData(&data);    // all iAnts get a pointer to the iAnt_loop_functions object
        c.SetLoopFunctions(this);
    }

    /* Set up the food distribution based on the XML file. */
    data.SetFoodDistribution();
}

/*****
 * This hook function is called before iAnts call their ControlStep() function.
 *****/
void iAnt_loop_functions::PreStep() {

    data.SimTime++;
    UpdatePheromoneList();

    if(data.SimTime > data.ResourceDensityDelay) {
        for(size_t i = 0; i < data.FoodColoringList.size(); i++) {
            data.FoodColoringList[i] = CColor::BLACK;
        }
    }

    if(data.FoodList.size() == 0) {
        data.FidelityList.clear();
        data.TargetRayList.clear();
        data.PheromoneList.clear();
    }

    /*
    if(data.SimTime % (16 * 10) == 0) {
        LOG << "pheromones: " << data.PheromoneList.size() << endl;
        LOG << "fidelities: " << data.FidelityList.size() << endl;
        LOG << "target ray list: " << data.TargetRayList.size() << endl << endl;
    }*/
}

/*****
 * This hook function is called after iAnts call their ControlStep() function.
 *****/
void iAnt_loop_functions::PostStep() {
    // TODO: add data tracking code for food collected by each robot
}

/*****
 * This function is called once all food is collected or the
 * time limit imposed in the XML file has been reached.
 *****/
void iAnt_loop_functions::PostExperiment() {
    size_t time_in_minutes = floor(floor(data.SimTime/data.TicksPerSecond)/60);
    size_t collectedFood = data.FoodItemCount - data.FoodList.size();

    // This variable is set in XML
    if(data.OutputData == 1) {
        // This file is created in the directory where you run ARGoS
        // it is always created or appended to, never overwritten, i.e. ios::app
        ofstream dataOutput("iAntTagdata.txt", ios::app);

        // output to file
        if(dataOutput.tellp() == 0) {
            dataOutput << "tags_collected, time_in_minutes, random_seed\n";
        }

        dataOutput << collectedFood << ", ";
        dataOutput << time_in_minutes << ", " << data.RandomSeed << endl;
        dataOutput.close();
    }

    // output to ARGoS GUI
    if(data.SimCounter == 0) {
        LOG << "\ntags_collected, time_in_minutes, random_seed\n";
        LOG << collectedFood << ", ";
        LOG << time_in_minutes << ", " << data.RandomSeed << endl;
    } else {
        LOG << collectedFood << ", ";
        LOG << time_in_minutes << ", " << data.RandomSeed << endl;

        /*
        ifstream dataInput("iAntTagdata.txt");
        string s;

        while(getline(dataInput, s)) {
            LOG << s << endl;
        }

        dataInput.close();
        */
    }

    data.SimCounter++;
}

/*****
 * This function resets all iAnts and restarts the simulation based on initial
 * conditions set in the XML file.
 *****/
void iAnt_loop_functions::Reset() {
    if(data.VariableSeed == 1) GetSimulator().SetRandomSeed(++data.RandomSeed);

    //GetSimulator().Reset();
    GetSpace().Reset();
    data.SimTime = 0;
    data.ResourceDensityDelay = 0;
    data.MaxSimCounter = data.SimCounter;
    data.SimCounter = 0;
    data.FoodList.clear();
    data.PheromoneList.clear();
    data.FidelityList.clear();
    data.TargetRayList.clear();
    data.SetFoodDistribution();

    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        c.Reset();
    }
}

/*****
 * An experiment is considered finished if all food items are collected and all iAnts have returned their food to the
 * nest. ARGoS also keeps track of the time limit in the XML file and will stop the experiment at that time limit.
 *****/
bool iAnt_loop_functions::IsExperimentFinished() {

    bool isFinished = false;

    if(data.FoodList.size() == 0 || data.SimTime >= data.MaxSimTime) {
        isFinished = true;
    }

    if(isFinished == true && data.MaxSimCounter > 1) {
        size_t newSimCounter = data.SimCounter + 1;
        size_t newMaxSimCounter = data.MaxSimCounter - 1;

        // LOG << endl << "FINISHED RUN: " << data.SimCounter << endl;

        PostExperiment();
        Reset();

        data.SimCounter    = newSimCounter;
        data.MaxSimCounter = newMaxSimCounter;
        isFinished         = false;
    }

    return isFinished;
}

/*****
 *
 *****/
void iAnt_loop_functions::UpdatePheromoneList() {
 
    //LOG << "Hello, world! " << data.PheromoneList.size() << endl << endl;

    vector<iAnt_pheromone> new_p_list;

    for(size_t i = 0; i < data.PheromoneList.size(); i++) {

        data.PheromoneList[i].Update((Real)(data.SimTime / data.TicksPerSecond));

        //if(data.PheromoneList[i].IsActive()) LOG << "O" << endl;
        //else LOG << "X" << endl;

        if(data.PheromoneList[i].IsActive() == true) {
            new_p_list.push_back(data.PheromoneList[i]);
        }
    }

    //LOG << endl;

    data.PheromoneList = new_p_list;
}

/*****
 *
 *****/
void iAnt_loop_functions::SetFoodDistribution() {
    switch(FoodDistribution) {
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

/*****
 *
 *****/
void iAnt_loop_functions::RandomFoodDistribution() {
    FoodList.clear();

    CVector2 placementPosition;

    for(size_t i = 0; i < FoodItemCount; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX),
                              RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX),
                                  RNG->Uniform(ForageRangeY));
        }

        FoodList.push_back(placementPosition);
        FoodColoringList.push_back(CColor::BLACK);
    }
}

/*****
 *
 *****/
void iAnt_loop_functions::ClusterFoodDistribution() {

    Real     foodOffset  = 3.0 * FoodRadius;
    size_t   foodToPlace = NumberOfClusters * ClusterWidthX * ClusterLengthY;
    CVector2 placementPosition;

    FoodItemCount = foodToPlace;

    for(size_t i = 0; i < NumberOfClusters; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        }

        for(size_t j = 0; j < ClusterLengthY; j++) {
            for(size_t k = 0; k < ClusterWidthX; k++) {
                FoodList.push_back(placementPosition);
                FoodColoringList.push_back(CColor::BLACK);
                placementPosition.SetX(placementPosition.GetX() + foodOffset);
            }

            placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
            placementPosition.SetY(placementPosition.GetY() + foodOffset);
        }
    }
}

/*****
 *
 *****/
void iAnt_loop_functions::PowerLawFoodDistribution() {
    Real   foodOffset     = 3.0 * FoodRadius;
    size_t foodPlaced     = 0;
    size_t powerLawLength = 1;
    size_t maxTrials      = 200;
    size_t trialCount     = 0;

    vector<size_t> powerLawClusters;
    vector<size_t> clusterSides;
    CVector2       placementPosition;

    for(size_t i = 0; i < PowerRank; i++) {
        powerLawClusters.push_back(powerLawLength * powerLawLength);
        powerLawLength *= 2;
    }

    for(size_t i = 0; i < PowerRank; i++) {
        powerLawLength /= 2;
        clusterSides.push_back(powerLawLength);
    }

    for(size_t h = 0; h < powerLawClusters.size(); h++) {
        for(size_t i = 0; i < powerLawClusters[h]; i++) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

            while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
                trialCount++;
                placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

                if(trialCount > maxTrials) {
                    LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
                    break;
                }
            }

            for(size_t j = 0; j < clusterSides[h]; j++) {
                for(size_t k = 0; k < clusterSides[h]; k++) {
                    foodPlaced++;
                    FoodList.push_back(placementPosition);
                    FoodColoringList.push_back(CColor::BLACK);
                    placementPosition.SetX(placementPosition.GetX() + foodOffset);
                }

                placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
                placementPosition.SetY(placementPosition.GetY() + foodOffset);
            }
        }
    }

    FoodItemCount = foodPlaced;
}

/*****
 *
 *****/
bool iAnt_loop_functions::IsOutOfBounds(CVector2 p, size_t length, size_t width) {
    CVector2 placementPosition = p;

    Real foodOffset   = 3.0 * FoodRadius;
    Real widthOffset  = 3.0 * FoodRadius * (Real)width;
    Real lengthOffset = 3.0 * FoodRadius * (Real)length;

    Real x_min = p.GetX() - FoodRadius;
    Real x_max = p.GetX() + FoodRadius + widthOffset;

    Real y_min = p.GetY() - FoodRadius;
    Real y_max = p.GetY() + FoodRadius + lengthOffset;

    if((x_min < (ForageRangeX.GetMin() + FoodRadius)) ||
       (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
       (y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
       (y_max > (ForageRangeY.GetMax() - FoodRadius))) {
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

/*****
 *
 *****/
bool iAnt_loop_functions::IsCollidingWithNest(CVector2 p) {
    Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
    Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    return ((p - NestPosition).SquareLength() < NRPB_squared);
}

/*****
 *
 *****/
bool iAnt_loop_functions::IsCollidingWithFood(CVector2 p) {
    Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
    Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

    for(size_t i = 0; i < FoodList.size(); i++) {
        if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
    }

    return false;
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions");
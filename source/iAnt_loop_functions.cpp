#include "iAnt_loop_functions.h"

/*****
 * The constructor function is used only to initialize variables to null/0 values. Primary setup is done with Init().
 *****/
iAnt_loop_functions::iAnt_loop_functions() :
    SimTime(0),
    MaxSimTime(0),
    TicksPerSecond(0),
    ResourceDensityDelay(0),
    RandomSeed(0),
    SimCounter(0),
    MaxSimCounter(0),
    VariableFoodPlacement(0),
    OutputData(0),
    DrawDensityRate(0),
    DrawIDs(0),
    DrawTrails(0),
    DrawTargetRays(0),
    FoodDistribution(0),
    FoodItemCount(0),
    NumberOfClusters(0),
    ClusterWidthX(0),
    ClusterLengthY(0),
    PowerRank(0),
    ProbabilityOfSwitchingToSearching(0.0),
    ProbabilityOfReturningToNest(0.0),
    UninformedSearchVariation(0.0),
    RateOfInformedSearchDecay(0.0),
    RateOfSiteFidelity(0.0),
    RateOfLayingPheromone(0.0),
    RateOfPheromoneDecay(0.0),
    NestRadius(0.0),
    NestRadiusSquared(0.0),
    NestElevation(0.0),
    SearchRadius(0.0),
    FoodRadius(0.0),
    FoodRadiusSquared(0.0),
    ForageRangeX(-1.0, 1.0),
    ForageRangeY(-1.0, 1.0)
{}

/*****
 * Required by ARGoS. This function initializes global variables from the XML configuration file.
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
    TConfigurationNode CPFA      = GetNode(node, "CPFA");
    TConfigurationNode simNode   = GetNode(node, "simulation");
    TConfigurationNode random    = GetNode(node, "_0_FoodDistribution_Random");
    TConfigurationNode cluster   = GetNode(node, "_1_FoodDistribution_Cluster");
    TConfigurationNode powerLaw  = GetNode(node, "_2_FoodDistribution_PowerLaw");
   
    /* Initialize all loop functions variables from the XML file. */
    GetNodeAttribute(CPFA,     "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
    GetNodeAttribute(CPFA,     "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
    GetNodeAttribute(CPFA,     "UninformedSearchVariation",         USV_InDegrees);
    GetNodeAttribute(CPFA,     "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
    GetNodeAttribute(CPFA,     "RateOfSiteFidelity",                RateOfSiteFidelity);
    GetNodeAttribute(CPFA,     "RateOfLayingPheromone",             RateOfLayingPheromone);
    GetNodeAttribute(CPFA,     "RateOfPheromoneDecay",              RateOfPheromoneDecay);
    GetNodeAttribute(simNode,  "MaxSimCounter",                     MaxSimCounter);
    GetNodeAttribute(simNode,  "MaxSimTime",                        MaxSimTime);
    GetNodeAttribute(simNode,  "VariableFoodPlacement",             VariableFoodPlacement);
    GetNodeAttribute(simNode,  "OutputData",                        OutputData);
    GetNodeAttribute(simNode,  "ResourceDensityDelay",              ResourceDensityDelay);
    GetNodeAttribute(simNode,  "DrawDensityRate",                   DrawDensityRate);
    GetNodeAttribute(simNode,  "DrawIDs",                           DrawIDs);
    GetNodeAttribute(simNode,  "DrawTrails",                        DrawTrails);
    GetNodeAttribute(simNode,  "DrawTargetRays",                    DrawTargetRays);
    GetNodeAttribute(simNode,  "NestPosition",                      NestPosition);
    GetNodeAttribute(simNode,  "NestRadius",                        NestRadius);
    GetNodeAttribute(simNode,  "NestElevation",                     NestElevation);
    GetNodeAttribute(simNode,  "FoodRadius",                        FoodRadius);
    GetNodeAttribute(simNode,  "FoodDistribution",                  FoodDistribution);
    GetNodeAttribute(random,   "FoodItemCount",                     FoodItemCount);
    GetNodeAttribute(cluster,  "NumberOfClusters",                  NumberOfClusters);
    GetNodeAttribute(cluster,  "ClusterWidthX",                     ClusterWidthX);
    GetNodeAttribute(cluster,  "ClusterLengthY",                    ClusterLengthY);
    GetNodeAttribute(powerLaw, "PowerRank",                         PowerRank);
    
    RNG = CRandom::CreateRNG("argos");

    /* Convert and calculate additional values. */
    TicksPerSecond            = physicsEngine->GetInverseSimulationClockTick();
    RandomSeed                = simulator->GetRandomSeed();
    UninformedSearchVariation = ToRadians(USV_InDegrees);
    NestRadiusSquared         = (NestRadius) * (NestRadius);
    MaxSimTime                = MaxSimTime * TicksPerSecond;
    ResourceDensityDelay      = ResourceDensityDelay * TicksPerSecond;

    /* Compensate for the radius of the footbot and scale the search radius to the size of food. */
    FoodRadiusSquared         = (FoodRadius + 0.04) * (FoodRadius + 0.04);
    SearchRadius              = (4.0 * FoodRadiusSquared);

    /* Define the boundary of the arena where iAnts can search */
    ForageRangeX.Set(rangeX.GetX() + (2.0 * FoodRadius), rangeX.GetY() - (2.0 * FoodRadius));
    ForageRangeY.Set(rangeY.GetX() + (2.0 * FoodRadius), rangeY.GetY() - (2.0 * FoodRadius));


   
    // until dsa loop functions is written; disable this
    // Send a pointer to this loop functions object to each controller.
    NumberOfRobots = 0;
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        //CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        //iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        // c.SetLoopFunctions(&this);
        // set robot pattern vector, etc.
        // MoveEntity(footBot.GetEmbodiedEntity(), c.GetStartPosition(), CQuaternion(), false);
        NumberOfRobots++;
    }

/*
    // Send a pointer to this loop functions object to each controller.
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;
    size_t waitTime = 0;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        DSA_controller& c = dynamic_cast<DSA_controller&>(footBot.GetControllableEntity().GetController());

        // linker problems... ?!
        //c.SetStop(waitTime);
        waitTime += 5;
        // c.SetLoopFunctions(&this);
        // set robot pattern vector, etc.
        // MoveEntity(footBot.GetEmbodiedEntity(), c.GetStartPosition(), CQuaternion(), false);

    }
*/

    /* Set up the food distribution based on the XML file. */
    SetFoodDistribution();
}

/*****
 * This hook function is called before iAnts call their ControlStep() function.
 *****/
void iAnt_loop_functions::PreStep() {

    SimTime++;
    UpdatePheromoneList();

    if(SimTime > ResourceDensityDelay) {
        for(size_t i = 0; i < FoodColoringList.size(); i++) {
            FoodColoringList[i] = CColor::BLACK;
        }
    }

    if(FoodList.size() == 0) {
        FidelityList.clear();
        TargetRayList.clear();
        PheromoneList.clear();
    }

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
    size_t time_in_minutes = floor(floor(SimTime/TicksPerSecond)/60);
    size_t collectedFood = FoodItemCount - FoodList.size();
    string distribution;


//    DSA_controller x;
    // Send a pointer to this loop functions object to each controller.
    // CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    // CSpace::TMapPerType::iterator it;
    int numberOfSpirals = 0;

    // for(it = footbots.begin(); it != footbots.end(); it++) {
    //     CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
    //     DSA_controller& c = dynamic_cast<DSA_controller&>(footBot.GetControllableEntity().GetController());

    //     //numberOfSpirals = c.getNumberOfSpirals();
    //     break;
    // }



    // This variable is set in XML
    if(OutputData == 1) {
        // This file is created in the directory where you run ARGoS
        // it is always created or appended to, never overwritten, i.e. ios::app
        ofstream dataOutput("iAntTagData.txt", ios::app);

        // write out spiral stuff



        // output to file
        if(dataOutput.tellp() == 0) {
            if (FoodDistribution == 0){
                distribution = "Random";
            }
            else if (FoodDistribution == 1){
                distribution = "Cluster";
            }
            else{
                distribution = "PowerLaw";
            }
            dataOutput  << "Algorithm                         = " << "CFPA/DSA"                        <<endl;

            //paremters for DSA
            dataOutput  << "N_robots                          = " << NumberOfRobots                    <<endl;
            dataOutput  << "N_circuits                        = " << "?"                               <<endl;
            dataOutput  << "SearcherGap                       = " << FoodRadiusSquared                 <<endl;


            //parameters for CPFA
            dataOutput  << "FoodDistribution                  = " << distribution                      <<endl;
            dataOutput  << "FoodItemCount                     = " << FoodItemCount                     <<endl;
            dataOutput  << "NumberOfClusters                  = " << NumberOfClusters                  <<endl;
            dataOutput  << "ClusterWidthX                     = " << ClusterWidthX                     <<endl;
            dataOutput  << "ClusterLengthY                    = " << ClusterLengthY                    <<endl;
            dataOutput  << "PowerRank                         = " << PowerRank                         <<endl;
            dataOutput  << "ProbabilityOfSwitchingToSearching = " << ProbabilityOfSwitchingToSearching <<endl;
            dataOutput  << "ProbabilityOfReturningToNest      = " << ProbabilityOfReturningToNest      <<endl;
            dataOutput  << "UninformedSearchVariation         = " << UninformedSearchVariation         <<endl;
            dataOutput  << "RateOfInformedSearchDecay         = " << RateOfInformedSearchDecay         <<endl;
            dataOutput  << "RateOfSiteFidelity                = " << RateOfSiteFidelity                <<endl;
            dataOutput  << "RateOfLayingPheromone             = " << RateOfLayingPheromone             <<endl;
            dataOutput  << "RateOfPheromoneDecay              = " << RateOfPheromoneDecay              <<endl;
            dataOutput  << endl;

            dataOutput << "tags_collected, time_in_minutes, random_seed, experiment_number\n";
        }
        


        dataOutput << collectedFood << ",     ";
        dataOutput << time_in_minutes << ",     ";
        dataOutput << RandomSeed << ",     ";
        dataOutput << SimCounter + 1 << endl;
        dataOutput.close();
    }

    // output to ARGoS GUI
    if(SimCounter == 0) {
        LOG << "\ntags_collected, time_in_minutes, random_seed, experiment_number\n";
        LOG << collectedFood << ",     ";
        LOG << time_in_minutes << ",     ";
        LOG << RandomSeed << ",     ";
        LOG << SimCounter + 1 << endl;
    } else {
        LOG << collectedFood << ",     ";
        LOG << time_in_minutes << ",     ";
        LOG << RandomSeed << ",     ";
        LOG << SimCounter + 1 << endl;
    }

    SimCounter++;

    // add a variable time feature???
    // MaxSimTime += (TicksPerSecond * 60);
}

/*****
 * This function resets all iAnts and restarts the simulation based on initial
 * conditions set in the XML file.
 *****/
void iAnt_loop_functions::Reset() {

    if(VariableFoodPlacement == 0) {
        RNG->Reset();
    }

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    SimTime = 0;
    ResourceDensityDelay = 0;
    MaxSimCounter = SimCounter;
    SimCounter = 0;

    FoodList.clear();
    FoodColoringList.clear();
    PheromoneList.clear();
    FidelityList.clear();
    TargetRayList.clear();

    SetFoodDistribution();

    /* disable placement reset... for now (it currently does not work with the addition of dsa controllers */
    /*
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>(footBot.GetControllableEntity().GetController());

        MoveEntity(footBot.GetEmbodiedEntity(), c.GetStartPosition(), CQuaternion(), false);
    }
    */
}

/*****
 * An experiment is considered finished if all food items are collected and all iAnts have returned their food to the
 * nest. ARGoS also keeps track of the time limit in the XML file and will stop the experiment at that time limit.
 *****/
bool iAnt_loop_functions::IsExperimentFinished() {

    bool isFinished = false;

    if(FoodList.size() == 0 || SimTime >= MaxSimTime) {
        isFinished = true;
    }

    if(isFinished == true && MaxSimCounter > 1) {
        size_t newSimCounter = SimCounter + 1;
        size_t newMaxSimCounter = MaxSimCounter - 1;

        PostExperiment();
        Reset();

        SimCounter    = newSimCounter;
        MaxSimCounter = newMaxSimCounter;
        isFinished    = false;
    }

    return isFinished;
}

/*****
 *
 *****/
void iAnt_loop_functions::UpdatePheromoneList() {
 
    //LOG << "Hello, world! " << PheromoneList.size() << endl << endl;

    vector<iAnt_pheromone> new_p_list;

    for(size_t i = 0; i < PheromoneList.size(); i++) {

        PheromoneList[i].Update((Real)(SimTime / TicksPerSecond));

        if(PheromoneList[i].IsActive() == true) {
            new_p_list.push_back(PheromoneList[i]);
        }
    }

    //LOG << endl;

    PheromoneList = new_p_list;
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
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
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
    size_t   foodPlaced = 0;
    CVector2 placementPosition;

    FoodItemCount = foodToPlace;

    for(size_t i = 0; i < NumberOfClusters; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        }

        for(size_t j = 0; j < ClusterLengthY; j++) {
            for(size_t k = 0; k < ClusterWidthX; k++) {

                foodPlaced++;
                /*
                #include <argos3/plugins/simulator/entities/box_entity.h>

                string label("my_box_");
                label.push_back('0' + foodPlaced++);

                CBoxEntity *b = new CBoxEntity(label,
                                               CVector3(placementPosition.GetX(), placementPosition.GetY(), 0.0),
                                               CQuaternion(),
                                               true,
                                               CVector3(0.1, 0.1, 0.001),
                                               1.0);
                AddEntity(*b);
                */

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
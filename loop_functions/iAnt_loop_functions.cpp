#include "iAnt_loop_functions.h"

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
    TConfigurationNode random   = GetNode(node, "FoodDistribution_Random_0");
    TConfigurationNode cluster  = GetNode(node, "FoodDistribution_Cluster_1");
    TConfigurationNode powerLaw = GetNode(node, "FoodDistribution_PowerLaw_2");

    /* Initialize all loop functions variables from the XML file. */
    GetNodeAttribute(CPFA, "ProbabilityOfSwitchingToSearching",
                           data.ProbabilityOfSwitchingToSearching);
    GetNodeAttribute(CPFA, "ProbabilityOfReturningToNest",
                           data.ProbabilityOfReturningToNest);
    GetNodeAttribute(CPFA, "UninformedSearchVariation",
                           USV_InDegrees);
    GetNodeAttribute(CPFA, "RateOfInformedSearchDecay",
                           data.RateOfInformedSearchDecay);
    GetNodeAttribute(CPFA, "RateOfSiteFidelity",
                           data.RateOfSiteFidelity);
    GetNodeAttribute(CPFA, "RateOfLayingPheromone",
                           data.RateOfLayingPheromone);
    GetNodeAttribute(CPFA, "RateOfPheromoneDecay",
                           data.RateOfPheromoneDecay);
    GetNodeAttribute(simNode,  "VariableSeed",     data.VariableSeed);
    GetNodeAttribute(simNode,  "OutputData",       data.OutputData);
    GetNodeAttribute(simNode,  "NestPosition",     data.NestPosition);
    GetNodeAttribute(simNode,  "NestRadius",       data.NestRadius);
    GetNodeAttribute(simNode,  "FoodRadius",       data.FoodRadius);
    GetNodeAttribute(simNode,  "FoodDistribution", data.FoodDistribution);
    GetNodeAttribute(random,   "FoodItemCount",    data.FoodItemCount);
    GetNodeAttribute(cluster,  "NumberOfClusters", data.NumberOfClusters);
    GetNodeAttribute(cluster,  "ClusterWidthX",    data.ClusterWidthX);
    GetNodeAttribute(cluster,  "ClusterLengthY",   data.ClusterLengthY);
    GetNodeAttribute(powerLaw, "PowerRank",        data.PowerRank);

    /* Convert and calculate additional values. */
    data.RandomSeed                = simulator->GetRandomSeed();
    data.TicksPerSecond            = physicsEngine->GetInverseSimulationClockTick();
    data.UninformedSearchVariation = ToRadians(USV_InDegrees);
    data.NestRadiusSquared         = data.NestRadius * data.NestRadius;
    data.FoodRadiusSquared         = data.FoodRadius * data.FoodRadius;
    data.ForageRangeX.Set(rangeX.GetX() + (2.0 * data.FoodRadius),
                          rangeX.GetY() + (2.0 * data.FoodRadius));
    data.ForageRangeY.Set(rangeY.GetX() + (2.0 * data.FoodRadius),
                          rangeY.GetY() + (2.0 * data.FoodRadius));
    RNG = CRandom::CreateRNG("argos");
    data.RNG = RNG;

    /* Store the iAnts in a more friendly, human-readable structure. */
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    for(it = footbots.begin(); it != footbots.end(); it++) {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        iAnt_controller& c = dynamic_cast<iAnt_controller&>
                             (footBot.GetControllableEntity().GetController());
        iAnts.push_back(&c);
        c.SetData(&data);    // all iAnts get a pointer to the iAnt_data object
    }

    /* Set up the food distribution based on the XML file. */
    data.SetFoodDistribution();
}

/*****
 * This hook function is called before iAnts call their ControlStep() function.
 *****/
void iAnt_loop_functions::PreStep() {
    data.SimTime++;
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
    // This variable is set in XML
    if(data.OutputData == false) return;

    // This file is created in the directory where you run ARGoS
    // it is always created or appended to, never overwritten, i.e. ios::app
    ofstream dataOutput("iAntTagData.txt", ios::app);

    size_t time_in_minutes = floor(floor(data.SimTime/data.TicksPerSecond)/60);
    size_t collectedFood = data.FoodItemCount - data.FoodList.size();

    // output to file
    if(dataOutput.tellp() == 0) {
        dataOutput << "tags_collected, time_in_minutes, random_seed\n";
    }

    dataOutput << collectedFood << ", ";
    dataOutput << time_in_minutes << ", " << data.RandomSeed << endl;
    dataOutput.close();

    // output to ARGoS GUI
    if(data.SimCounter == 0) {
        LOG << "\ntags_collected, time_in_minutes, random_seed\n";
        LOG << collectedFood << ", ";
        LOG << time_in_minutes << ", " << data.RandomSeed << endl;
    } else {
        ifstream dataInput("iAntTagData.txt");
        string s;

        while(getline(dataInput, s)) {
            LOG << s << endl;
        }

        dataInput.close();
    }

    data.SimCounter++;
}

/*****
 * This function resets all iAnts and restarts the simulation based on initial
 * conditions set in the XML file.
 *****/
void iAnt_loop_functions::Reset() {
    if(data.VariableSeed == 1) GetSimulator().SetRandomSeed(++data.RandomSeed);

    data.SimTime = 0;
    data.FoodList.clear();
    data.PheromoneList.clear();
    data.SetFoodDistribution();

    for(size_t i = 0; i < iAnts.size(); i++) {
        iAnts[i]->Reset();
    }
}

/*****
 * An experiment is considered finished if all food items are collected and all
 * iAnts have returned their food to the nest. ARGoS also keeps track of the
 * time limit in the XML file and will stop the experiment at that time limit.
 *****/
bool iAnt_loop_functions::IsExperimentFinished() {
    if(data.FoodList.size() > 0) return false;

    for(size_t i = 0; i < iAnts.size(); i++) {
        if(iAnts[i]->IsHoldingFood() == true) return false;
    }

    return true;
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

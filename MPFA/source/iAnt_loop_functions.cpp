#include "iAnt_loop_functions.h"
#include "iAnt_nest.h" //qilu 07/05
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
    data.ArenaX = ArenaSize.GetX();//qilu 06/07
    data.ArenaY = ArenaSize.GetY();
    CVector2		nestPosition;//qilu 07/05
    CDegrees        USV_InDegrees;

    /* Get each tag in the loop functions section of the XML file. */
    TConfigurationNode MPFA     = GetNode(node, "MPFA");
    TConfigurationNode simNode  = GetNode(node, "simulation");
    TConfigurationNode random   = GetNode(node, "_0_FoodDistribution_Random");
    TConfigurationNode cluster  = GetNode(node, "_1_FoodDistribution_Cluster");
    TConfigurationNode powerLaw = GetNode(node, "_2_FoodDistribution_PowerLaw");

    /* Initialize all loop functions variables from the XML file. */
    GetNodeAttribute(MPFA, "ProbabilityOfSwitchingToSearching",
                           data.ProbabilityOfSwitchingToSearching);
    GetNodeAttribute(MPFA, "ProbabilityOfReturningToNest",
                           data.ProbabilityOfReturningToNest);
    GetNodeAttribute(MPFA, "UninformedSearchVariation",
                           USV_InDegrees);
    GetNodeAttribute(MPFA, "RateOfInformedSearchDecay",
                           data.RateOfInformedSearchDecay);
    GetNodeAttribute(MPFA, "RateOfSiteFidelity",
                           data.RateOfSiteFidelity);
    GetNodeAttribute(MPFA, "RateOfLayingPheromone",
                           data.RateOfLayingPheromone);
    GetNodeAttribute(MPFA, "RateOfPheromoneDecay",
                           data.RateOfPheromoneDecay);
    GetNodeAttribute(simNode,  "MaxSimCounter",    data.MaxSimCounter);
    GetNodeAttribute(simNode,  "VariableSeed",     data.VariableSeed);
    GetNodeAttribute(simNode,  "OutputData",       data.OutputData);
    
    GetNodeAttribute(simNode, "nestPosition_0"   , nestPosition     );
    iAnt_nest nest0= iAnt_nest(nestPosition); //qilu 07/05
    data.nests.push_back(nest0);
    GetNodeAttribute(simNode, "nestPosition_1"    , nestPosition    );
    iAnt_nest nest1= iAnt_nest(nestPosition);
    data.nests.push_back(nest1);
    
    GetNodeAttribute(simNode, "nestPosition_2"    , nestPosition    );
    iAnt_nest nest2= iAnt_nest(nestPosition);
    data.nests.push_back(nest2);
    GetNodeAttribute(simNode, "nestPosition_3"    , nestPosition    );
    iAnt_nest nest3=iAnt_nest(nestPosition);
    data.nests.push_back(nest3);
    
//    GetNodeAttribute(simNode, "nestPosition_4"   , data.NestPositions[4]     );//nestPosition_0 is defined in iAnt_data.h
//    GetNodeAttribute(simNode, "nestPosition_5"    , data.NestPositions[5]    );
//    GetNodeAttribute(simNode, "nestPosition_6"    , data.NestPositions[6]    );//nestPosition_0 is defined in iAnt_data.h
//    GetNodeAttribute(simNode, "nestPosition_7"    , data.NestPositions[7]    );
//    GetNodeAttribute(simNode, "nestPosition_8"   , data.NestPositions[8]     );//nestPosition_0 is defined in iAnt_data.h
//    GetNodeAttribute(simNode, "nestPosition_9"    , data.NestPositions[9]    );
//    GetNodeAttribute(simNode, "nestPosition_10"    , data.NestPositions[10]    );//nestPosition_0 is defined in iAnt_data.h
//    GetNodeAttribute(simNode, "nestPosition_11"    , data.NestPositions[11]    );
//    GetNodeAttribute(simNode, "nestPosition_12"   , data.NestPositions[12]     );//nestPosition_0 is defined in iAnt_data.h
//    GetNodeAttribute(simNode, "nestPosition_13"    , data.NestPositions[13]    );
//    GetNodeAttribute(simNode, "nestPosition_14"    , data.NestPositions[14]    );//nestPosition_0 is defined in iAnt_data.h
//    GetNodeAttribute(simNode, "nestPosition_15"    , data.NestPositions[15]    );
    
    GetNodeAttribute(simNode,  "NestRadius",       data.NestRadius);
    GetNodeAttribute(simNode,  "FoodRadius",       data.FoodRadius);
    GetNodeAttribute(simNode,  "FoodDistribution", data.FoodDistribution);
    GetNodeAttribute(random,   "FoodItemCount",    data.FoodItemCount);
    GetNodeAttribute(cluster,  "NumberOfClusters", data.NumberOfClusters);
    GetNodeAttribute(cluster,  "ClusterWidthX",    data.ClusterWidthX);
    GetNodeAttribute(cluster,  "ClusterLengthY",   data.ClusterLengthY);
    GetNodeAttribute(powerLaw, "PowerRank",        data.PowerRank);
    GetNodeAttribute(powerLaw, "PowerLawCopies",      data.PowerLawCopies);//qilu 08/14/2015
    
    /* Convert and calculate additional values. */
    //data.MaxSimTime                = simulator->GetMaxSimulationClock(); //qilu 07/19 
    data.RandomSeed                = simulator->GetRandomSeed();
    data.TicksPerSecond            = physicsEngine->GetInverseSimulationClockTick();
    data.UninformedSearchVariation = ToRadians(USV_InDegrees);
    data.NestRadiusSquared         = (data.NestRadius) * (data.NestRadius);
    data.FoodRadiusSquared         = (data.FoodRadius + 0.04) * (data.FoodRadius + 0.04);
    data.SearchRadius              = (4.0 * data.FoodRadiusSquared);

    data.ForageRangeX.Set(rangeX.GetX() + (2.0 * data.FoodRadius),
                          rangeX.GetY() - (2.0 * data.FoodRadius));
    data.ForageRangeY.Set(rangeY.GetX() + (2.0 * data.FoodRadius),
                          rangeY.GetY() - (2.0 * data.FoodRadius));

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
    data.UpdatePheromoneList();//qilu 07/05

    if(data.SimTime > data.ResourceDensityDelay) {
        for(size_t i = 0; i < data.FoodColoringList.size(); i++) {
            data.FoodColoringList[i] = CColor::BLACK;
        }
    }

    if(data.FoodList.size() == 0) {
		for(size_t i=0; i<data.nests.size(); i++)
			data.nests[i].PheromoneList.clear();//qilu 07/13
        data.FidelityList.clear();//qilu 07/16
        data.TargetRayList.clear();//qilu 07/13
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
    size_t time_in_minutes = floor(floor(data.SimTime/data.TicksPerSecond)/60);
    size_t collectedFood = data.FoodItemCount - data.FoodList.size();

    // This variable is set in XML
    if(data.OutputData == 1) {
        // This file is created in the directory where you run ARGoS
        // it is always created or appended to, never overwritten, i.e. ios::app
        ofstream dataOutput("iAntTagData.txt", ios::app);
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

    }

    data.SimCounter++;
}

/*****
 * This function resets all iAnts and restarts the simulation based on initial
 * conditions set in the XML file.
 *****/
void iAnt_loop_functions::Reset() {
    if(data.VariableSeed == 1) GetSimulator().SetRandomSeed(++data.RandomSeed);

    GetSimulator().Reset();
    GetSpace().Reset();
    data.SimTime = 0;
    data.ResourceDensityDelay = 0;
    data.FoodList.clear();
    for(size_t i=0; i<data.nests.size(); i++)
		data.nests[i].PheromoneList.clear();//qilu 07/13
		
	data.FidelityList.clear();//qilu 07/16
	data.TargetRayList.clear();//qilu 07/13
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
    bool isFinished = false;
    for(size_t i = 0; i < iAnts.size(); i++) {//qilu 06/07
        if(iAnts[i]->IsHoldingFood() == true){
            isFinished=false;
            return isFinished;}
    }
    
    //if(data.FoodList.size() == 0 || data.SimTime >= data.MaxSimTime) { //qilu 07/19 no need MaxSimTime
    if(data.FoodList.size() == 0) 
    isFinished = true;
    //}
    
    if(isFinished == true && data.MaxSimCounter > 1) {
        size_t newSimCounter = data.SimCounter + 1;
        size_t newMaxSimCounter = data.MaxSimCounter - 1;

        //LOG << endl << "FINISHED RUN: " << data.SimCounter << endl;

        PostExperiment();
        Reset();

        data.SimCounter    = newSimCounter;
        data.MaxSimCounter = newMaxSimCounter;
        isFinished         = false;
    }
	return isFinished;
}

REGISTER_LOOP_FUNCTIONS(iAnt_loop_functions, "iAnt_loop_functions")

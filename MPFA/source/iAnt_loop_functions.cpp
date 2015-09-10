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
    
    //GetNodeAttribute(simNode, "nestPosition_4"   , nestPosition     );
    //iAnt_nest nest4=iAnt_nest(nestPosition);
    //data.nests.push_back(nest4);
    //GetNodeAttribute(simNode, "nestPosition_5"    , nestPosition    );
    //iAnt_nest nest5=iAnt_nest(nestPosition);
    //data.nests.push_back(nest5);
    //GetNodeAttribute(simNode, "nestPosition_6"    , nestPosition    );
    //iAnt_nest nest6=iAnt_nest(nestPosition);
    //data.nests.push_back(nest6);
    //GetNodeAttribute(simNode, "nestPosition_7"    , nestPosition    );
    //iAnt_nest nest7=iAnt_nest(nestPosition);
    //data.nests.push_back(nest7);
    //GetNodeAttribute(simNode, "nestPosition_8"   , nestPosition     );
    //iAnt_nest nest8=iAnt_nest(nestPosition);
    //data.nests.push_back(nest8);
    //GetNodeAttribute(simNode, "nestPosition_9"    , nestPosition    );
    //iAnt_nest nest9=iAnt_nest(nestPosition);
    //data.nests.push_back(nest9);
    //GetNodeAttribute(simNode, "nestPosition_10"    , nestPosition    );
    //iAnt_nest nest10=iAnt_nest(nestPosition);
    //data.nests.push_back(nest10);
    //GetNodeAttribute(simNode, "nestPosition_11"    , nestPosition    );
    //iAnt_nest nest11=iAnt_nest(nestPosition);
    //data.nests.push_back(nest11);
    //GetNodeAttribute(simNode, "nestPosition_12"   , nestPosition     );
    //iAnt_nest nest12=iAnt_nest(nestPosition);
    //data.nests.push_back(nest12);
    //GetNodeAttribute(simNode, "nestPosition_13"    , nestPosition    );
    //iAnt_nest nest13=iAnt_nest(nestPosition);
    //data.nests.push_back(nest13);
    //GetNodeAttribute(simNode, "nestPosition_14"    , nestPosition    );
    //iAnt_nest nest14=iAnt_nest(nestPosition);
    //data.nests.push_back(nest14);
    //GetNodeAttribute(simNode, "nestPosition_15"    , nestPosition    );
    //iAnt_nest nest15=iAnt_nest(nestPosition);
    //data.nests.push_back(nest15);
    
    GetNodeAttribute(simNode,  "NestRadius",       data.NestRadius);
    GetNodeAttribute(simNode,  "FoodRadius",       data.FoodRadius);
    GetNodeAttribute(simNode,  "FoodDistribution", data.FoodDistribution);
    GetNodeAttribute(random,   "FoodItemCount",    data.FoodItemCount);
    GetNodeAttribute(cluster,  "NumberOfClusters", data.NumberOfClusters);
    GetNodeAttribute(cluster,  "ClusterWidthX",    data.ClusterWidthX);
    GetNodeAttribute(cluster,  "ClusterLengthY",   data.ClusterLengthY);
    GetNodeAttribute(powerLaw, "PowerRank",        data.PowerRank);
    GetNodeAttribute(powerLaw, "PowerLawCopies",        data.PowerLawCopies); //qilu 07/14/2015
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
	data.CollisionList.clear(); //qilu 08/19
    data.ForageList.clear(); //qilu 08/19
    data.last_time_in_minutes=0; //qilu 08/19
}

/*****
 * This hook function is called before iAnts call their ControlStep() function.
 *****/
void iAnt_loop_functions::PreStep() {
    data.SimTime++;
	//qilu 08/19
    data.curr_time_in_minutes = floor(floor(data.SimTime/data.TicksPerSecond)/60);
    if(data.curr_time_in_minutes - data.last_time_in_minutes==1){
		data.CollisionList.push_back(data.currNumCollision - data.lastNumCollision);
		data.lastNumCollision = data.currNumCollision;
				
		data.ForageList.push_back(data.currNumCollectedFood - data.lastNumCollectedFood);
		data.lastNumCollectedFood = data.currNumCollectedFood;
		data.last_time_in_minutes++;
    }
    // end qilu 08/19 
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
            dataOutput << "tags_collected, collisions, time_in_minutes, random_seed\n";//qilu 08/18
        }

        dataOutput << collectedFood << ", ";
        dataOutput << data.currNumCollision <<", "<< time_in_minutes << ", " << data.RandomSeed << endl;//qilu 08/19
        dataOutput.close();
		//qilu 08/19
	    ofstream forageDataOutput("ForageData.txt", ios::app);
		if(data.ForageList.size()!=0) forageDataOutput<<"Forage: "<< data.ForageList[0];
        for(size_t i=1; i< data.ForageList.size(); i++) forageDataOutput<<", "<<data.ForageList[i];
        forageDataOutput<<"\n";
        forageDataOutput.close();
        
        ofstream collisonDataOutput("CollisonData.txt", ios::app);
        if(data.CollisionList.size()!=0) collisonDataOutput<<"Collision: "<< data.CollisionList[0];
        for(size_t i=1; i< data.CollisionList.size(); i++) collisonDataOutput<<", "<<data.CollisionList[i];
        collisonDataOutput<<"\n";
        collisonDataOutput.close();
        //qilu 08/19
    }

    // output to ARGoS GUI
    if(data.SimCounter == 0) {
        LOG << "\ntags_collected, collisions, time_in_minutes, random_seed\n"; //qilu 08/18
        LOG << collectedFood << ", ";
        LOG << data.currNumCollision <<", "<<time_in_minutes << ", " << data.RandomSeed << endl; //qilu 08/19
    } else {
        LOG << collectedFood << ", ";
        LOG << data.currNumCollision <<", "<< time_in_minutes << ", " << data.RandomSeed << endl; //qilu 08/19

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

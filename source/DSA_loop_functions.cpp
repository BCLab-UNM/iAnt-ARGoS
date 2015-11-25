#include "DSA_loop_functions.h"

/*****
 * Required by ARGoS. This function initializes global variables using
 * values from the XML configuration file supplied when ARGoS is run.
 *****/
void DSA_loop_functions::Init(TConfigurationNode& node) {
    /* Temporary variables. */
    CSimulator     *simulator     = &GetSimulator();
    CPhysicsEngine *physicsEngine = &simulator->GetPhysicsEngine("default");
    CVector3        ArenaSize     = GetSpace().GetArenaSize();
    CVector2        rangeX        = CVector2(-ArenaSize.GetX()/2.0, ArenaSize.GetX()/2.0);
    CVector2        rangeY        = CVector2(-ArenaSize.GetY()/2.0, ArenaSize.GetY()/2.0);
    CDegrees        USV_InDegrees;

    /* Get each tag in the loop functions section of the XML file. */
    TConfigurationNode simNode  = GetNode(node, "simulation");
    TConfigurationNode random   = GetNode(node, "_0_FoodDistribution_Random");
    TConfigurationNode cluster  = GetNode(node, "_1_FoodDistribution_Cluster");
    TConfigurationNode powerLaw = GetNode(node, "_2_FoodDistribution_PowerLaw");

    GetNodeAttribute(simNode,  "MaxSimCounter",    MaxSimCounter);
    GetNodeAttribute(simNode,  "VariableSeed",     VariableSeed);
    GetNodeAttribute(simNode,  "OutputData",       OutputData);
    GetNodeAttribute(simNode,  "NestPosition",     NestPosition);
    GetNodeAttribute(simNode,  "NestRadius",       NestRadius);
    GetNodeAttribute(simNode,  "FoodRadius",       FoodRadius);
    GetNodeAttribute(simNode,  "FoodDistribution", FoodDistribution);
    GetNodeAttribute(random,   "FoodItemCount",    FoodItemCount);
    GetNodeAttribute(cluster,  "NumberOfClusters", NumberOfClusters);
    GetNodeAttribute(cluster,  "ClusterWidthX",    ClusterWidthX);
    GetNodeAttribute(cluster,  "ClusterLengthY",   ClusterLengthY);
    GetNodeAttribute(powerLaw, "PowerRank",        PowerRank);

    /* Convert and calculate additional values. */
    NestRadiusSquared         = (NestRadius) * (NestRadius);
    FoodRadiusSquared         = (FoodRadius + 0.04) * (FoodRadius + 0.04);

    ForageRangeX.Set(rangeX.GetX() + (2.0 * FoodRadius),
                          rangeX.GetY() - (2.0 * FoodRadius));
    ForageRangeY.Set(rangeY.GetX() + (2.0 * FoodRadius),
                          rangeY.GetY() - (2.0 * FoodRadius));

    RNG = CRandom::CreateRNG("argos");

    /* Store the iAnts in a more friendly, human-readable structure. */
    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;

    ReadFile();

    size_t STOP = 0;
    size_t robots = 0;

    for(it = footbots.begin(); it != footbots.end(); it++) 
    {
        CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
        DSA_controller& c = dynamic_cast<DSA_controller&>(footBot.GetControllableEntity().GetController());
        DSAnts.push_back(&c);
        c.SetData(&data); 
        c.SetStop(STOP);
        STOP += 10; /* adds 10 seconds extra pause for each robot */                
      
        if(robots <= N_robots)
        {
            c.GetPattern(robotPattern[robots]);
            robots++;
            //attempt to add mutiple colors for each robot
            // if(robots==2)
            // {
            //     DrawTargetRays = 2;
            // }
            // else if(robots ==3)
            // {
            //     DrawTargetRays = 3;
            // }
        }    
    }

    /* Set up the food distribution based on the XML file. */
    SetFoodDistribution();
}

/*****
 * This hook function is called before iAnts call their ControlStep() function.
 * Read the file of strings from txt file parse by new line delimiter for each
 * robot. Strings are stored by indx inside robotPattern.
 *****/
void DSA_loop_functions::ReadFile()
{
    ifstream inFile;
    /* Specify the file to open. */
    inFile.open("Sample.txt");
    string stringPattern;
    string delimiter = " ";
    int indxN_robots;
    int indxEndl;
    N_robots = 0;
    string i_circuits;
    string i_robotPattern;

    if(inFile.fail()){
        cerr << "Error in opening file.";
        exit(1);
    }
    
    if (inFile.is_open()){
        while (getline (inFile,stringPattern)){
            indxEndl = stringPattern.find(delimiter);
            N_robots++;
            i_robotPattern = stringPattern.substr(0, indxEndl);
            /* Calls to push that robot string onto the robotPattern string vector */
            StoreStringPattern(i_robotPattern);
        }  
         
        inFile.close();
    }    
}

/*****
 * This a helper function for storing each robot string pattern.
 *****/
void DSA_loop_functions::StoreStringPattern (string i_robotPath)
{
    robotPattern.push_back(i_robotPath);
}

/*****
 * This hook function is called before iAnts call their ControlStep() function.
 *****/
void DSA_loop_functions::PreStep() {
    SimTime++;
}

/*****
 * This hook function is called after iAnts call their ControlStep() function.
 *****/
void DSA_loop_functions::PostStep() {

}

/*****
 * This function is called once all food is collected or the
 * time limit imposed in the XML file has been reached.
 *****/
void DSA_loop_functions::PostExperiment() {
    size_t time_in_minutes = floor(floor(SimTime/TicksPerSecond)/60);
    size_t collectedFood = FoodItemCount - FoodList.size();

    // This variable is set in XML
    if(OutputData == 1) {
        // This file is created in the directory where you run ARGoS
        // it is always created or appended to, never overwritten, i.e. ios::app
        ofstream dataOutput("iAntSpiralTagtxt", ios::app);

        // output to file
        if(dataOutput.tellp() == 0) {
            dataOutput << "tags_collected, time_in_minutes, random_seed\n"; //, levels\n";
        }

        dataOutput << collectedFood << ", ";
        dataOutput << time_in_minutes << ", " << RandomSeed << endl; //<< ", " << levels << endl;
        dataOutput.close();



        // ofstream spiralOutput("filename", ios::app);
        // write out spiral stuff





    }

     // output to ARGoS GUI
    if(SimCounter == 0) {
        LOG << "\ntags_collected, time_in_minutes, random_seed\n"; //, levels\n";
        LOG << collectedFood << ", ";
        LOG << time_in_minutes << ", " << RandomSeed << endl; //<< ", " << levels << endl;
    } else {
        LOG << collectedFood << ", ";
        LOG << time_in_minutes << ", " << RandomSeed << endl; // <<", " << levels << endl;

        /*
        ifstream dataInput("iAntTagtxt");
        string s;
        while(getline(dataInput, s)) {
            LOG << s << endl;
        }
        dataInput.close();
        */
        
    }

    SimCounter++;
}

/*****
 * This function resets all iAnts and restarts the simulation based on initial
 * conditions set in the XML file.
 *****/
void DSA_loop_functions::Reset() {
    if(VariableSeed == 1) GetSimulator().SetRandomSeed(++RandomSeed);

    //GetSimulator().Reset();
    GetSpace().Reset();
    SimTime = 0;
    ResourceDensityDelay = 0;
    FoodList.clear();
    //PheromoneList.clear();
    //FidelityList.clear();
    TargetRayList.clear();
    SetFoodDistribution();

    size_t STOP = 0;
    size_t robots = 0;
    for(size_t i = 0; i < iAnts.size(); i++) {
        iAnts[i]->Reset();
        ReadFile();
        iAnts[i]->SetStop(STOP);
        STOP += 10;

        if(robots <= N_robots)
        {
            iAnts[i]->GetPattern(robotPattern[robots]);
            robots++;
        }    
    }
}

/*****
 * An experiment is considered finished if all food items are collected and all
 * iAnts have returned their food to the nest. ARGoS also keeps track of the
 * time limit in the XML file and will stop the experiment at that time limit.
 *****/
bool DSA_loop_functions::IsExperimentFinished() {
    bool isFinished = false;

    if(FoodList.size() == 0 || SimTime >= MaxSimTime) {
        isFinished = true;
    }

    if(isFinished == true && MaxSimCounter > 1) {
        size_t newSimCounter = SimCounter + 1;
        size_t newMaxSimCounter = MaxSimCounter - 1;

        // LOG << endl << "FINISHED RUN: " << SimCounter << endl;

        PostExperiment();
        Reset();

        SimCounter    = newSimCounter;
        MaxSimCounter = newMaxSimCounter;
        isFinished         = false;
    }

    return isFinished;
}

REGISTER_LOOP_FUNCTIONS(DSA_loop_functions, "DSA_loop_functions")
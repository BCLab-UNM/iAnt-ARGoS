#include "DSA_controller.h"

/*****
 * Initialize most basic variables and objects here. Most of the setup should
 * be done in the Init(...) function instead of here where possible.
 *****/
DSA_controller::DSA_controller() :
    NumberOfRobots(0),
    NumberOfSpirals(0),
    DSA(SEARCHING),
    RNG(NULL),
    collisionCounter(0),
    ResetReturnPosition(true),
    stopTimeStep(0)
{}

/*****
 * Initialize the controller via the XML configuration file. ARGoS typically
 * wants objects & variables initialized here instead of in the constructor(s).
 *****/
void DSA_controller::Init(TConfigurationNode& node) {

    compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
    wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");

    argos::TConfigurationNode settings = argos::GetNode(node, "settings");
    argos::GetNodeAttribute(settings, "NumberOfRobots",          NumberOfRobots);
    argos::GetNodeAttribute(settings, "NumberOfSpirals",         NumberOfSpirals);
    argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
    argos::GetNodeAttribute(settings, "SearcherGap",             SearcherGap);
    argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
    argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
    argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);

    FoodDistanceTolerance *= FoodDistanceTolerance;

    argos::CVector2 p(GetPosition());
    SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
    
    startPosition = CVector3(0.0, 0.0, 0.0);

    RNG = CRandom::CreateRNG("argos");
    generatePattern(NumberOfSpirals, NumberOfRobots);
    TrailColor = CColor(std::rand()%255, std::rand()%255, std::rand()%255, 255);
}

size_t DSA_controller::generatePattern(int N_circuits, int N_robots)
{
    string ID = GetId();
    string ID_number;

    for(size_t i = 0; i < ID.size(); i++) {
        if(ID[i] >= '0' && ID[i] <= '9') {
            ID_number += ID[i];
        }
    }

    size_t RobotNumber = stoi(ID_number);
    vector<string> paths;
    string ith_robot_path;

    for (int i_robot = 1; i_robot <= N_robots; i_robot++)
    {
        // cout << "inside for 1" << endl;
        for (int i_circuit = 0; i_circuit < N_circuits; i_circuit++)
        {
            int n_steps_north = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'N');
            for (int j = 0; j < n_steps_north; j++)
            {
                //ith_robot_path.push_back('N');
                ith_robot_path += 'N';
            }
            
            int n_steps_east = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'E');
            for (int j = 0; j < n_steps_east; j++)
            {
                //ith_robot_path.push_back('E');
                ith_robot_path += 'E';
            }

            int n_steps_south = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'S');
            for (int j = 0; j < n_steps_south; j++)
            {
                //ith_robot_path.push_back('S');
                ith_robot_path += 'S';
            }

            int n_steps_west = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'W');
            for (int j = 0; j < n_steps_west; j++)
            {
                //ith_robot_path.push_back('W');
                ith_robot_path += 'W';
            }

        }

        paths.push_back(ith_robot_path);
        ith_robot_path.clear();
    }

    //pattern = ith_robot_path;
    GetPattern(paths[RobotNumber]);

    return RobotNumber;
}

int DSA_controller::calcDistanceToTravel(int i_robot, int i_circuit, int N_robots, char direction)
{
    int i = i_robot;
    int j = i_circuit;
    int N = N_robots;
    int n_steps  = 0;

    if (direction == 'N' || direction == 'E')
    {
        if (j == 0)
        {
            n_steps = i;
            return n_steps;
        }
        else if (j == 1)
        {
            n_steps = calcDistanceToTravel(i, j-1, N, direction) + i + N;
            return n_steps;
        }
        else 
        {
            n_steps = calcDistanceToTravel(i, j-1, N, direction) + 2*N;
            return n_steps;
        }
    }

    else if (direction == 'S' || direction == 'W')
    {
        if (j == 0)
        {
            n_steps = calcDistanceToTravel(i, j , N, 'N') + i;
            return n_steps;
        }

        else if (j > 0)
        {
            n_steps = calcDistanceToTravel(i, j, N, 'N') + N;
            return n_steps;
        }

        else
        {
            cout << "Error direction" << direction << "is invalid" << endl;
        }

    }
    return 0;
}

void DSA_controller::printPath(vector<char>& path)
{
    cout << path.size() << endl;
    for(int i = 0; i<path.size(); i++)
    { 
        cout << path.at(i) << endl;
    }
}

void DSA_controller::GetPattern(string ith_Pattern)
{
    copy(ith_Pattern.begin(),ith_Pattern.end(),back_inserter(tempPattern));
    reverse(tempPattern.begin(), tempPattern.end());
}

// /*****
//  *
//  *****/
void DSA_controller::CopyPatterntoTemp() 
{
    copy(pattern.begin(),pattern.end(),back_inserter(tempPattern));
    reverse(tempPattern.begin(),tempPattern.end());/* Reverses the tempPattern */
}

/*****
 * Primary control loop for this controller object. This function will execute
 * the CPFA logic using the CPFA enumeration flag once per frame.
 *****/
void DSA_controller::ControlStep() {

    // argos::LOG << DSA << " : " << GetTarget() << std::endl;

	/* Checks if the robot found a food */
	SetHoldingFood();

    /* If it didn't continue in a sprial */
    if(IsHoldingFood() == false && DSA == SEARCHING){

        /* draws target rays every 2 seconds */
        if((SimulationTick() % (SimulationTicksPerSecond() * 1)) == 0) {
            CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.02);
            CVector3 target3d(GetTarget().GetX(), GetTarget().GetY(), 0.02);
            CRay3 targetRay(target3d, position3d);
            myTrail.push_back(targetRay);
            //LOG << myTrail.size() << endl;
            loopFunctions->TargetRayList.push_back(targetRay);
            loopFunctions->TargetRayColorList.push_back(TrailColor);
            //loopFunctions->TargetRayList.insert(loopFunctions->TargetRayList.end(), myTrail.begin(), myTrail.end());
        }

        // DSA = SEARCHING;
        GetTargets(); /* Initializes targets positions. */

    } else if(IsHoldingFood() == true && DSA == RETURN_TO_NEST) { /* Check if it is near the nest then set isHoldingFood to false */

        if((GetPosition() - loopFunctions->NestPosition).SquareLength() < loopFunctions->NestRadiusSquared) {
            // isHoldingFood = false;
            DSA = RETURN_TO_SEARCH;
        }

    } else if(IsHoldingFood() == false && DSA == RETURN_TO_SEARCH) {

        if((GetPosition() - ReturnPosition).SquareLength() < loopFunctions->NestRadiusSquared) {
            DSA = SEARCHING;
        }

    }

	Move();
}   

/*****
 * Sets target North of the robot's current target.
 *****/
void DSA_controller::SetTargetN(char x){
    CVector2 position = GetTarget();
    SetTarget(CVector2(position.GetX()+SearcherGap,position.GetY()));
}

/*****
 * Sets target South of the robot's current target.
 *****/
void DSA_controller::SetTargetS(char x){
    CVector2 position = GetTarget();
    SetTarget(CVector2(position.GetX()-SearcherGap,position.GetY()));
}

/*****
 * Sets target East of the robot's current target.
 *****/
void DSA_controller::SetTargetE(char x){
   CVector2 position = GetTarget();
   SetTarget(CVector2(position.GetX(),position.GetY()-SearcherGap));
}

/*****
 * Sets target West of the robot's current target.
 *****/
void DSA_controller::SetTargetW(char x){
    CVector2 position = GetTarget();
    SetTarget(CVector2(position.GetX(),position.GetY()+SearcherGap));
}

/*****
 * Helper function that reads vector <char> pattern
 * and sets the target's direction base on the 
 * char at the current vector index.
 *****/
 void DSA_controller::GetTargets(){

    /* Finds the last direction of the pattern. */
    char direction_last = tempPattern[tempPattern.size() - 1]; 
   
    /* If the robot hit target and the patter size >0
       then find the next direction. */
    if(TargetHit() == true && tempPattern.size() > 0) {
        tempPattern.pop_back();

        switch(direction_last)
        {
            case 'N':
                SetTargetN('N');
                break;
            case 'S':
                SetTargetS('S');
                break;
            case 'E':
                SetTargetE('E');
                break;
            case 'W':
                SetTargetW('W');
                break;
            //default:
            	//Stop();
        }

        DSA = SEARCHING;
    }
    /* If the robot is down traversing the tempPattern, then return home */
    else if(tempPattern.size() == 0) {
    	Stop();
        // DSA = RETURN_TO_NEST;
        // SetTarget(loopFunctions->NestPosition);
        // Reset();
	}
}

/*****
 * Returns a boolean based on weather the robot is with 0.01 
 * distance tolerance. Declares that the robot had reached 
 * current target.
 *****/
 bool DSA_controller::TargetHit() {
    CVector2 position = GetPosition() - GetTarget();
    bool hit = false;
     
    if(position.SquareLength() < TargetDistanceTolerance){
        hit = true;
    }
    return hit;
 }

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void DSA_controller::SetHoldingFood(){
    if(IsHoldingFood() == false) {
        vector <CVector2> newFoodList; 
        size_t i = 0;
        for (i = 0; i < loopFunctions->FoodList.size(); i++){
            if ((GetPosition()-loopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance){
                isHoldingFood = true;
                ResetReturnPosition = false;
                ReturnPosition = GetTarget();
                SetTarget(loopFunctions->NestPosition);
                //if(ResetReturnPosition == true) {
                //    ReturnPosition = GetTarget();
                //}
                DSA = RETURN_TO_NEST;
            } else {
                newFoodList.push_back(loopFunctions->FoodList[i]);
            }
        } 
        loopFunctions->FoodList = newFoodList;
    } else if(IsHoldingFood() == true) {
    	if((GetPosition()-loopFunctions->NestPosition).SquareLength() < loopFunctions->NestRadiusSquared) {
    		isHoldingFood = false;
            ResetReturnPosition = true;
    		DSA = RETURN_TO_SEARCH;
    		SetTarget(ReturnPosition);
    	}
    }
}
/*****
 * Is this iAnt_controller holding food?
 *     true  = yes
 *     false = no
 *****/
bool DSA_controller::IsHoldingFood() {
    return isHoldingFood;
}
/*****
 * After pressing the reset button in the GUI, this controller will be set to
 * default factory settings like at the start of a simulation.
 *****/
void DSA_controller::Reset() {
    collisionDelay  = 0;
    SetTarget(loopFunctions->NestPosition);
    tempPattern.clear();
    CopyPatterntoTemp();
    generatePattern(NumberOfSpirals, NumberOfRobots);
}

REGISTER_CONTROLLER(DSA_controller, "DSA_controller")
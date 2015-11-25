#include "DSA_controller.h"

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

int DSA_controller::getNumberOfSpirals()
{
    return NumberOfSpirals;
}

void DSA_controller::printPath(vector<char>& path)
{
	cout << path.size() << endl;
	for(int i = 0; i<path.size(); i++)
	{ 
		cout << path.at(i) << endl;
	}
}

/*****
 * Initialize most basic variables and objects here. Most of the setup should
 * be done in the Init(...) function instead of here where possible.
 *****/
DSA_controller::DSA_controller() :
    compass(NULL),
    motorActuator(NULL),
    proximitySensor(NULL),
    RobotForwardSpeed(0.0),
    RobotTurningSpeed(0.0),
    NumberOfRobots(0),
    NumberOfSpirals(0),
    DSA(RETURNING),
    RNG(NULL),
    loopFunctions(dynamic_cast<iAnt_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions())),
    collisionCounter(0),
    stopTimeStep(0)
{}

/*****
 * Initialize the controller via the XML configuration file. ARGoS typically
 * wants objects & variables initialized here instead of in the constructor(s).
 *****/
void DSA_controller::Init(TConfigurationNode& node) {
    /* Shorter names, please. #This_Is_Not_Java */
    typedef CCI_PositioningSensor            CCI_PS;
    typedef CCI_DifferentialSteeringActuator CCI_DSA;
    typedef CCI_FootBotProximitySensor       CCI_FBPS;

    /* Initialize the robot's actuator and sensor objects. */
    motorActuator   = GetActuator<CCI_DSA>("differential_steering");
    compass         = GetSensor<CCI_PS>   ("positioning");
    proximitySensor = GetSensor<CCI_FBPS> ("footbot_proximity");

    TConfigurationNode DSA_params = GetNode(node, "DSA_params");
    GetNodeAttribute(DSA_params, "RobotForwardSpeed", RobotForwardSpeed);
    GetNodeAttribute(DSA_params, "RobotTurningSpeed", RobotTurningSpeed);
    GetNodeAttribute(DSA_params, "AngleToleranceInDegrees", angleInDegrees);
    GetNodeAttribute(DSA_params, "NumberOfRobots", NumberOfRobots);
    GetNodeAttribute(DSA_params, "NumberOfSpirals", NumberOfSpirals);

    AngleToleranceInRadians.Set(-ToRadians(angleInDegrees),ToRadians(angleInDegrees));
    
    stepSize = 0.1; /* Assigns the robot's stepSize */
    startPosition = CVector3(0.0, 0.0, 0.0);

    RNG = CRandom::CreateRNG("argos");
    generatePattern(NumberOfSpirals, NumberOfRobots);
}


bool DSA_controller::Stop()
{
    bool stop = false;

    if(stopTimeStep > 0)
    {
        stopTimeStep--;
        stop = true;
    }

    return stop;
}

void DSA_controller::SetStop(size_t stop_time_in_seconds)
{
    stopTimeStep += stop_time_in_seconds * loopFunctions.TicksPerSecond;

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
	if(IsHoldingFood() == false && DSA == SEARCHING) {
	    /* draws target rays every 2 seconds */
	    if((loopFunctions.SimTime % (loopFunctions.TicksPerSecond)) == 0) {
	        CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.02);
        	CVector3 target3d(GetTarget().GetX(), GetTarget().GetY(), 0.02);
        	CRay3 targetRay(target3d, position3d);
        	myTrail.push_back(targetRay);
        	loopFunctions.TargetRayList.push_back(targetRay);
	        //loopFunctions.TargetRayList.insert(loopFunctions.TargetRayList.end(), myTrail.begin(), myTrail.end());
	    }
	}

    if(Stop() == true) {
        return;//motorActuator->SetLinearVelocity(0.0, 0.0);
    } else {

	    /* Checks if the robot found a food */
	    SetHoldingFood();

	    /* If it didn't continue in a sprial */
	    if(IsHoldingFood() == false) {
	       GetTargets(); /* Initializes targets positions. */

	    } else { /* Check if it is near the nest then set isHoldingFood to false */

	    	DSA = RETURNING;

	        if((GetPosition() - loopFunctions.NestPosition).SquareLength() < loopFunctions.NestRadiusSquared) {
	            isHoldingFood = false;
	        } else {
	            ApproachTheTarget(loopFunctions.NestPosition);
	        }
	    }
	}
}   

/*****
 *
 *****/
CRadians DSA_controller::GetCollisionHeading() {

    typedef const CCI_FootBotProximitySensor::TReadings PR;
    PR &proximityReadings = proximitySensor->GetReadings();
    size_t collisionsDetected = 0;
    CRadians angle;

    for(size_t i = 0; i < proximityReadings.size(); i++) {
        if(proximityReadings[i].Value == 0.0) {
            angle += proximityReadings[i].Angle;
        }
    }

    return angle;
}

/*****
 *
 *****/
bool DSA_controller::IsCollisionDetected() {
    //MODIFY COLLISION DECTECTION HERE.
   
    typedef const CCI_FootBotProximitySensor::TReadings PR;
    PR &proximityReadings = proximitySensor->GetReadings();
    size_t collisionsDetected = 0;
    CRadians angle;

    for(size_t i = 0; i < proximityReadings.size(); i++) {
        angle = proximityReadings[i].Angle;

        if((proximityReadings[i].Value > 0.0) &&
           (AngleToleranceInRadians.WithinMinBoundIncludedMaxBoundIncluded(angle))) {
            collisionsDetected++;
        }
    }

    return (collisionsDetected > 0) ? (true) : (false);

   //THIS IS CODE FROM DIFFUSION EXAMPLE IN ARGOS 
   /* Get readings from proximity sensor */
   // const CCI_FootBotProximitySensor::TReadings& tProxReads = proximitySensor->GetReadings();
   // /* Sum them together */
   // CVector2 cAccumulator;
   // size_t collisionsDetected = 0;
   // Real m_fDelta = 0.25;

   // for(size_t i = 0; i < tProxReads.size(); ++i) {
   //    cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);

   //    if(tProxReads[i].Value > 0.0) collisionsDetected++;
   // }
   
   // cAccumulator /= tProxReads.size();
   // /* If the angle of the vector is small enough and the closest obstacle
   //  * is far enough, continue going straight, otherwise curve a little
   //  */
   // CRadians cAngle = cAccumulator.Angle();

   // if(AngleToleranceInRadians.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
   //    cAccumulator.Length() < m_fDelta ) {
   //    /* Go straight */
   //    motorActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);
   // }
   // else {
   //    /* Turn, depending on the sign of the angle */
   //    if(cAngle.GetValue() > 0.0f) {
   //       motorActuator->SetLinearVelocity(RobotTurningSpeed, -RobotTurningSpeed);
   //    }
   //    else {
   //       motorActuator->SetLinearVelocity(-RobotTurningSpeed, RobotForwardSpeed);
   //    }
   // }

   //  return (collisionsDetected > 0) ? (true) : (false);
}

/*****
 * Sets target North of the robot's current target.
 *****/
void DSA_controller::SetTargetN(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX()+stepSize,position.GetY());
}

/*****
 * Sets target South of the robot's current target.
 *****/
void DSA_controller::SetTargetS(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX()-stepSize,position.GetY());
}

/*****
 * Sets target East of the robot's current target.
 *****/
void DSA_controller::SetTargetE(char x){
   CVector2 position = GetTarget();
   target = CVector2(position.GetX(),position.GetY()-stepSize);
}

/*****
 * Sets target West of the robot's current target.
 *****/
void DSA_controller::SetTargetW(char x){
    CVector2 position = GetTarget();
    target = CVector2(position.GetX(),position.GetY()+stepSize);
}

/*****
 * Controls the robot's motor to go in the correct angle relative to the target.
 *****/
void DSA_controller::ApproachTheTarget(){

    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();

    /* angle from the target to the robot's position */
    CRadians angle2  = (target - GetPosition()).Angle();

    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
    CRadians heading = (angle1 - angle2).SignedNormalize();

    //UDPDATED TO ACCOMADATE THE DIFFUSION COLLSION AVOIDANCE
    // if(IsCollisionDetected() == false) 
    // {  
        
    //     if(heading <= AngleToleranceInRadians.GetMin()) {
    //         /* turn left */
    //         motorActuator->SetLinearVelocity(-RobotTurningSpeed, 
    //                                           RobotTurningSpeed);
    //     } else if(heading >= AngleToleranceInRadians.GetMax()){
    //         /* turn right */
    //         motorActuator->SetLinearVelocity( RobotTurningSpeed, 
    //                                          -RobotTurningSpeed);
    //     } else {
    //         /* go straight */
    //         motorActuator->SetLinearVelocity(RobotForwardSpeed, 
    //                                          RobotForwardSpeed);
    //     }

    // }
    if(heading <= AngleToleranceInRadians.GetMin()) {
        /* turn left */
        motorActuator->SetLinearVelocity(-RobotTurningSpeed, 
                                          RobotTurningSpeed);
    } else if(heading >= AngleToleranceInRadians.GetMax()){
        /* turn right */
        motorActuator->SetLinearVelocity( RobotTurningSpeed, 
                                         -RobotTurningSpeed);
    } else {
        /* go straight */
        motorActuator->SetLinearVelocity(RobotForwardSpeed, 
                                         RobotForwardSpeed);
    }
}

void DSA_controller::ApproachTheTarget(CVector2 myTarget) {

	const CCI_FootBotProximitySensor::TReadings& tProxReads = proximitySensor->GetReadings();
	CVector2 cAccumulator;
	Real front(0.0);
	Real back(0.0);
	Real m = 1.0;

	for(size_t i = 0; i <= 1; ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
		front += tProxReads[i].Value;
	}

	for(size_t i = 22; i <= 23; ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
		front += tProxReads[i].Value;
	}

	for(size_t i = 2; i <= 21; ++i) {
		back += tProxReads[i].Value;
	}

 	cAccumulator /= tProxReads.size();
	CVector2 t = myTarget;

	if(front > 0.0) {
		t += cAccumulator;
		collisionCounter++;
	}

	if(front == 0 && collisionCounter > 0) collisionCounter--;

	if(collisionCounter > loopFunctions.TicksPerSecond * 2.0) {
		m *= -1.0;
	}

    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();
    /* angle from the target to the robot's position */
    CRadians angle2  = (t - GetPosition()).Angle();
    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
    CRadians heading = (angle1 - angle2).SignedNormalize();

	if(AngleToleranceInRadians.WithinMinBoundIncludedMaxBoundIncluded(heading)) {
        /* Go straight */
        motorActuator->SetLinearVelocity(m * RobotForwardSpeed, m * RobotForwardSpeed);
        //SetStop(1);
    } else {
	    if(heading > AngleToleranceInRadians.GetMax()) {
	        motorActuator->SetLinearVelocity(RobotTurningSpeed, -RobotForwardSpeed);
	    } else if(heading < AngleToleranceInRadians.GetMin()) {
	        motorActuator->SetLinearVelocity(-RobotForwardSpeed, RobotTurningSpeed);
	    }
	}

}

/*****
 *
 *****/
CVector2 DSA_controller::GetTarget() {
    return target;
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
            default:
                motorActuator->SetLinearVelocity(0.0, 0.0);        
        }

        DSA = SEARCHING;
    }
    /* If the robot is down traversing the tempPattern, then return home */
    else if(tempPattern.size() == 0) {
    	DSA = RETURNING;
        ApproachTheTarget(loopFunctions.NestPosition);
        Reset();
    /* Otherwise we continue to approach the target. */  
    } else ApproachTheTarget(target);
 }

/*****
 * Returns a boolean based on weather the robot is with 0.01 
 * distance tolerance. Declares that the robot had reached 
 * current target.
 *****/
 bool DSA_controller::TargetHit(){
    CVector2 position = GetPosition();
    bool hit = false;
     
    if((position-target).SquareLength() < 0.01){
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
    /* Is the iAnt already holding food? */
    if(IsHoldingFood() == false) {
        vector <CVector2> newFoodList; 
        size_t i = 0; 

        /* No, the iAnt isn't holding food. Check if we have found food at our
           current position and update the food list if we have. */

        for (i = 0; i < loopFunctions.FoodList.size(); i++){
            /* We found food! */
            if ((GetPosition()-loopFunctions.FoodList[i]).SquareLength() < loopFunctions.FoodRadiusSquared){
                isHoldingFood = true;
                //goingHome = true;
            }
            /* Else push the that current food onto the newFoodList. */
            else {
                /* Return this unfound-food position to the list */
                newFoodList.push_back(loopFunctions.FoodList[i]);
            }
        } 
        loopFunctions.FoodList = newFoodList;
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
    target          = loopFunctions.NestPosition;
    tempPattern.clear();
    CopyPatterntoTemp();
    generatePattern(NumberOfSpirals, NumberOfRobots);
}

/*****
 * Return the robot's 2D position on the arena.
 *****/
CVector2 DSA_controller::GetPosition() {
    /* The robot's compass sensor gives us a 3D position. */
    CVector3 position3D = compass->GetReading().Position;
    /* Return the 2D position components of the compass sensor reading. */
    return CVector2(position3D.GetX(), position3D.GetY());
}

CVector3 DSA_controller::GetStartPosition() 
{ 
    return startPosition; 
}

/*****
 * Return the angle the robot is facing relative to the arena's origin.
 *****/
CRadians DSA_controller::GetHeading() {
    /* in ARGoS, the robot's orientation is represented by a quaternion */
    const CCI_PositioningSensor::SReading& sReading = compass->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /* convert the quaternion to euler angles */
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* the angle to the z-axis represents the compass heading */
    return z_angle;
}

REGISTER_CONTROLLER(DSA_controller, "DSA_controller")
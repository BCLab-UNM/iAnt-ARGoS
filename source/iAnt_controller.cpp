#include "iAnt_controller.h"

/*****
 * Initialize most basic variables and objects here. Most of the setup should be done in the Init(...) function instead
 * of here where possible.
 *****/
iAnt_controller::iAnt_controller() :
    compass(NULL),
    wheels(NULL),
    proximitySensor(NULL),
    maxTrailSize(0),
    distanceTolerance(0.0),
    searchStepSize(0.0),
    robotForwardSpeed(0.0),
    robotRotationSpeed(0.0),
    RNG(NULL),
    loopFunctions(dynamic_cast<iAnt_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions())),
    isHoldingFood(false),
    isInformed(false),
    isUsingSiteFidelity(false),
    isGivingUpSearch(false),
    searchTime(0),
    waitTime(0),
    collisionDelay(0),
    resourceDensity(0),
    CPFA(DEPARTING)
{}

/*****
 * Initialize the controller via the XML configuration file. ARGoS typically wants objects & variables initialized here
 * instead of in the constructor(s).
 *****/
void iAnt_controller::Init(TConfigurationNode& node) {

    /* Shorter names, please. #This_Is_Not_Java */
    typedef CCI_PositioningSensor            CCI_PS;
    typedef CCI_DifferentialSteeringActuator CCI_DSA;
    typedef CCI_FootBotProximitySensor       CCI_FBPS;

    /* Initialize the robot's actuator and sensor objects. */
    wheels          = GetActuator<CCI_DSA>("differential_steering");
    compass         = GetSensor<CCI_PS>   ("positioning");
    proximitySensor = GetSensor<CCI_FBPS> ("footbot_proximity");

    /* Initialize the random number generator. */
    RNG = CRandom::CreateRNG("argos");

    /* CPFA node from the iAnt.argos XML file */
    TConfigurationNode iAnt_params = GetNode(node, "iAnt_params");

    /* Temporary variable, XML accepts input in degrees and is converted to radians for use with ARGoS. */
    CDegrees angleInDegrees;

    /* Input from XML for iAnt parameter variables for this controller. */
    GetNodeAttribute(iAnt_params, "maxTrailSize",            maxTrailSize);
    GetNodeAttribute(iAnt_params, "distanceTolerance",       distanceTolerance);
    GetNodeAttribute(iAnt_params, "searchStepSize",          searchStepSize);
    GetNodeAttribute(iAnt_params, "robotForwardSpeed",       robotForwardSpeed);
    GetNodeAttribute(iAnt_params, "robotRotationSpeed",      robotRotationSpeed);
    GetNodeAttribute(iAnt_params, "angleToleranceInDegrees", angleInDegrees);

    /* Convert the XML Degree input into Radians. */
    angleToleranceInRadians.Set(-ToRadians(angleInDegrees), ToRadians(angleInDegrees));

    /* Initialize position variables */
    CVector2 p(GetPosition());

    startPosition    = CVector3(p.GetX(), p.GetY(), 0.0);
    targetPosition   = loopFunctions.NestPosition;
    fidelityPosition = loopFunctions.NestPosition;
}

/*****
 * Primary control loop for this controller object. This function will execute the CPFA logic using the CPFA 
 * enumeration flag once per frame.
 *****/
void iAnt_controller::ControlStep() {

//     if(Wait() == true) return;

//     /* angle of the robot's direction relative to the arena's origin */
//     CRadians angle1  = GetHeading();
//     /* angle from the target to the robot's position */
//     CRadians angle2  = (targetPosition - GetPosition()).Angle();
//     /* get the collision detection angle */
//     CRadians angle3 = GetCollisionHeading();
//     /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
//     CRadians heading = (angle1 - angle2).SignedNormalize() - angle3;

//     //if(IsCollisionDetected() == true) heading = -heading;

//     UpdateTargetRayList();

//     if((GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
//         SetRandomSearchLocation();
//     } else {
//         // ApproachTheTarget();

//         if((heading < angleToleranceInRadians.GetMin()) ){ //&& (collisionDelay < loopFunctions.SimTime)) {

//             /* turn left */
//             wheels->SetLinearVelocity(-robotRotationSpeed, robotRotationSpeed);
//             /* turn right */
// //            wheels->SetLinearVelocity(robotRotationSpeed, -robotRotationSpeed);
//             //Wait(1);
//             return;

//         } else if((heading > angleToleranceInRadians.GetMax())  ){ //&& (collisionDelay < loopFunctions.SimTime)) {

//             /* turn right */
//             wheels->SetLinearVelocity(robotRotationSpeed, -robotRotationSpeed);
//             /* turn left */
// //            wheels->SetLinearVelocity(-robotRotationSpeed, robotRotationSpeed);
//             //Wait(1);
//             return;

//         }

//         //else if(IsCollisionDetected() == false) {
//             // go straight
//             wheels->SetLinearVelocity(robotForwardSpeed, robotForwardSpeed);
//             //Wait(1);            
//         //} 
//     }

    // don't run the control step if the robot is waiting
    if(Wait() == true) {
        return;
    }

    UpdateTargetRayList();

    // CPFA "state machine" switching mechanism
    switch(CPFA) {
        // depart from nest after food drop off (or at simulation start)
        case DEPARTING:
	       	Departing();
	       	break;
        // after departing(), once conditions are met, begin searching()
        case SEARCHING:
	       	Searching();
	       	break;
        // return to nest after food pick up or giving up searching()
        case RETURNING:
	       	Returning();
            break;
    }
}

/*****
 * After pressing the reset button in the GUI, this controller will be set to default factory settings like at the
 * start of a simulation.
 *****/
void iAnt_controller::Reset() {

    /* Reset all local variables. */
    isHoldingFood       = false;
    isInformed          = false;
    isUsingSiteFidelity = false;
    isGivingUpSearch    = false;
    searchTime          = 0;
    waitTime            = 0;
    collisionDelay      = 0;
    resourceDensity     = 0;
    CPFA                = DEPARTING;
    targetPosition      = loopFunctions.NestPosition;
    fidelityPosition    = loopFunctions.NestPosition;

    compass->Reset();
    wheels->Reset();
    proximitySensor->Reset();
    RNG->Reset();

    /* Clear all pheromone trail data. */
    trailToShare.clear();
    trailToFollow.clear();
}

/*****
 *
 *****/
bool iAnt_controller::IsHoldingFood() {
    return isHoldingFood;
}

/*****
 *
 *****/
bool iAnt_controller::IsInTheNest() {
    return ((GetPosition() - loopFunctions.NestPosition).SquareLength() < loopFunctions.NestRadiusSquared);
}

/*****
 * Return the robot's 2D position on the arena.
 *****/
CVector2 iAnt_controller::GetPosition() {
    /* The robot's compass sensor gives us a 3D position. */
    CVector3 position3D = compass->GetReading().Position;
    /* Return the 2D position components of the compass sensor reading. */
    return CVector2(position3D.GetX(), position3D.GetY());
}

/*****
 *
 *****/
CVector3 iAnt_controller::GetStartPosition() {
    return startPosition;
}

bool iAnt_controller::Wait() {

    bool wait = false;

    if(waitTime > 0) {
        waitTime--;
        wait = true;
    }

    return wait;
}

void iAnt_controller::Wait(size_t wait_time_in_seconds) {

    waitTime += (wait_time_in_seconds * loopFunctions.TicksPerSecond);

}

/*****
 * DEPARTING: CPFA state when the robot is leaving the nest and heading towards
 * a site fidelity waypoint, pheromone marker, or a random search location. The
 * iAnt robot will NOT pick up any food discovered during this state. Food is
 * only interacted with during the searching() state.
 ****/
void iAnt_controller::Departing() {

    if(loopFunctions.SimTime == 0) {
        SetRandomSearchLocation();
    } else {
        CVector2 distance = (GetPosition() - targetPosition);
        Real randomNumber = RNG->Uniform(CRange<Real>(0.0, 1.0));
        
        /* When not informed, continue to travel until randomly switching to the searching state. */
        if(isInformed == false && randomNumber < loopFunctions.ProbabilityOfSwitchingToSearching) {
            searchTime = 0;
            CPFA = SEARCHING;
            Real USV = loopFunctions.UninformedSearchVariation.GetValue();
            Real rand = RNG->Gaussian(USV);
            CRadians rotation(rand);
            CRadians angle1(rotation.UnsignedNormalize());     /* standard deviation */
            CRadians angle2(GetHeading().UnsignedNormalize()); /* theta t-1 */
            CRadians turn_angle(angle1 + angle2);
            CVector2 turn_vector(searchStepSize, turn_angle);

            SetTargetInBounds(turn_vector + GetPosition());
        }
        /* Are we informed? I.E. using site fidelity or pheromones. */
        else if(distance.SquareLength() < distanceTolerance) {
            searchTime = 0;
            CPFA = SEARCHING;

            if(isUsingSiteFidelity == true) {
                isUsingSiteFidelity = false;
                SetFidelityList();
            }
        }
    }

    /* Adjust motor speeds and direction based on the target position. */
    ApproachTheTarget();
}

/*****
 * SEARCHING: The searching state will vary based on CPFA variables. The gist
 * of this function is that the iAnt is searching for food to pick up. Unlike
 * the departing() function, food WILL be picked up and returned to the nest.
 *****/
void iAnt_controller::Searching() {
    /* "scan" for food only every half of a second */
    if(loopFunctions.SimTime % (loopFunctions.TicksPerSecond / 2) == 0) {
        SetHoldingFood();
    }

    /* When not carrying food, calculate movement. */
    if(IsHoldingFood() == false) {
        CVector2 distance = GetPosition() - targetPosition;
        Real     random   = RNG->Uniform(CRange<Real>(0.0, 1.0));

        /* randomly give up searching */
		if(random < loopFunctions.ProbabilityOfReturningToNest) {
            SetTargetInBounds(loopFunctions.NestPosition);
            isGivingUpSearch = true;
            CPFA = RETURNING;
        }
        /* If we reached our target search location, set a new one. The 
           new search location calculation is different based on wether
           we are currently using informed or uninformed search. */
        else if(distance.SquareLength() < distanceTolerance) {
            /* uninformed search */
            if(isInformed == false) {
                Real USCV = loopFunctions.UninformedSearchVariation.GetValue();
                Real rand = RNG->Gaussian(USCV);
                CRadians rotation(rand);
			    CRadians angle1(rotation.UnsignedNormalize());
                CRadians angle2(GetHeading().UnsignedNormalize());
			    CRadians turn_angle(angle1 + angle2);
                CVector2 turn_vector(searchStepSize, turn_angle);

                SetTargetInBounds(turn_vector + GetPosition());
			}
            /* informed search */
            else if(isInformed == true) {
                size_t   t           = searchTime++;

                Real     twoPi       = (CRadians::TWO_PI).GetValue();
                Real     pi          = (CRadians::PI).GetValue();
                Real     isd         = loopFunctions.RateOfInformedSearchDecay;

				Real     correlation = GetExponentialDecay((2.0 * twoPi) - loopFunctions.UninformedSearchVariation.GetValue(), t, isd);
                Real     rand = RNG->Gaussian(correlation + loopFunctions.UninformedSearchVariation.GetValue());

                CRadians rotation(GetBound(rand, -pi, pi));
                CRadians angle1(rotation);
                CRadians angle2(GetHeading());
                CRadians turn_angle(angle2 + angle1);
                CVector2 turn_vector(searchStepSize, turn_angle);

				SetTargetInBounds(turn_vector + GetPosition());
            }
        }
    }
    /* Food has been found, change state to RETURNING and go to the nest */
    else {
       	SetTargetInBounds(loopFunctions.NestPosition);
        CPFA = RETURNING;
    }

    /* Adjust motor speeds and direction based on the target position. */
    ApproachTheTarget();
}

/*****
 * RETURNING: Stay in this state until the robot has returned to the nest.
 * This state is triggered when a robot has found food or when it has given
 * up on searching and is returning to the nest.
 *****/
void iAnt_controller::Returning() {
    
    SetHoldingFood();
    
    CVector2 distance = GetPosition() - targetPosition;

    /* Are we there yet? (To the nest, that is.) */
   	if(IsInTheNest() == true) {
        /* Based on a Poisson CDF, the robot may or may not create a pheromone
           located at the last place it picked up food. */
        Real poissonCDF_pLayRate    = GetPoissonCDF(resourceDensity, loopFunctions.RateOfLayingPheromone);
        Real poissonCDF_sFollowRate = GetPoissonCDF(resourceDensity, loopFunctions.RateOfSiteFidelity);
        Real r1 = RNG->Uniform(CRange<Real>(0.0, 1.0));
        Real r2 = RNG->Uniform(CRange<Real>(0.0, 1.0));

		if(poissonCDF_pLayRate > r1) {
            if(isGivingUpSearch == false) {
                trailToShare.push_back(loopFunctions.NestPosition);
                Real timeInSeconds = (Real)(loopFunctions.SimTime / loopFunctions.TicksPerSecond);
                iAnt_pheromone sharedPheromone(fidelityPosition, trailToShare, timeInSeconds, loopFunctions.RateOfPheromoneDecay);
    			loopFunctions.PheromoneList.push_back(sharedPheromone);
                trailToShare.clear();
                sharedPheromone.Deactivate(); // make sure this won't get re-added later...
            } else {
                isGivingUpSearch = false;
            }
		}

        /* Determine probabilistically wether to use site fidelity, pheromone
           trails, or random search. */

        /* use site fidelity */
		if((isUsingSiteFidelity == true) && (poissonCDF_sFollowRate > r2)) {
			SetTargetInBounds(fidelityPosition);
			isInformed = true;
		}
        /* use pheromone waypoints */
        else if(SetTargetPheromone() == true) {
            SetFidelityList();
			isInformed = true;
            isUsingSiteFidelity = false;
		}
        /* use random search */
        else {
			SetRandomSearchLocation();
			isInformed = false;
            SetFidelityList();
            isUsingSiteFidelity = false;
		}

		CPFA = DEPARTING;
	}

    if(distance.SquareLength() < distanceTolerance) {

        CVector2 step(searchStepSize, GetHeading().UnsignedNormalize());

        SetTargetInBounds(GetPosition() + step);
    }

    /* Adjust motor speeds and direction based on the target position. */
    ApproachTheTarget();
}

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void iAnt_controller::SetHoldingFood() {

    /* Is the iAnt already holding food? */
    if(IsHoldingFood() == false) {

        /* No, the iAnt isn't holding food. Check if we have found food at our
           current position and update the food list if we have. */

        vector<CVector2> newFoodList;
        vector<CColor>   newFoodColoringList;
        size_t i = 0, j = 0;

        for(i = 0; i < loopFunctions.FoodList.size(); i++) {
            if((GetPosition() - loopFunctions.FoodList[i]).SquareLength() < loopFunctions.FoodRadiusSquared) {
                /* We found food! Calculate the nearby food density. */
                isHoldingFood = true;
                //SetLocalResourceDensity();
                j = i + 1;
                break;
            } else {
                /* Return this unfound-food position to the list */
                newFoodList.push_back(loopFunctions.FoodList[i]);
                newFoodColoringList.push_back(CColor::BLACK);
            }
        }

        for( ; j < loopFunctions.FoodList.size(); j++) {
            newFoodList.push_back(loopFunctions.FoodList[j]);
            newFoodColoringList.push_back(CColor::BLACK);
        }

        /* We picked up food. Update the food list minus what we picked up. */
        if(IsHoldingFood() == true) {
            loopFunctions.FoodList = newFoodList;
            SetLocalResourceDensity();
            SetTargetInBounds(loopFunctions.NestPosition);
        }
        /* We dropped off food. Clear the built-up pheromone trail. */
        else trailToShare.clear();
    }
    /* Drop off food: We are holding food and have reached the nest. */
    else if((GetPosition() - loopFunctions.NestPosition).SquareLength() < loopFunctions.NestRadiusSquared) {
        isHoldingFood = false;
    }

    /* We are carrying food and haven't reached the nest, keep building up the
       pheromone trail attached to this found food item. */
    if(IsHoldingFood() == true && loopFunctions.SimTime % loopFunctions.DrawDensityRate == 0) {
        trailToShare.push_back(GetPosition());
    }
}

/*****
 * Set the target to a random position along the edge of the arena.
 *****/
void iAnt_controller::SetRandomSearchLocation() {
    CVector2 p = GetPosition();

    Real newX = RNG->Uniform(loopFunctions.ForageRangeX), newY = RNG->Uniform(loopFunctions.ForageRangeY),
         x_max = loopFunctions.ForageRangeX.GetMax(), x_min = loopFunctions.ForageRangeX.GetMin(),
         y_max = loopFunctions.ForageRangeY.GetMax(), y_min = loopFunctions.ForageRangeY.GetMin();

    bool set_y_max = false;

    /* if I'm @ x_max side of arena, newX = opposite side */
    if((p.GetX() - x_max) * (p.GetX() - x_max) < distanceTolerance) {
        newX = x_min;
    }
    /* if I'm @ x_min side of arena, newX = opposite side */
    else if((p.GetX() - x_min) * (p.GetX() - x_min) < distanceTolerance) {
        newX = x_max;
    }
    /* middle of arena, randomly pick newX at plus or minus x-axis edge */
    else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
        newX = x_min;
    }
    else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
        newX = x_max;
    }
    /* if newX != the edge of the x-axis: force movement to y-axis edge */
    else {
        set_y_max = true;
    }

    /* if I'm @ y_max side of arena, newY = opposite side */
    if((p.GetY() - y_max) * (p.GetY() - y_max) < distanceTolerance) {
        newX = y_min;
    }
    /* if I'm @ y_max side of arena, newY = opposite side */
    else if((p.GetY() - y_min) * (p.GetY() - y_min) < distanceTolerance) {
        newX = y_max;
    } else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
        newY = y_min;
    }
    /* middle of arena, randomly pick newY at plus or minus y-axis edge */
    else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) {
        newY = y_max;
    }
    /* if set_y_max = true: guarantee newY is at plus or minus y-axis edge */
    else if(set_y_max == true) {
        if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < 0.5) newY = y_min;
        else newY = y_max;
    }

    SetTargetInBounds(CVector2(newX, newY));
}

/*****
 * If the robot has just picked up a food item, this function will be called
 * so that the food density in the local region is analyzed and saved. This
 * helps facilitate calculations for pheromone laying.
 *
 * Ideally, given that: [*] is food, and [=] is a robot
 *
 * [*] [*] [*] | The maximum resource density that should be calculated is
 * [*] [=] [*] | equal to 9, counting the food that the robot just picked up
 * [*] [*] [*] | and up to 8 of its neighbors.
 *
 * That being said, the random and non-grid nature of movement will not
 * produce the ideal result most of the time. This is especially true since
 * item detection is based on distance calculations with circles.
 *****/
void iAnt_controller::SetLocalResourceDensity() {

    CVector2 distance;
	resourceDensity = 1; // remember: the food we picked up is removed from the foodList before this function call
                         // therefore compensate here by counting that food (which we want to count)

    /* Calculate resource density based on the global food list positions. */
	for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
        distance = GetPosition() - loopFunctions.FoodList[i];

		if(distance.SquareLength() < loopFunctions.SearchRadius) {
			resourceDensity++;
            loopFunctions.FoodColoringList[i] = CColor::ORANGE;
            loopFunctions.ResourceDensityDelay = loopFunctions.SimTime + loopFunctions.TicksPerSecond * 10;
		} else {
            loopFunctions.FoodColoringList[i] = CColor::BLACK;
        }
	}

    /* Set the fidelity position to the robot's current position. */
    SetFidelityList(GetPosition());
    isUsingSiteFidelity = true;

    /* Delay for 4 seconds (simulate iAnts scannning rotation). */
    Wait(4);
}

/*****
 * Update the global site fidelity list for graphics display and add a new fidelity position.
 *****/
void iAnt_controller::SetFidelityList(CVector2 newFidelity) {

    vector<CVector2> newFidelityList;

    /* Remove this robot's old fidelity position from the fidelity list. */
    for(size_t i = 0; i < loopFunctions.FidelityList.size(); i++) {
        if((loopFunctions.FidelityList[i] - fidelityPosition).SquareLength() != 0.0)
            newFidelityList.push_back(loopFunctions.FidelityList[i]);
    }

    /* Update the global fidelity list. */
    loopFunctions.FidelityList = newFidelityList;

    /* Add the robot's new fidelity position to the global fidelity list. */
    loopFunctions.FidelityList.push_back(newFidelity);

    /* Update the local fidelity position for this robot. */
    fidelityPosition = newFidelity;
}

/*****
 * Update the global site fidelity list for graphics display and remove the old fidelity position.
 *****/
void iAnt_controller::SetFidelityList() {

    vector<CVector2> newFidelityList;

    /* Remove this robot's old fidelity position from the fidelity list. */
    for(size_t i = 0; i < loopFunctions.FidelityList.size(); i++) {
        if((loopFunctions.FidelityList[i] - fidelityPosition).SquareLength() != 0.0)
            newFidelityList.push_back(loopFunctions.FidelityList[i]);
    }

    /* Update the global fidelity list. */
    loopFunctions.FidelityList = newFidelityList;
}

/*****
 * Update the pheromone list and set the target to a pheromone position.
 * return TRUE:  pheromone was successfully targeted
 *        FALSE: pheromones don't exist or are all inactive
 *****/
bool iAnt_controller::SetTargetPheromone() {
	double maxStrength = 0.0, randomWeight = 0.0;
    bool isPheromoneSet = false;

    /* update the pheromone list and remove inactive pheromones */
    // loopFunctions.UpdatePheromoneList();

    /* default target = nest; in case we have 0 active pheromones */
    targetPosition = loopFunctions.NestPosition;

    /* Calculate a maximum strength based on active pheromone weights. */
	for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
        if(loopFunctions.PheromoneList[i].IsActive() == true) {
            maxStrength += loopFunctions.PheromoneList[i].GetWeight();
        }
    }

    /* Calculate a random weight. */
    randomWeight = RNG->Uniform(CRange<double>(0.0, maxStrength));

    /* Randomly select an active pheromone to follow. */
    for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
	    if(randomWeight < loopFunctions.PheromoneList[i].GetWeight()) {
            /* We've chosen a pheromone! */
            SetTargetInBounds(loopFunctions.PheromoneList[i].GetLocation());
            trailToFollow = loopFunctions.PheromoneList[i].GetTrail();
            isPheromoneSet = true;
            /* If we pick a pheromone, break out of this loop. */
            break;
	    }

        /* We didn't pick a pheromone! Remove its weight from randomWeight. */
        randomWeight -= loopFunctions.PheromoneList[i].GetWeight();
    }

    return isPheromoneSet;
}

/*****
 * Calculate and return the exponential decay of "value."
 *****/
Real iAnt_controller::GetExponentialDecay(Real value, Real time, Real lambda) {

    /* convert time into units of half-seconds from simulation frames */
    //time = time / (loopFunctions.TicksPerSecond / 2.0);

    //LOG << "time: " << time << endl;
    //LOG << "correlation: " << (value * exp(-lambda * time)) << endl << endl;

	return (value * exp(-lambda * time));
}

/*****
 * Provides a bound on the value by rolling over a la modulo.
 *****/
Real iAnt_controller::GetBound(Real value, Real min, Real max) {
    /* Calculate an offset. */
    Real offset = Abs(min) + Abs(max);

    /* Increment value by the offset while it's less than min. */
    while (value < min) {
        value += offset;
    }

    /* Decrement value by the offset while it's greater than max. */
    while (value > max) {
        value -= offset;
    }

    /* Return the bounded value. */
    return value;
}

/*****
 * Return the Poisson cumulative probability at a given k and lambda.
 *****/
Real iAnt_controller::GetPoissonCDF(Real k, Real lambda) {
    Real sumAccumulator       = 1.0;
    Real factorialAccumulator = 1.0;

    for (size_t i = 1; i <= floor(k); i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }

    return (exp(-lambda) * sumAccumulator);
}

/*****
 * Return the angle the robot is facing relative to the arena's origin.
 *****/
CRadians iAnt_controller::GetHeading() {
    /* in ARGoS, the robot's orientation is represented by a quaternion */
    const CCI_PositioningSensor::SReading& sReading = compass->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /* convert the quaternion to euler angles */
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* the angle to the z-axis represents the compass heading */
    return z_angle;
}

/*****
 *
 *****/
CRadians iAnt_controller::GetCollisionHeading() {

    typedef const CCI_FootBotProximitySensor::TReadings PR;
    PR &proximityReadings = proximitySensor->GetReadings();
    CRadians angle(0);

    /*
    for(size_t i = 0; i < proximityReadings.size(); i++) {
        if(proximityReadings[i].Value > 0.0) {
            angle += proximityReadings[i].Angle;
        }
    }
    */

    for(size_t i = 0; i <= 1; i++) {
        if(proximityReadings[i].Value > 0.0) {
            angle += proximityReadings[i].Angle;
        }
    }

    for(size_t i = 22; i <= 23; i++) {
        if(proximityReadings[i].Value > 0.0) {
            angle += proximityReadings[i].Angle;
        }
    }

    return angle;
}

/*****
 *
 *****/
bool iAnt_controller::IsCollisionDetected() {

    typedef const CCI_FootBotProximitySensor::TReadings PR;
	PR &proximityReadings = proximitySensor->GetReadings();
	size_t collisionsDetected = 0;
    CRadians angle;

    /*
	for(size_t i = 0; i < proximityReadings.size(); i++) {
        angle = proximityReadings[i].Angle;

		if((proximityReadings[i].Value > 0.0) &&
           (angleToleranceInRadians.WithinMinBoundIncludedMaxBoundIncluded(angle))) {
            collisionsDetected++;
		}
	}
    */

    for(size_t i = 0; i <= 1; i++) {
        angle = proximityReadings[i].Angle;

        if((proximityReadings[i].Value > 0.0)) {
            // && (angleToleranceInRadians.WithinMinBoundIncludedMaxBoundIncluded(angle))) {
            collisionsDetected++;
        }
    }

    for(size_t i = 22; i <= 23; i++) {
        angle = proximityReadings[i].Angle;

        if((proximityReadings[i].Value > 0.0)) {
            // && (angleToleranceInRadians.WithinMinBoundIncludedMaxBoundIncluded(angle))) {
            collisionsDetected++;
        }
    }

	return (collisionsDetected > 0) ? (true) : (false);
}

/*****
 * The footbot proximity sensor is composed of 24 sensors that are equally
 * spaced around the footbot.
 *
 * Index [0]  - [11] represents sensors on the left side of the robot.
 *                   Sensors are designated by positive angle values
 *                   7.5 degrees, 22.5, ... (+15) ..., and 172.5 degrees.
 * Index [12] - [23] represents sensors on the right side of the robot.
 *                   Sensors are designated by negative angle values
 *                   -7.5 degrees, -22.5, ... (-15) ..., and -172.5 degrees.
 *
 *        7.5   -7.5         | Random motion is introduced by choosing
 *  22.5 /==front===\ -22.5  | behavior based both on sensor readings
 *    . /============\ .     | and random probabilities.
 *    . |left===right| .     |
 *    . \============/ .     | This design intends to help robots "jiggle"
 * 157.5 \===back===/ -157.5 | out of situations where they would normally
 *      172.5    -172.5      | become trapped and stop moving.
 *
 *****/
/* bool iAnt_controller::DetectCollisions() {
    // Get the collision sensor readings.
	const CCI_FootBotProximitySensor::TReadings& pReadings = proximitySensor->GetReadings();
    Real left = 0.0, right = 0.0, collision = 0.0;

    // Calculate collisions on the left side of the robot.
    for(size_t i = 0; i < 12; i++) {
        left += pReadings[i].Value;
    }

    // Calculate collisions on the right side of the robot.
    for(size_t i = 12; i < 24; i++) {
        right += pReadings[i].Value;
    }

    // Calculate the total collision value for the left and right sides
    collision = left + right;

    // Randomly react to collisions based on the following probabilities.

    // FIRST: Randomly decide whether to turn based on collision data.
    if(collision > 0.0 && RNG->Uniform(CRange<Real>(0.0, 1.0)) < loopFunctions.TurnProbability) {

        if(left > right)
		    wheels->SetLinearVelocity(MaxRobotSpeed, -MaxRobotSpeed);

        if(right > left)
		    wheels->SetLinearVelocity(-MaxRobotSpeed, MaxRobotSpeed);

    }
    // SECOND: Randomly decide to ignore sensors and move forward.
    else if(collision > 0.0 && RNG->Uniform(CRange<Real>(0.0, 1.0)) < loopFunctions.PushProbability)
		wheels->SetLinearVelocity(MaxRobotSpeed, MaxRobotSpeed);
    // THIRD: Randomly decide to reverse away from (or into) a collision.
    else if(collision > 0.0 && RNG->Uniform(CRange<Real>(0.0, 1.0)) < loopFunctions.PullProbability)
		wheels->SetLinearVelocity(-MaxRobotSpeed, -MaxRobotSpeed);
    // FOURTH: Randomly decide to stop. Wait for obstacles to move (or not).
    else if(collision > 0.0 && RNG->Uniform(CRange<Real>(0.0, 1.0)) < loopFunctions.WaitProbability)
		wheels->SetLinearVelocity(0.0, 0.0);

    // Return true if we detected collisions, false otherwise.
    return (collision > 0.0) ? (true) : (false);
} */

/*****
 * If the robot is heading towards the target position set by the ControlStep()
 * function then no correction is made to the robot's motor speeds.
 *
 * Otherwise, the robot will calculate whether turn left or right such that it
 * is facing its intended target and then move forward.
 *****/
void iAnt_controller::ApproachTheTarget() {
    /* angle of the robot's direction relative to the arena's origin */
    CRadians angle1  = GetHeading();

    /* angle from the target to the robot's position */
    CRadians angle2  = (targetPosition - GetPosition()).Angle();

    /* heading = angle1 - angle2 = 0.0 when the robot is facing its target */
	CRadians heading = (angle1 - angle2).SignedNormalize();

    if(IsCollisionDetected() == true) {
	   collisionDelay = loopFunctions.SimTime + (loopFunctions.TicksPerSecond * 2);

       /* turn left */
	   wheels->SetLinearVelocity(-robotRotationSpeed, robotRotationSpeed);
    } else if((heading <= angleToleranceInRadians.GetMin()) && (collisionDelay < loopFunctions.SimTime)) {

    	/* turn left */
    	wheels->SetLinearVelocity(-robotRotationSpeed, robotRotationSpeed);

    } else if((heading >= angleToleranceInRadians.GetMax()) && (collisionDelay < loopFunctions.SimTime)) {

    	/* turn right */
    	wheels->SetLinearVelocity(robotRotationSpeed, -robotRotationSpeed);

    } else {

        // go straight
        wheels->SetLinearVelocity(robotForwardSpeed, robotForwardSpeed);
    }
}

/*****
 * The CPFA random and correlated walks (in addition to other sources) may generate new target points outside of the
 * bounds of the arena. We will use this function to adjust any targets such that they always fall within the bounds of
 * the forageRange X and Y values saved in the iAnt_loop_functions object.
 *****/
void iAnt_controller::SetTargetInBounds(CVector2 t) {
    /* Bound the X value based on the forage range. */
    if(t.GetX() > loopFunctions.ForageRangeX.GetMax())
        t = CVector2(loopFunctions.ForageRangeX.GetMax(), t.GetY());

    if(t.GetX() < loopFunctions.ForageRangeX.GetMin())
        t = CVector2(loopFunctions.ForageRangeX.GetMin(), t.GetY());

    /* Bound the Y value based on the forage range. */
    if(t.GetY() > loopFunctions.ForageRangeY.GetMax())
        t = CVector2(t.GetX(), loopFunctions.ForageRangeY.GetMax());

    if(t.GetY() < loopFunctions.ForageRangeY.GetMin())
        t = CVector2(t.GetX(), loopFunctions.ForageRangeY.GetMin());

    /* Set the robot's target to the bounded t position. */
    targetPosition = t;
}

void iAnt_controller::UpdateTargetRayList() {
    if(loopFunctions.SimTime % loopFunctions.DrawDensityRate == 0 && loopFunctions.DrawTargetRays == 1) {
        /* Get position values required to construct a new ray */
        CVector2 t(targetPosition);
        CVector2 p(GetPosition());
        CVector3 position3d(p.GetX(), p.GetY(), 0.02);
        CVector3 target3d(t.GetX(), t.GetY(), 0.02);

        /* scale the target ray to be <= searchStepSize */
        Real length = abs(t.Length() - p.Length());

        if(length > searchStepSize) {
            myTrail.clear();
        } else {
            /* add the ray to the robot's target trail */
            CRay3 targetRay(target3d, position3d);
            myTrail.push_back(targetRay);

            /* delete the oldest ray from the trail */
            if(myTrail.size() > maxTrailSize) {
                myTrail.erase(myTrail.begin());
            }

            loopFunctions.TargetRayList.insert(loopFunctions.TargetRayList.end(), myTrail.begin(), myTrail.end());
            // loopFunctions.TargetRayList.push_back(myTrail);
        }
    }
}

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")

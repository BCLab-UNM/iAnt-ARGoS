/* Include our class definition. */
#include "iAnt_controller.h"

using namespace argos;

/*******************************************************************************
* Constructor
********************************************************************************
* This constructor will initialize all variables except those with default
* constructors. Additional initialization will occur in the Init() function
* using settings and values from the ARGoS XML file.
*******************************************************************************/
iAnt_controller::iAnt_controller():
    steeringActuator(NULL),
    proximitySensor(NULL),
    compassSensor(NULL),
    loopFunctions(NULL),
    isHoldingFood(false),
    isInformed(false),
    resourceDensity(0),
    collisionDelay(0),
    simTime(0),
    searchTime(0),
    framesPerSecond(0),
    maxRobotSpeed(0.0),
    nestRadiusSquared(0.0),
    foodRadiusSquared(0.0),
    searchRadiusSquared(0.0),
    distanceTolerance(0.0),
    RNG(NULL),
    travelGiveupProbability(0.0),
    searchGiveupProbability(0.0),
    searchStepSize(0.0),
    informedSearchDecay(0.0),
    siteFidelityRate(0.0),
    pheromoneRate(0.0),
    pheromoneDecayRate(0.0),
    CPFA(INACTIVE)
{}

/*******************************************************************************
* Class Initialization Function
********************************************************************************
* This function uses an ARGoS configuration XML file to set up variables and
* settings. This function is inherited from the CCI_Controller class. It is
* highly recommended that all setup for simulation variables be focused here.
*
* @param node ARGoS XML configuration node
*******************************************************************************/
void iAnt_controller::Init(TConfigurationNode& node) {
    /* CPFA node from the iAnt.argos XML file */
    TConfigurationNode cpfa = GetNode(node, "CPFA");

    /* Input from XML is in degrees, but it will be converted to radians. */
    CDegrees USC_InDegrees, AT_InDegrees;

    /* Useful shorthand to keep lines of code <= 80 characters. #OCD */
    typedef CCI_DifferentialSteeringActuator CCI_DSA;
    typedef CCI_FootBotProximitySensor       CCI_FBPS;
    typedef CCI_PositioningSensor            CCI_PS;

    /* Initialize ARGoS built-in sensors and actuators. */
    steeringActuator = GetActuator<CCI_DSA>("differential_steering");
    proximitySensor  = GetSensor<CCI_FBPS>("footbot_proximity");
    compassSensor    = GetSensor<CCI_PS>("positioning");

    /* Input from XML to CPFA variables for this controller object. */
    GetNodeAttribute(cpfa, "searchRadius", searchRadiusSquared);
    GetNodeAttribute(cpfa, "uninformedSearchCorrelation", USC_InDegrees);
    GetNodeAttribute(cpfa, "distanceTolerance", distanceTolerance);
    GetNodeAttribute(cpfa, "angleTolerance", AT_InDegrees);
    GetNodeAttribute(cpfa, "searchGiveupProbability", searchGiveupProbability);
    GetNodeAttribute(cpfa, "travelGiveupProbability", travelGiveupProbability);
    GetNodeAttribute(cpfa, "siteFidelityRate", siteFidelityRate);
    GetNodeAttribute(cpfa, "informedSearchDecay", informedSearchDecay);
    GetNodeAttribute(cpfa, "pheromoneRate", pheromoneRate);
    GetNodeAttribute(cpfa, "pheromoneDecayRate", pheromoneDecayRate);
    GetNodeAttribute(cpfa, "searchStepSize", searchStepSize);
    GetNodeAttribute(cpfa, "maxRobotSpeed", maxRobotSpeed);

    /******************************************************************/
    /* Further processing and converting required for some variables. */
    /******************************************************************/

    /* Change searchRadius into (searchRadius * searchRadius). It's useful for
       distance formula calculations where taking square roots is skipped. */
    searchRadiusSquared *= searchRadiusSquared;

    /* The input for uninformed search correlation is in Degrees. We convert
       that value into Radians for use within ARGoS. */
    uninformedSearchCorrelation = ToRadians(USC_InDegrees);

    /* For footbot robots, the front of the robot is set to 0 Degrees where
       negative values are to the robot's relative right and positive values
       are to the robot's relative left. This tolerance range is used for
       collision detection calculations. */
    angleTolerance.Set(-ToRadians(AT_InDegrees), ToRadians(AT_InDegrees));

    /* Set the CPFA pheromone decay rates for this robot's pheromones. */
    targetPheromone.SetDecay(pheromoneDecayRate);
    sharedPheromone.SetDecay(pheromoneDecayRate);

    /* Initialize this controller's random number generator. Used in primarily
       for navigation calculations in this context. */
    RNG = CRandom::CreateRNG("argos");
}

/*******************************************************************************
* Calculate the distance to the edge of the nest using the X and Y coordinates
* of the robot's current position and the Pythagorean Theorem. The robot's
* position will be centered in the middle of the robot. Therefore, this check
* will not return TRUE until the robot is at least halfway into the nest.
*
* - Subtract the robot's nest position from its current position to ensure the
*   calculation occurs as if the nest was always at the origin (0,0).
*
* - Pythagorean Theorem: a^2 + b^2 = c^2
*   a   = position.GetX() - nestPosition.GetX()
*   b   = position.GetY() - nestPosition.GetY()
*   c^2 = a^2 + b^2 = (position - nestPosition).SquareLength()
*
* - Use the squared value of the radius and c^2 for the comparison calculation
*   instead of calculating square roots for improved efficiency.
*
* - If c^2 < nestRadiusSquared, the robot is inside of the nest.
*
* @return TRUE:  The robot is in the nest.
*         FALSE: The robot is not in the nest.
*******************************************************************************/
bool iAnt_controller::IsInTheNest() {
  return ((GetPosition() - nestPosition).SquareLength() < nestRadiusSquared);
}

/*******************************************************************************
* Using the Pythagorean Theorem, the robot's current position, and a list of
* all food item positions currently available, this function will determine if
* a robot has encountered a food item. See comments for IsInTheNest() for
* details on how the checked distances are calculated.
*
* @return TRUE:  The robot is finding food.
*         FALSE: The robot is not finding food.
*******************************************************************************/
bool iAnt_controller::IsFindingFood() {
    Real squareDistance = 0.0;

    for(size_t i = 0; i < foodPositions.size(); i++) {
        /* Calculate our square distance to foodPositions[i]. */
        squareDistance = (GetPosition() - foodPositions[i]).SquareLength();

        /* Bingo! We have discovered a food item! */
        if(squareDistance < foodRadiusSquared) return true;
    }

    /* Shucks... We haven't discovered a food item. */
	return false;
}

/*******************************************************************************
* By design, iAnt robots cannot carry more then 1 piece of food at a time. This
* function is used to check the status of the isHoldingFood boolean flag. This
* flag is used to ensure that an iAnt is not attempting to pick up another food
* item while it is already carrying one to the nest zone.
*
* @return TRUE:  The robot is holding food.
*         FALSE: The robot is not holding food.
*******************************************************************************/
bool iAnt_controller::IsHoldingFood() {
	return isHoldingFood;
}

/*******************************************************************************
* Return the status of the isInformed flag. This flag is set when the robot is
* using informed search in the CPFA.
*
* @return TRUE:  The robot is using informed search.
*         FALSE: The robot is not using informed search.
*******************************************************************************/
bool iAnt_controller::IsInformed() {
    return isInformed;
}

/*******************************************************************************
* @return A string representation of the current state of the Central Place
*         Foraging Algorithm. This is used by the iAnt_loop_functions class
*         to ensure that food is only picked up during the SEARCHING state.
*******************************************************************************/
string iAnt_controller::Get_CPFA_ID() {
    switch(CPFA) {
	    case INACTIVE:
	    	return "INACTIVE";
	    	break;
	    case DEPARTING:
	    	return "DEPARTING";
	    	break;
	    case SEARCHING:
	    	return "SEARCHING";
	    	break;
	    case RETURNING:
	    	return "RETURNING";
            break;
        case SHUTDOWN:
	    	return "SHUTDOWN";
    }

    return "INVALID";
}

/*******************************************************************************
* If IsFindingFood() = true AND IsHoldingFood() = false, then this function
* should be called to indicate that the iAnt is picking up a food item.
*******************************************************************************/
void iAnt_controller::PickupFood() {
	isHoldingFood = true;
    SetLocalResourceDensity();
}

/*******************************************************************************
* If IsInTheNest() = true AND IsHoldingFood() = true, then this function
* should be called to indicate that the iAnt is dropping off a food item.
*******************************************************************************/
void iAnt_controller::DropOffFood() {
	isHoldingFood = false;
}

/*******************************************************************************
* Update the isInformed flag for this robot. Indicates the transition from
* informed search to non-informed or random search.
*******************************************************************************/
void iAnt_controller::SetInformed(bool i) {
    isInformed = i;
}

/*******************************************************************************
* A pointer to the iAnt_loop_functions object is shared with each robot. This
* is currently a "necessary evil" in order to get properly updated information
* that the iAnt_qt_user_functions can use to draw food items, pheromones, and
* site fidelity waypoints on the arena during visualization.
*******************************************************************************/
void iAnt_controller::SetLoopFunctions(iAnt_loop_functions *lf) {
    loopFunctions = lf;
}

/*******************************************************************************
* Update the list of all food positions on the map. This list should decrease
* in size as robots find food during the simulation and should be called by
* the iAnt_loop_functions class.
*******************************************************************************/
void iAnt_controller::SetFoodPositions(vector<CVector2> fp) {
	foodPositions = fp;
}

/*******************************************************************************
* Update the list of all pheromone positions on the map. This list should
* variably change in size from [0, max number of food items] depending upon
* the number of robots and pheromone laying rate. This should be called by
* the iAnt_loop_functions class. The robot's don't work with this directly,
* but this is required in order to properly draw pheromones on the arena.
*******************************************************************************/
void iAnt_controller::SetPheromonePositions(vector<CVector2> pp) {
    pheromonePositions = pp;
}

/*******************************************************************************
* Update the list of all site fidelity positions on the map. This list should
* range in size from [0, max number of robots]. This should be called by the
* iAnt_loop_functions class. The robot's don't work with this directly, but it
* is required in order to properly draw fidelity waypoints on the arena.
*******************************************************************************/
void iAnt_controller::SetFidelityPositions(vector<CVector2> fp) {
    fidelityPositions = fp;
}

/*******************************************************************************
* Update the simulation time, which is kept track of in frames. The number of
* frames per second is defined in the XML file. The iAnt_loop_functions class
* uses this function for each robot to keep sim time synchronized.
*******************************************************************************/
void iAnt_controller::SetTime(size_t t) {
	simTime = t;
}

/*******************************************************************************
*******************************************************************************/
void iAnt_controller::SetFramesPerSecond(size_t fps) {
    framesPerSecond = fps;
}

/*******************************************************************************
* Return the Cartesian X,Y coordinates of the robot using the compassSensor.
*******************************************************************************/
CVector2 iAnt_controller::GetPosition() {
    /* 3D Vector Position: (x-position, y-position, z-position(height)). */
    CVector3 position3D = compassSensor->GetReading().Position;

    /* Return only the 2D (X,Y) coordinates. */
    return CVector2(position3D.GetX(), position3D.GetY());
}

/*******************************************************************************
* Return the fidelity position of this robot. Essentially, the fidelity
* position represents the last (X,Y) coordinate where a robot found food.
*******************************************************************************/
CVector2 iAnt_controller::GetFidelityPosition() {
    return fidelityPosition;
}

/*******************************************************************************
* Set the nest position. Most generally, this will be (0.0, 0.0), but the nest
* position can be adjusted to any position in the XML settings file.
*******************************************************************************/
void iAnt_controller::SetNestPosition(CVector2 np) {
    nestPosition = np;
}

/*******************************************************************************
* Set the squared value of the nest radius. This value is used in distance
* formulas in order to calculate proximity to the nest position. We use square
* distances to avoid taking square roots for efficiency.
*******************************************************************************/
void iAnt_controller::SetNestRadiusSquared(Real r) {
    nestRadiusSquared = r;
}

/*******************************************************************************
* Set the squared value of a food item's radius. This value is used in distance
* formulas in order to calculate proximity to a food item on the field. We use
* square distances to avoid taking square roots for efficiency.
*******************************************************************************/
void iAnt_controller::SetFoodRadiusSquared(Real rs) {
    foodRadiusSquared = rs;
}

/*******************************************************************************
* The loop functions class will set the target pheromone of this robot here.
* There is an important distinction between a controller's shared and target
* pheromones. The target pheromone is assigned to the robot by the
* iAnt_loop_functions class, while the shared pheromone is uploaded from all
* robots into the loop_functions class to form a pool of candidate pheromones
* used in the CPFA when calculating new search targets and target pheromones.
*******************************************************************************/
void iAnt_controller::SetTargetPheromone(iAnt_pheromone tp) {
	targetPheromone.Set(tp);
}

/*******************************************************************************
* The forage boundary should be set by the loop functions class and should
* represent all of the white area in the arena in the visualization. That is,
* the forage range is set to match the dimensions of the arena.
*******************************************************************************/
void iAnt_controller::SetForageRange(CRange<Real> x, CRange<Real> y) {
    forageRangeX = x;
    forageRangeY = y;
}

/*******************************************************************************
* Return the nest position.
*******************************************************************************/
CVector2 iAnt_controller::GetNestPosition() {
    return nestPosition;
}

/*******************************************************************************
* Return the nest radius.
*******************************************************************************/
Real iAnt_controller::GetNestRadius() {
    return sqrt(nestRadiusSquared);
}

/*******************************************************************************
* Return the food radius.
*******************************************************************************/
Real iAnt_controller::GetFoodRadius() {
    return sqrt(foodRadiusSquared);
}

/*******************************************************************************
* Return the list of all available food positions.
*******************************************************************************/
vector<CVector2> iAnt_controller::GetFoodPositions() {
    return foodPositions;
}

/*******************************************************************************
* Return the list of all available pheromone positions.
*******************************************************************************/
vector<CVector2> iAnt_controller::GetPheromonePositions() {
    return pheromonePositions;
}

/*******************************************************************************
* Return the list of all available site fidelity positions.
*******************************************************************************/
vector<CVector2> iAnt_controller::GetFidelityPositions() {
    return fidelityPositions;
}

/*******************************************************************************
* Return this robot's shared pheromone. The loop functions class will gather
* pheromones from each robot using this function according to the CPFA.
*******************************************************************************/
iAnt_pheromone iAnt_controller::GetSharedPheromone() {
	return sharedPheromone;
}

/*******************************************************************************
* Return the pointer to this robot's associated iAnt_loop_functions class. The
* primary use of this function will be within the iAnt_qt_user_functions in
* order to get updated values of fidelityPositions, foodPositions, and also
* pheromonePositions.
*******************************************************************************/
iAnt_loop_functions* iAnt_controller::GetLoopFunctions() {
    return loopFunctions;
}

/*******************************************************************************
* iAnt_controller Control Step Function
********************************************************************************
* This function is the primary integration between this class and the ARGoS
* simulator. ARGoS will call this function once per frame or tick for each
* currently active robot using this controller. The control scheme for this
* controller is modeled after the Central Place Foraging Algorithm (CPFA).
*******************************************************************************/
void iAnt_controller::ControlStep() {
    /* Run the CPFA based on the CPFA enumeration flag. */
    switch(CPFA) {
	    case INACTIVE:
	    	inactive();
	    	break;
	    case DEPARTING:
	    	departing();
	    	break;
	    case SEARCHING:
	    	searching();
	    	break;
	    case RETURNING:
	    	returning();
            break;
        case SHUTDOWN:
            shutdown();
    }

    /* If all food is collected, return to the nest and shutdown. */
    if(foodPositions.size() == 0 && !IsHoldingFood()) {
        CPFA = SHUTDOWN;
    }
}

/*******************************************************************************
* iAnt_controller Reset Function
********************************************************************************
* This function is called when the simulation user presses the reset button in
* the ARGoS simulator GUI. Most variables in this controller are automatically
* set or only need to be set once except for the ones in this reset list which
* are reset to default initialized values.
*******************************************************************************/
void iAnt_controller::Reset() {
    /* Reset the random number generator. */
    RNG = CRandom::CreateRNG("argos");

    /* Reset all sensors and actuators. */
    steeringActuator->Reset();
    proximitySensor->Reset();
    compassSensor->Reset();

    /* Reset targeting and pheromone positions to the nest area. We do this
       because the nest will always be drawn on top, hiding the graphics for
       these points until they are correctly reset during the next sim run. */
    targetPosition   = nestPosition;
    fidelityPosition = nestPosition;
    targetPheromone.Reset(nestPosition, 0, 0.0);
    sharedPheromone.Reset(nestPosition, 0, 0.0);

    /* Reset all timing and resource count variables. Fantastically frustrating
       bugs occur when these values become corrupted... */
    resourceDensity = 0;
    collisionDelay  = 0;
    simTime         = 0;
    searchTime      = 0;

	/* Restart the simulation with the CPFA in the INACTIVE state. */
	CPFA = INACTIVE;

	/* Reset boolean flags for this controller. */
	DropOffFood();
    SetInformed(false);
}

/*******************************************************************************
* INACTIVE: The default/starting state for the CPFA. All robots will begin here
* and immediately set a random search location to begin foraging for food.
*******************************************************************************/
void iAnt_controller::inactive() {
	SetRandomSearchLocation();
	CPFA = DEPARTING;
}

/*******************************************************************************
* DEPARTING: CPFA state when the robot is leaving the nest and heading towards
* a site fidelity waypoint, pheromone marker, or a random search location. The
* iAnt robot will NOT pick up any food discovered during this state. Food is
* only interacted with during the searching() state.
*******************************************************************************/
void iAnt_controller::departing() {
    CVector2 distance = (GetPosition() - targetPosition);
    int informedFlag = (IsInformed()) ? (1) : (0);
    
    /* Are we informed? I.E. using site fidelity or pheromones. */
    switch(informedFlag) {
        /* When informed, proceed to the target location. */
        case (true): {
            if(distance.SquareLength() < distanceTolerance) {
                searchTime = 0;
                CPFA = SEARCHING;
                SetInformed(false);
            }
            break;
        }
        /* When not informed, continue to travel until randomly switching
           to the searching state. */
        case (false): {
            Real randomNumber = RNG->Uniform(CRange<Real>(0.0, 1.0));
    		if(randomNumber < travelGiveupProbability) {
    			searchTime = 0;
    			CPFA = SEARCHING;
    		}
            /* Random search locations are "against the wall" of the arena.
               if we reach our destination "bounce" off the wall by setting
               a new random search location. This code handles a special case
               when travelGiveUpProbability = 0.0 */
            else if(distance.SquareLength() < distanceTolerance) {
                SetRandomSearchLocation();
            }
            break;
        }
    }

    /* Adjust motor speeds and direction based on targetPosition. */
	SetRobotMotors();
}

/*******************************************************************************
*******************************************************************************/
void iAnt_controller::searching() {
    int isHoldingFoodFlag = (IsHoldingFood()) ? (1) : (0);
    
    /* Are we carrying a food item? */
    switch(isHoldingFoodFlag) {
        /* When not carrying food, calculate movement. */
        case (false): {
            CVector2 distance = GetPosition() - targetPosition;
            Real     random   = RNG->Uniform(CRange<Real>(0.0, 1.0));

            /* Randomly give up searching based on searchGiveUpProbability */
		    if(random < searchGiveupProbability) {
		    	//SetTargetPosition(nestPosition);
		    	CPFA = RETURNING;
		    }
            /* If we reached our target search location, set a new one. The 
               new search location calculation is different based on wether
               we are currently using informed or uninformed search. */
            else if(distance.SquareLength() < distanceTolerance) {
                /* uninformed search */
			    if(IsInformed() == false) {
                    Real USCV = uninformedSearchCorrelation.GetValue();
                    Real rand = RNG->Gaussian(USCV);
                    CRadians rotation(rand);
			        CRadians angle1(rotation.UnsignedNormalize());
                    CRadians angle2(GetRobotHeading().UnsignedNormalize());
			        CRadians t_angle(angle1 + angle2);
                    CVector2 newTarget = CVector2(searchStepSize, t_angle) +
                                         GetPosition();

				    SetTargetPosition(newTarget);
			    }
                /* informed search */
                else if(IsInformed() == true) {
                    size_t   t           = searchTime++;
                    Real     twoPi       = (CRadians::TWO_PI).GetValue();
                    Real     pi          = (CRadians::PI).GetValue();
                    Real     isd         = informedSearchDecay;
				    Real     correlation = GetExponentialDecay(twoPi, t, isd);
				    CRadians rotation(GetBound(correlation, -pi, pi));
				    CRadians angle1(rotation.UnsignedNormalize());
				    CRadians angle2(GetRobotHeading().UnsignedNormalize());
				    CRadians t_angle(angle1 + angle2);
                    CVector2 newTarget = CVector2(searchStepSize, t_angle) +
                                         GetPosition();

				    SetTargetPosition(newTarget);
			    }
		    }

            break;
        }
        /* Food has been found, change state to RETURNING and go to the nest */
        case (true): {
            /* To turn on monitoring (i.e., food collection w/o nest return)
               replace the following code with:

               // SetTargetPosition(nestPosition); comment this out
               CPFA = RETURNING

               (additional changes need to be made to the returning() function)
            */

       	    SetTargetPosition(nestPosition);
		    CPFA = RETURNING;
            break;
        }
    }

    /* Adjust motor speeds and direction based on targetPosition. */
	SetRobotMotors();
}

/*******************************************************************************
*******************************************************************************/
void iAnt_controller::returning() {
    CVector2 distance = GetPosition() - targetPosition;

    /* Stay in this state until the robot has returned to the nest. */

    /* To turn on food monitoring (i.e. food collection w/o nest return)
       changes need to be made to the searching() function. Also, you need
       to comment out the if conditional for the code below.

       // if(distance.SquareLength() < distanceTolerance) {
    */
   	if(distance.SquareLength() < distanceTolerance) {
        /* Based on a Poisson CDF, the robot may or may not create a pheromone
           located at the last place it picked up food. */
        Real poissonCDF   = GetPoissonCDF(resourceDensity, pheromoneRate);
        Real randomNumber = RNG->Uniform(CRange<Real>(0.0, 1.0));

		if(poissonCDF > randomNumber) {
            iAnt_pheromone p(fidelityPosition, simTime, pheromoneDecayRate);
			sharedPheromone.Set(p);
		}

        /* Determine probabilistically wether to use site fidelity, pheromone
           trails, or random search. */
        poissonCDF   = GetPoissonCDF(resourceDensity, siteFidelityRate);
        randomNumber = RNG->Uniform(CRange<Real>(0.0, 1.0));

        /* use site fidelity */
		if(poissonCDF > randomNumber) {
			SetTargetPosition(fidelityPosition);
			SetInformed(true);
            // LOG << "site fidelity\n";
		}
        /* use pheromone waypoints */
        else if(targetPheromone.IsActive() == true) {
			SetTargetPosition(targetPheromone.Location());
			SetInformed(true);
            // LOG << "pheromones\n";
		}
        /* use random search */
        else {
			SetRandomSearchLocation();
			SetInformed(false);
            // LOG << "randomSearch\n";
		}

		CPFA = DEPARTING;
	}

    /* Adjust motor speeds and direction based on targetPosition. */
    SetRobotMotors();
}

/*******************************************************************************
*******************************************************************************/
void iAnt_controller::shutdown() {
	SetTargetPosition(nestPosition);

    if((GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
        steeringActuator->SetLinearVelocity(0.0, 0.0);
    } else {
        SetRobotMotors();
    }
}

/*******************************************************************************
*******************************************************************************/
void iAnt_controller::SetLocalResourceDensity()
{
    CVector2 distance;
	resourceDensity = 0;

	for(size_t i = 0; i < foodPositions.size(); i++) {
        distance = GetPosition() - foodPositions[i];

		if(distance.SquareLength() < searchRadiusSquared) {
			resourceDensity++;
		}
	}

    // LOG << resourceDensity << '\n';

    fidelityPosition = GetPosition();
}

CVector2 iAnt_controller::SetPositionInBounds(CVector2 p) {
	if(p.GetX() > forageRangeX.GetMax()) p.SetX(forageRangeX.GetMax());
	if(p.GetX() < forageRangeX.GetMin()) p.SetX(forageRangeX.GetMin());
	if(p.GetY() > forageRangeY.GetMax()) p.SetY(forageRangeY.GetMax());
	if(p.GetY() < forageRangeY.GetMin()) p.SetY(forageRangeY.GetMin());

	return p;
}

// set target to a random location
void iAnt_controller::SetRandomSearchLocation() {
    Real newX = RNG->Uniform(forageRangeX),
         newY = RNG->Uniform(forageRangeY),
         randomNumber = RNG->Uniform(CRange<Real>(0.0,4.0));

    if(randomNumber >= 0.0 && randomNumber < 1.0) {
        newX = forageRangeX.GetMax() * 2.0;
    } else if(randomNumber >= 1.0 && randomNumber < 2.0) {
        newX = forageRangeX.GetMin() * 2.0;
    } else if(randomNumber >= 2.0 && randomNumber < 3.0) {
        newY = forageRangeY.GetMax() * 2.0;
    } else if(randomNumber >= 3.0 && randomNumber < 4.0) {
        newY = forageRangeY.GetMin() * 2.0;
    }

	// randomly set the target somewhere outside the arena
	targetPosition.SetX(newX);
	targetPosition.SetY(newY);

    // reel the point in, so that we've randomly selected a point
    // on the edge of the arena
    targetPosition = SetPositionInBounds(targetPosition);
}

bool iAnt_controller::IsCollisionDetected() {
    typedef const CCI_FootBotProximitySensor::TReadings PR;
	PR &proximityReadings = proximitySensor->GetReadings();
	size_t collisionsDetected = 0;
    CRadians angle;

	for(size_t i = 0; i < proximityReadings.size(); i++) {
        angle = proximityReadings[i].Angle;

		if((proximityReadings[i].Value > 0.0) &&
           (angleTolerance.WithinMinBoundIncludedMaxBoundIncluded(angle))) {
            collisionsDetected++;
		}
	}

	return (collisionsDetected > 0) ? (true) : (false);
}

/*
 * ADD to this heading to turn LEFT
 * SUBTRACT to this heading to turn RIGHT
 *
 * this reading will give a value:
 *
 * +     0 degrees [north],
 * -    90 degrees [east],
 * +/- 180 degrees [south],
 * +    90 degrees [west]
 */
CRadians iAnt_controller::GetRobotHeading() {
    typedef const CCI_PositioningSensor::SReading PS;
    PS &sReading = compassSensor->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /* Convert quaternion to euler. */
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* Angle to z-axis represents compass heading */
    return z_angle;
}

void iAnt_controller::SetRobotMotors() {
    CRadians angle1  = GetRobotHeading();
    CRadians angle2  = (targetPosition - GetPosition()).Angle();
	CRadians heading = (angle1 - angle2).SignedNormalize();

	if(IsCollisionDetected() == true) {
		collisionDelay = simTime + (framesPerSecond * 2);

		/* turn left */
		steeringActuator->SetLinearVelocity(-maxRobotSpeed, maxRobotSpeed);

	} else if((heading <= angleTolerance.GetMin()) &&
              (collisionDelay < simTime)) {

		/* turn left */
		steeringActuator->SetLinearVelocity(-maxRobotSpeed, maxRobotSpeed);

	} else if((heading >= angleTolerance.GetMax()) &&
              (collisionDelay < simTime)) {

		/* turn right */
		steeringActuator->SetLinearVelocity(maxRobotSpeed, -maxRobotSpeed);

	} else {

		/* go straight */
		steeringActuator->SetLinearVelocity(maxRobotSpeed, maxRobotSpeed);
	}
}

void iAnt_controller::SetTargetPosition(CVector2 p) {
    targetPosition = SetPositionInBounds(p);
}

// exponential decay
Real iAnt_controller::GetExponentialDecay(Real q, Real time, Real lambda) {
	return (q * exp(-lambda * time));
}

//Provides bound on value by rolling over a la modulo
Real iAnt_controller::GetBound(Real x, Real min, Real max) {
    Real offset = Abs(min) + Abs(max);
    while (x < min) {
        x += offset;
    }
    while (x > max) {
        x -= offset;
    }
    return x;
}

// Returns Poisson cumulative probability at a given k and lambda
Real iAnt_controller::GetPoissonCDF(Real k, Real lambda) {
    Real sumAccumulator       = 1.0;
    Real factorialAccumulator = 1.0;

    for (size_t i = 1; i <= floor(k); i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }

    return (exp(-lambda) * sumAccumulator);
}

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")

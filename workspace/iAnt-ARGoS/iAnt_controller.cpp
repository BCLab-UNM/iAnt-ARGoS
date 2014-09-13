#include "iAnt_controller.h"

/* inline functions and constants */
/* TODO verify necessity of these constants */

/*
static const CVector2 NullVector = CVector2(-1, -1);
static const double m_fWheelVelocity = 16; // this is maxSpeed
static const double m_fWheelVelocityAligning = 5;
*/

/*
//Provides bound on value by rolling over a la modulo
static inline double bound(double x, double min, double max) {
    double offset = Abs(min) + Abs(max);
    while (x < min) {
        x += offset;
    }
    while (x > max) {
        x -= offset;
    }
    return x;
}

// Returns Poisson cumulative probability at a given k and lambda
static inline float poissonCDF(float k, float lambda) {
    float sumAccumulator = 1;
    float factorialAccumulator = 1;

    for (int i = 1; i <= floor(k); i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }

    return (exp(-lambda) * sumAccumulator);
}
*/

// constructor, see iAnt_controller::Init(TConfigurationNode& node);
iAnt_controller::iAnt_controller() :
	steeringActuator(NULL),
	proximitySensor(NULL),
	groundSensor(NULL),
	lightSensor(NULL),
	RNG(NULL),
	holdingFood(false),
	siteFidelity(false),
	resourceDensity(0),
	searchRadiusSquared(0.0),
	distanceTolerance(0.0),
	travelProbability(0.0),
	searchProbability(0.0),
	searchStepSize(0.0),
	maxSpeed(0.0),
	pheromoneDecayRate(0.0),
	simTime(0),
	CPFA(TRAVEL_TO_NEST)
{}

// destructor
iAnt_controller::~iAnt_controller() {
	// not in use
}

/* iAnt_controller Class Initialization Function
 *
 * Set the iAnt_controller class variables from data in a *.argos XML file. This function is
 * inherited from the CCI_Controller class.
 *
 * @param     node     ARGoS XML configuration node
 */
void iAnt_controller::Init(TConfigurationNode& node) {
	// Initialize ARGoS sensors and actuators from these categories in the *.argos XML file.
	steeringActuator = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor  = GetSensor<CCI_FootBotProximitySensor>        ("footbot_proximity"    );
	groundSensor     = GetSensor<CCI_FootBotMotorGroundSensor>      ("footbot_motor_ground" );
	lightSensor      = GetSensor<CCI_FootBotLightSensor>            ("footbot_light"        );

	GetNodeAttribute(GetNode(node, "navigation"), "searchRadius"       , searchRadiusSquared);
	GetNodeAttribute(GetNode(node, "navigation"), "arenaSize"          , arenaSize);
	GetNodeAttribute(GetNode(node, "navigation"), "nestPosition"       , nestPosition);
	GetNodeAttribute(GetNode(node, "navigation"), "distanceTolerance"  , distanceTolerance);
	GetNodeAttribute(GetNode(node, "navigation"), "searchProbability"  , searchProbability);
	GetNodeAttribute(GetNode(node, "navigation"), "searchStepSize"     , searchStepSize);
	GetNodeAttribute(GetNode(node, "navigation"), "maxSpeed"           , maxSpeed);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "pheromoneDecayRate" , pheromoneDecayRate);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "travelProbability"  , travelProbability);

	targetPheromone.SetDecay(pheromoneDecayRate);
	sharedPheromone.SetDecay(pheromoneDecayRate);

	// We get input in degrees from the XML file for the user's ease of use.
	CDegrees angleInDegrees;

    GetNodeAttribute(GetNode(node, "navigation"), "uninformedSearchCorrelation", angleInDegrees);
    // Convert the input from angleInDegrees to Radians.
    uninformedSearchCorrelation = ToRadians(angleInDegrees);

    GetNodeAttribute(GetNode(node, "navigation"), "angleTolerance", angleInDegrees);
    // Convert the input from angleInDegrees to radians.
    angleTolerance.Set(-ToRadians(angleInDegrees), ToRadians(angleInDegrees));

    // Square all of the Squared input variables
    searchRadiusSquared *= searchRadiusSquared;

	// Initialize the random number generator using the "random_seed" found in the XML file.
	RNG = CRandom::CreateRNG("argos");
}

/*
 * iAnt_controller Helper Function
 *
 * The CCI_FootBotMotorGroundSensor sensors are built on the motor PCB and they are located close
 * to the motors. There are four sensors and are useful to detect changes in color on the ground.
 *
 * The readings are in the following order (seeing the robot from TOP, battery socket is the BACK):
 *
 *   /---[front]---\    The color values read by these sensors ranges in value from 0.0 to 1.0,
 * l|w             r|w  where 0.0 = black and 1.0 = white. iAnt_loop_functions uses the predefined
 * e|h   1     0   i|h  CColor gray-scale colors to set up various objects on the field.
 * f|e             g|e
 * t|e   2     3   h|e  Arena Floor = [CColor::WHITE]  reading value is approximately 1.00
 *  |l             t|l  Food Item   = [CColor::BLACK]  reading value is approximately 0.00
 *   \---[back ]---/    Nest Zone   = [CColor::GRAY80] reading value is approximately 0.80
 *                      Pheromone   = [CColor::GRAY40] reading value is approximately 0.40
 *
 * This function is used to determine if the robot is currently inside the Nest Zone.
 *
 * @return     bool     "true" if the robot is currently within the borders of the nest,
 *                      "false" if the robot is not currently within the borders of the nest
 */
bool iAnt_controller::IsInTheNest() {
	// Obtain the current ground sensor readings for this controller object.
	const CCI_FootBotMotorGroundSensor::TReadings& groundReadings = groundSensor->GetReadings();
	// The ideal value is 0.8, but we must account for sensor read errors (+/- 0.1).
	CRange<Real> nestSensorRange(0.7, 0.9);
	// Assign the ground readings to temporary variables for clarity.
	Real backLeftWheelReading  = groundReadings[2].Value;
	Real backRightWheelReading = groundReadings[3].Value;

	// We only need to check the back side sensors. If these are in the nest so is the front.
	if(nestSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backLeftWheelReading) &&
	   nestSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backRightWheelReading)) {
	    return true; // robot is in the nest zone
	}

	return false; // robot is not in the nest zone
}

/*
 * iAnt_controller Helper Function
 *
 * The CCI_FootBotMotorGroundSensor sensors are built on the motor PCB and they are located close
 * to the motors. There are four sensors and are useful to detect changes in color on the ground.
 *
 * The readings are in the following order (seeing the robot from TOP, battery socket is the BACK):
 *
 *   /---[front]---\    The color values read by these sensors ranges in value from 0.0 to 1.0,
 * l|w             r|w  where 0.0 = black and 1.0 = white. iAnt_loop_functions uses the predefined
 * e|h   1     0   i|h  CColor gray-scale colors to set up various objects on the field.
 * f|e             g|e
 * t|e   2     3   h|e  Arena Floor = [CColor::WHITE]  reading value is approximately 1.00
 *  |l             t|l  Food Item   = [CColor::BLACK]  reading value is approximately 0.00
 *   \---[back ]---/    Nest Zone   = [CColor::GRAY80] reading value is approximately 0.80
 *                      Pheromone   = [CColor::GRAY40] reading value is approximately 0.40
 *
 * This function is used to determine if the robot is currently on top of a food item.
 *
 * @return     bool     "true" if the robot is currently on top of a food item,
 *                      "false" if the robot is not currently on top of a food item
 */
bool iAnt_controller::IsFindingFood() {
	// Obtain the current ground sensor readings for this controller object.
	const CCI_FootBotMotorGroundSensor::TReadings& groundReadings = groundSensor->GetReadings();
	// The ideal value is 0.0, but we must account for sensor read errors (+/- 0.1).
	CRange<Real> foodSensorRange(-0.1, 0.1);
	// Assign the ground readings to temporary variables for clarity.
	Real frontRightWheelReading = groundReadings[0].Value;
	Real frontLeftWheelReading  = groundReadings[1].Value;
	Real backLeftWheelReading   = groundReadings[2].Value;
	Real backRightWheelReading  = groundReadings[3].Value;

	// Pick up a food item if ANY of the four sensors is detecting it.
	if(foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(frontRightWheelReading) ||
	   foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(frontLeftWheelReading ) ||
	   foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backLeftWheelReading  ) ||
	   foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backRightWheelReading )) {
	    return true; // found food
	}

	return false; // has not found food
}

// Is the robot holding a food item?
bool iAnt_controller::IsHoldingFood() {
	return holdingFood;
}

// the robot is picking up a food item
void iAnt_controller::PickupFood() {
	holdingFood = true;
}

// the robot is dropping off a food item
void iAnt_controller::DropOffFood() {
	holdingFood = false;
}

// update the list of available food positions
void iAnt_controller::UpdateFoodList(vector<CVector2> newFoodPositions) {
	foodPositions = newFoodPositions;
}

// update the iAnt's position
void iAnt_controller::UpdatePosition(CVector2 newPosition) {
	position = newPosition;
}

// update simulation time
void iAnt_controller::UpdateTime(long int newTime) {
	simTime = newTime;
}
// return the robot's position
CVector2 iAnt_controller::Position()
{
	return position;
}

/* iAnt_controller Control Step Function
 *
 * This function is the primary integration between this class and the ARGoS simulator. ARGoS will
 * call this function once per frame or tick for each currently active robot using this controller.
 * The control scheme for this controller is modeled after a state machine, but it is not a true
 * state machine do to various modifications.
 */
void iAnt_controller::ControlStep() {
	LOG << target << endl;

	// Check for collisions and move out of the way before running the state machine.
	if(!collisionDetection()) {
		// Perform actions based on the modified state machine.
		switch(CPFA) {
			// The robot will select a location to search for food.
			case SET_SEARCH_LOCATION:
				setSearchLocation();
				break;
			// The robot will travel to the location it has selected to search from.
			case TRAVEL_TO_SEARCH_SITE:
				travelToSearchSite();
				break;
			// The robot will perform an informed walk while searching for food.
			case PERFORM_INFORMED_WALK:
				performInformedWalk();
				break;
			// The robot will perform an uninformed walk while searching for food.
			case PERFORM_UNINFORMED_WALK:
				performUninformedWalk();
				break;
			// The robot has found food and is checking the local resource density.
			case SENSE_LOCAL_RESOURCE_DENSITY:
				senseLocalResourceDensity();
				break;
			// The robot is traveling to the nest after finding food or giving up a search.
			case TRAVEL_TO_NEST:
				travelToNest();
		}
	}
}

/* iAnt_controller Reset Function
 *
 * This function is called when the simulation user presses the reset button on the ARGoS simulator
 * GUI. Most variables in this controller are automatically set or only need to be set once except
 * for the ones in this reset list which are reset to default initialized values.
 */
void iAnt_controller::Reset() {
	// Restart the simulation with the CPFA in the REST state.
	CPFA = SET_SEARCH_LOCATION;

	// Reset food data for this controller.
	holdingFood = false;
}

/* iAnt_controller Destroy Function
 *
 * This function is called after the user exits the simulation GUI. Any file read/write objects
 * or other objects you must delete or otherwise terminate are to be closed or deleted here. This
 * function operates similarly to a destructor function and undoes whatever was done by the call
 * to the iAnt_controller::Init(TConfigurationNode& node) function. Currently unimplemented and
 * unnecessary. This function is kept here for completeness and possible future expansion of the
 * iAnt_controller class.
 */
void iAnt_controller::Destroy() {
	// not in use
}

/* CPFA::SET_SEARCH_LOCATION State Function
 *
 * When in this state, the robot will determine the next area to move to before it begins a search
 * for food items. The robot will pick a random heading angle from 0 to 2 PI and head in that
 * direction until the travel probability causes the robot to switch to searching mode.
 */
void iAnt_controller::setSearchLocation() {
	// Always make sure the robot isn't carrying food first.
	if(IsHoldingFood()) {
		CPFA = SENSE_LOCAL_RESOURCE_DENSITY;
	}
	else {
		// Set the target vector to a length that will always reach outside of the arena bounds.
		//
		// Recall that the arena's coordinate domain is [-arenaSize.GetX()/2.0, arenaSize.GetX()/2.0]
		// and that its coordinate range is [-arenaSize.GetY()/2.0, arenaSize.GetY()/2.0].
		//
		// In other words, head straight out until we run into the wall OR the CPFA's search
		// probability causes the robot to change its direction and search a location for food.
		Real length(searchStepSize);

		// Obtain a heading angle from a uniform distribution between 0 and 2 PI.
		CRadians angle(RNG->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI)));

		// Set the navigation target to the position given by the length and angle parameters
		// and do so in relation to the robot's position and NOT the origin of the arena.
		target = getVectorToPosition(CVector2(length, angle) + position);

		// Search location has been set, change to travel state.
		CPFA = TRAVEL_TO_SEARCH_SITE;
	}
}

void iAnt_controller::travelToSearchSite() {
	if(IsHoldingFood()) {
		CPFA = SENSE_LOCAL_RESOURCE_DENSITY;
	}
	else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < travelProbability) {
		CPFA = PERFORM_UNINFORMED_WALK;
	}
	else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < searchProbability) {
		CPFA = PERFORM_INFORMED_WALK;
	}
	//else {
		//CPFA = TRAVEL_TO_NEST;
	//}
}

void iAnt_controller::performInformedWalk() {
	if(IsHoldingFood()) {
		CPFA = SENSE_LOCAL_RESOURCE_DENSITY;
	}
	else if(targetPheromone.IsActive()) {
		LOG << "target = pheromone" << endl;//////////////////////////////////////////////////////
		target = targetPheromone.Location();
	}
	else if(siteFidelity){
		LOG << "target = fidelity" << endl;//////////////////////////////////////////////////////
		target = fidelityPosition;
	}
	else {
		setWheelSpeed(getVectorToPosition(target));
	}
}

void iAnt_controller::performUninformedWalk() {
	if(IsHoldingFood()) {
		CPFA = SENSE_LOCAL_RESOURCE_DENSITY;
	}
	else if((position - target).SquareLength() < distanceTolerance) {
		// Get a random rotation angle and then add it to the getVectorToLight angle. This serves the functionality
		// of a compass and causes the rotation to be relative to the robot's current direction.
   		CRadians rotation(RNG->Gaussian(uninformedSearchCorrelation.GetValue())),
    			 angle(getVectorToLight().Angle().SignedNormalize() + rotation);

   		// Move from the current position "searchStepSize" distance away after turning "angle" degrees/radians etc.
   		target = (CVector2(searchStepSize, angle) + position);

   		LOG << "target = random" << endl;//////////////////////////////////////////////////////

   		// Bounds check: make sure the new position is not outside of the available arena space if the robot is near the edge.
   		// Note To Self: this bounds checking is completely borked... fix it... :'(
   		Real scale(0.8);

   		if(target.GetX() > (arenaSize.GetX() * scale / 2.0)/*forageRangeX.GetMax()*/) {
    		target.SetX((arenaSize.GetX() * scale / 2.0)/*forageRangeX.GetMax()*/);
    	} else if(target.GetX() < (-arenaSize.GetX() * scale / 2.0)/*forageRangeX.GetMin()*/) {
    		target.SetX((-arenaSize.GetX() * scale / 2.0)/*forageRangeX.GetMin()*/);
       	}

   		if(target.GetY() > (arenaSize.GetY() * scale / 2.0)/*forageRangeY.GetMax()*/) {
  			target.SetY((arenaSize.GetY() * scale / 2.0)/*forageRangeY.GetMax()*/);
        } else if(target.GetY() < (-arenaSize.GetY() * scale / 2.0)/*forageRangeY.GetMin()*/) {
        	target.SetY((-arenaSize.GetY() * scale / 2.0)/*forageRangeY.GetMin()*/);
        }
	}
	else {
        setWheelSpeed(getVectorToPosition(target));
	}
}

void iAnt_controller::senseLocalResourceDensity()
{
	resourceDensity = -1; // DON'T count the food item robot just found

	for(size_t i = 0; i < foodPositions.size(); i++) {
		if((position - foodPositions[i]).SquareLength() < searchRadiusSquared) {
			resourceDensity++;
		}
	}

	// TODO Get rid of magic numbers!
	if(resourceDensity >= 3) {
		siteFidelity = true;
		fidelityPosition = position;
		// if(/* probability logic for setting a pheromone goes here */) {
			sharedPheromone.Reset(position, simTime);
		//}
	}

	CPFA = TRAVEL_TO_NEST;
}

void iAnt_controller::travelToNest() {
	// If the robot has arrived inside the nest zone, transition to the "start" state.
	if(IsInTheNest()) {
		CPFA = SET_SEARCH_LOCATION;
    }
	// Otherwise, set the robot's heading toward the nest zone and move towards it.
    else {
    	setWheelSpeed(getVectorToLight());
    }
}

bool iAnt_controller::collisionDetection() {
	const CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();
	CVector2 accumulator;

	for(size_t i = 0; i < proximityReadings.size(); ++i) {
        accumulator += CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
	}

	accumulator /= proximityReadings.size();
	CRadians angle = accumulator.Angle();

	if(angleTolerance.WithinMinBoundIncludedMaxBoundIncluded(angle) &&
	   accumulator.Length() < distanceTolerance) {
		steeringActuator->SetLinearVelocity(maxSpeed, maxSpeed);
		return false; // collision not detected
	} else {
		if(angle.GetValue() > 0.0) {
			steeringActuator->SetLinearVelocity(maxSpeed, 0.0);
		} else {
			steeringActuator->SetLinearVelocity(0.0, maxSpeed);
		}
		return true; // collision detected
	}

	return false; // collision not detected
}

CRadians iAnt_controller::lawOfCosines(CVector2& A, CVector2& B, CVector2& C) {
    // the length of each side of the calculated triangle
    Real a(sqrt(((B.GetX() - A.GetX()) * (B.GetX() - A.GetX())) + ((B.GetY() - A.GetY()) * (B.GetY() - A.GetY())))),
         b(sqrt(((B.GetX() - C.GetX()) * (B.GetX() - C.GetX())) + ((B.GetY() - C.GetY()) * (B.GetY() - C.GetY())))),
         c(sqrt(((A.GetX() - C.GetX()) * (A.GetX() - C.GetX())) + ((A.GetY() - C.GetY()) * (A.GetY() - C.GetY()))));

    // determine whether we must add or subtract the rotation angle
    Real sign(getSignOfRotationAngle(A, B, C));

    // formula for the law of cosines
    return CRadians(sign * acos(((a * a) + (b * b) - (c * c)) / (2.0 * a * b)));
}

Real iAnt_controller::getSignOfRotationAngle(CVector2& A, CVector2& B, CVector2& C) {
    // returned as is for positive rotation, -1.0 for negative rotation
    Real result(1.0);

    // create a reference point midway between B and C
    CVector2 referencePoint(((B.GetX() + C.GetX())/2.0), ((B.GetY() + C.GetY())/2.0));

    // slope of the line created by the points B and C
    Real rise(B.GetY() - C.GetY()), run(B.GetX() - C.GetX());

    // avoid division by 0
    if(run == 0.0) { run += 0.001; }
    // and calculate the slope of the line created by points B and C
    Real slope(rise / run);

    // Is the nest above or below the line created by B and C?
    // we are using the point-slope formula for a line in this calculation
    bool nestIsAboveTheLine(A.GetY() > ((slope * (A.GetX() - referencePoint.GetX())) + referencePoint.GetY()));

    // Is the foot-bot to the left or right of the target?
    bool B_is_left_of_C(B.GetX() < C.GetX());

    if(nestIsAboveTheLine) {
        if(B_is_left_of_C) {
            result *= -1.0;
        }
    } else {
        if(!B_is_left_of_C) {
            result *= -1.0;
        }
    }

    return result;
}

CVector2 iAnt_controller::getVectorToLight() {
	const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
	CVector2 accumulator;

	for(size_t i = 0; i < readings.size(); ++i) {
	    accumulator += CVector2(readings[i].Value, readings[i].Angle);
	}

	return accumulator;
}

CVector2 iAnt_controller::getVectorToPosition(const CVector2& targetPosition) {
    const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
    CVector2 accumulator;
    // we will construct a triangle using these points: A, B, C
    CVector2 A(nestPosition), B(position), C(targetPosition);
    CRadians rotationTowardsTarget(lawOfCosines(A, B, C));

    for(size_t i = 0; i < readings.size(); ++i) {
        accumulator += CVector2(readings[i].Value, readings[i].Angle + rotationTowardsTarget);
    }

    return accumulator;
}

void iAnt_controller::setWheelSpeed(const CVector2& heading) {
	CRadians headingAngle = heading.Angle().SignedNormalize();
	enum turnStatus { NO_TURN = 0, TURN = 1 } turnStatus;

   if(Abs(headingAngle) < angleTolerance.GetMax()) {
	   turnStatus = NO_TURN;
   } else if(Abs(headingAngle) > angleTolerance.GetMax()) {
	   turnStatus = TURN;
   }

   // Wheel speeds based on current turning state
   Real speed1, speed2;

   switch(turnStatus) {
      case NO_TURN: {
         // Just go straight
         speed1 = maxSpeed;
         speed2 = maxSpeed;
         break;
      }
      case TURN: {
         // Opposite wheel speeds
         speed1 = -maxSpeed;
         speed2 =  maxSpeed;
         break;
      }
   }

   Real leftWheelSpeed, rightWheelSpeed;

   if(headingAngle > CRadians::ZERO) {
      // Turn Left
      leftWheelSpeed  = speed1;
      rightWheelSpeed = speed2;
   } else {
      // Turn Right
      leftWheelSpeed  = speed2;
      rightWheelSpeed = speed1;
   }

   steeringActuator->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);
}

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")

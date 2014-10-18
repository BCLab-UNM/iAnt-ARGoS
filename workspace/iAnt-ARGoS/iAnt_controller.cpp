#include "iAnt_controller.h"

/* inline functions and constants */
/* TODO verify necessity of these constants */

/*
static const CVector2 NullVector = CVector2(-1, -1);
static const double m_fWheelVelocity = 16; // this is maxSpeed
static const double m_fWheelVelocityAligning = 5;
*/

// exponential decay
static inline float exponentialDecay(float quantity, float time, float lambda) {
	return (quantity * exp(-lambda * time));
}

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

// constructor, see iAnt_controller::Init(TConfigurationNode& node);
iAnt_controller::iAnt_controller() :
	steeringActuator(NULL),
	proximitySensor(NULL),
	groundSensor(NULL),
	//lightSensor(NULL),
	m_pcPositioning(NULL),
	RNG(NULL),
	holdingFood(false),
	informed(false),
	resourceDensity(0),
	searchRadiusSquared(0.0),
	distanceTolerance(0.0),
	travelGiveupProbability(0.0),
	searchGiveupProbability(0.0),
	searchStepSize(0.0),
	maxSpeed(0.0),
	informedSearchDecay(0.0),
	siteFidelityRate(0.0),
	pheromoneRate(0.0),
	pheromoneDecayRate(0.0),
	simTime(0),
	searchTime(0),
	// CPFA(TRAVEL_TO_NEST)
	CPFA(INACTIVE)
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
	//lightSensor      = GetSensor<CCI_FootBotLightSensor>            ("footbot_light"        );
    m_pcPositioning  = GetSensor<CCI_PositioningSensor>             ("positioning"          );

	GetNodeAttribute(GetNode(node, "navigation"), "searchRadius"           , searchRadiusSquared);
	GetNodeAttribute(GetNode(node, "navigation"), "arenaSize"              , arenaSize);
	GetNodeAttribute(GetNode(node, "navigation"), "nestPosition"           , nestPosition);
	GetNodeAttribute(GetNode(node, "navigation"), "distanceTolerance"      , distanceTolerance);
	GetNodeAttribute(GetNode(node, "navigation"), "searchGiveupProbability", searchGiveupProbability);
	GetNodeAttribute(GetNode(node, "navigation"), "searchStepSize"         , searchStepSize);
	GetNodeAttribute(GetNode(node, "navigation"), "maxSpeed"               , maxSpeed);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "informedSearchDecay"    , informedSearchDecay);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "siteFidelityRate"       , siteFidelityRate);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "pheromoneRate"          , pheromoneRate);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "pheromoneDecayRate"     , pheromoneDecayRate);
	GetNodeAttribute(GetNode(node, "CPFA"      ), "travelGiveupProbability", travelGiveupProbability);

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

// update pheromone
void iAnt_controller::TargetPheromone(iAnt_pheromone p)
{
	targetPheromone.Set(p);
}

/* iAnt_controller Control Step Function
 *
 * This function is the primary integration between this class and the ARGoS simulator. ARGoS will
 * call this function once per frame or tick for each currently active robot using this controller.
 * The control scheme for this controller is modeled after a state machine, but it is not a true
 * state machine do to various modifications.
 */
void iAnt_controller::ControlStep() {
	// LOG << "T: " << target << endl << "P: " << position << endl << CRadians(getHeading()) << endl << (target - position).Angle() << endl << endl;

	// Check for collisions and move out of the way before running the state machine.
	if(!collisionDetection()) {
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
	// todo make sure this reset function is actually resetting "EVERYTHING" it needs to...
	// Restart the simulation with the CPFA in the REST state.
	CPFA = INACTIVE;

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

void iAnt_controller::inactive() {
	setRandomSearchLocation();
	CPFA = DEPARTING;
}

void iAnt_controller::departing() {
	if(informed == false) {
		if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < travelGiveupProbability) {
			searchTime = 0;
			CPFA = SEARCHING;
		}
	}
	else if((position - target).SquareLength() < distanceTolerance) {
		searchTime = 0;
		CPFA = SEARCHING;
		informed = false;
	}

	setWheelSpeed(); //setWheelSpeed(getVectorToPosition(target));
}

void iAnt_controller::searching() {
	if(IsHoldingFood() == false) {
		if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < searchGiveupProbability) {
			target = setPositionInBounds(nestPosition);
			CPFA = RETURNING;
		}
		else /*if((position - target).SquareLength() < distanceTolerance)*/ {
			if(informed == false) {
				// Get a random rotation angle and then add it to the getVectorToLight angle. This serves the functionality
				// of a compass and causes the rotation to be relative to the robot's current direction.
				CRadians rotation(RNG->Gaussian(uninformedSearchCorrelation.GetValue())),
						 //angle(getVectorToLight().Angle().SignedNormalize() + rotation);
				         angle((getHeading() > 0) ? (rotation + CRadians(getHeading())) : (rotation - CRadians(getHeading())));

				// Move from the current position "searchStepSize" distance away after turning "angle" degrees/radians etc.
				target = setPositionInBounds(CVector2(searchStepSize, angle) + position);
			}
			else {
				float correlation = exponentialDecay((CRadians::TWO_PI).GetValue(), searchTime++, informedSearchDecay);
				CRadians rotation(bound(correlation, -(CRadians::PI).GetValue(), (CRadians::PI).GetValue())),
				         //angle(getVectorToLight().Angle().SignedNormalize() + rotation);
				         angle((getHeading() > 0) ? (rotation + CRadians(getHeading())) : (rotation - CRadians(getHeading)));

				target = setPositionInBounds(CVector2(searchStepSize, angle) + position);
			}
		}
	}
	else {
		senseLocalResourceDensity();
		target = setPositionInBounds(nestPosition);
		CPFA = RETURNING;
	}

	setWheelSpeed(); //setWheelSpeed(getVectorToPosition(target));
}

void iAnt_controller::returning() {
	if((position - target).SquareLength() < distanceTolerance) {
		if(poissonCDF(resourceDensity, pheromoneRate) > RNG->Uniform(CRange<Real>(0.0, 1.0))) {
			sharedPheromone.Set(iAnt_pheromone(position, simTime, pheromoneDecayRate));
		}

		if(poissonCDF(resourceDensity, siteFidelityRate) > RNG->Uniform(CRange<Real>(0.0, 1.0))) {
			target = setPositionInBounds(fidelityPosition);
			informed = true;
		}
		else if(targetPheromone.IsActive() == true) {
			target = setPositionInBounds(targetPheromone.Location());
			informed = true;
		}
		else {
			informed = false;
			setRandomSearchLocation();
		}

		CPFA = DEPARTING;
	}
	else setWheelSpeed(); //setWheelSpeed(getVectorToLight());
}

void iAnt_controller::senseLocalResourceDensity()
{
	resourceDensity = -1; // DON'T count the food item robot just found

	for(size_t i = 0; i < foodPositions.size(); i++) {
		if((position - foodPositions[i]).SquareLength() < searchRadiusSquared) {
			resourceDensity++;
		}
	}

	fidelityPosition = position;
}

CVector2 iAnt_controller::setPositionInBounds(CVector2 rawPosition) {
	// create x and y point ranges at 90% of max grid size
	CRange<Real> x(-0.9*arenaSize.GetX()/2.0, 0.9*arenaSize.GetX()/2.0);
	CRange<Real> y(-0.9*arenaSize.GetY()/2.0, 0.9*arenaSize.GetY()/2.0);

	if(rawPosition.GetX() > x.GetMax()) rawPosition.SetX(x.GetMax());
	else if(rawPosition.GetX() < x.GetMin()) rawPosition.SetX(x.GetMin());

	if(rawPosition.GetY() > y.GetMax()) rawPosition.SetY(y.GetMax());
	else if(rawPosition.GetY() < y.GetMin()) rawPosition.SetY(y.GetMin());

	return rawPosition;
}

// set target to a random location
void iAnt_controller::setRandomSearchLocation() {
	// create x and y point ranges at 90% of max grid size
	CRange<Real> x(-0.9*arenaSize.GetX()/2.0, 0.9*arenaSize.GetX()/2.0);
	CRange<Real> y(-0.9*arenaSize.GetY()/2.0, 0.9*arenaSize.GetY()/2.0);

	// randomly set the target somewhere in the arena
	target.SetX(RNG->Uniform(x));
	target.SetY(RNG->Uniform(y));
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
    Real slope = rise / run;

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

/*
 * ADD to this heading to turn RIGHT
 * SUBTRACT to this heading to turn LEFT
 *
 * this reading will give a value:
 *
 * 0 degrees points due north,
 * 90 degrees east,
 * +/- 180 degrees south,
 * -90 degrees west
 */
double iAnt_controller::getHeading() {
    const CCI_PositioningSensor::SReading& sReading = m_pcPositioning->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /*Convert quaternion to euler*/
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /*Angle to z-axis represents compass heading*/
    return -z_angle.GetValue();
}


CVector2 iAnt_controller::getVectorToLight() {
//	const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
	CVector2 accumulator;

//	for(size_t i = 0; i < readings.size(); ++i) {
//	    accumulator += CVector2(readings[i].Value, readings[i].Angle);
//	}

	return accumulator;
}

CVector2 iAnt_controller::getVectorToPosition(const CVector2& targetPosition) {
//    const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
    CVector2 accumulator;
    // we will construct a triangle using these points: A, B, C
    CVector2 A(nestPosition), B(position), C(targetPosition);
    CRadians rotationTowardsTarget(lawOfCosines(A, B, C));

//    for(size_t i = 0; i < readings.size(); ++i) {
//        accumulator += CVector2(readings[i].Value, readings[i].Angle + rotationTowardsTarget);
//    }

    return accumulator;
}

void iAnt_controller::setWheelSpeed() {
	// ORDER MATTERS for (target - position).Angle() calculation, always do target first
	// to ensure opposite sign of getHeading() call in the case robot is directly facing its target
	CRadians headingAngle = CRadians(getHeading()) + (target - position).Angle();
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

#include "iAnt_controller.h"

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
static inline Real poissonCDF(Real k, Real lambda) {
    Real sumAccumulator       = 1.0;
    Real factorialAccumulator = 1.0;

    for (size_t i = 1; i <= floor(k); i++) {
        factorialAccumulator *= i;
        sumAccumulator += pow(lambda, i) / factorialAccumulator;
    }

    return (exp(-lambda) * sumAccumulator);
}

// constructor, see iAnt_controller::Init(TConfigurationNode& node);
iAnt_controller::iAnt_controller():
	steeringActuator(NULL),
	proximitySensor(NULL),
	compassSensor(NULL),
	RNG(NULL),
	holdingFood(false),
	informed(false),
	collisionDelay(0),
	resourceDensity(0),
    nestRadiusSquared(0.0),
    foodRadiusSquared(0.0),
	searchRadiusSquared(0.0),
	distanceTolerance(0.0),
	travelGiveupProbability(0.0),
	searchGiveupProbability(0.0),
	searchStepSize(0.0),
	maxRobotSpeed(0.0),
	informedSearchDecay(0.0),
	siteFidelityRate(0.0),
	pheromoneRate(0.0),
	pheromoneDecayRate(0.0),
	simTime(0),
	searchTime(0),
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
    compassSensor    = GetSensor<CCI_PositioningSensor>             ("positioning"          );

	GetNodeAttribute(GetNode(node, "navigation"), "searchRadius"           , searchRadiusSquared);
	GetNodeAttribute(GetNode(node, "navigation"), "distanceTolerance"      , distanceTolerance);
	GetNodeAttribute(GetNode(node, "navigation"), "searchGiveupProbability", searchGiveupProbability);
	GetNodeAttribute(GetNode(node, "navigation"), "searchStepSize"         , searchStepSize);
	GetNodeAttribute(GetNode(node, "navigation"), "maxRobotSpeed"          , maxRobotSpeed);
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
 ******************************************************************************/
bool iAnt_controller::IsInTheNest() {
    return ((GetPosition() - nestPosition).SquareLength() < nestRadiusSquared);
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
    /*
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
	} */

    Real dts = (distanceTolerance * distanceTolerance);

    for(size_t i = 0; i < foodPositions.size(); i++) {
        if((GetPosition() - foodPositions[i]).SquareLength() < dts) {
            return true; // robot is finding food
        }
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
void iAnt_controller::SetFoodPositions(vector<CVector2> fp) {
	foodPositions = fp;
}

// update pheromone positions
void iAnt_controller::SetPheromonePositions(vector<CVector2> pp) {
    pheromonePositions = pp;
}

// update fidelity positions
void iAnt_controller::SetFidelityPositions(vector<CVector2> fp) {
    fidelityPositions = fp;
}
// update the iAnt's position
void iAnt_controller::SetPosition(CVector2 p) {
	//position = p;
}

// update simulation time
void iAnt_controller::SetTime(size_t t) {
	simTime = t;
}

// return the robot's position
CVector2 iAnt_controller::GetPosition() {
    CVector3 position3D = compassSensor->GetReading().Position;
    return CVector2(position3D.GetX(), position3D.GetY());
}

// return the robot's fidelity position
CVector2 iAnt_controller::GetFidelityPosition() {
    return fidelityPosition;
}

// set new nest position
void iAnt_controller::SetNestPosition(CVector2 np) {
    nestPosition = np;
}

void iAnt_controller::SetNestRadiusSquared(Real r) {
    nestRadiusSquared = r;
}

// set squaqred radius of food items
void iAnt_controller::SetFoodRadiusSquared(Real rs) {
    foodRadiusSquared = rs;
}

// update pheromone
void iAnt_controller::SetTargetPheromone(iAnt_pheromone tp) {
	targetPheromone.Set(tp);
}

// setup forage boundary
void iAnt_controller::SetForageRange(CRange<Real> x, CRange<Real> y) {
    forageRangeX = x;
    forageRangeY = y;
}

// get nest position
CVector2 iAnt_controller::GetNestPosition() {
    return nestPosition;
}

// get nest radius
Real iAnt_controller::GetNestRadius() {
    return sqrt(nestRadiusSquared);
}

// get food radius
Real iAnt_controller::GetFoodRadius() {
    return sqrt(foodRadiusSquared);
}

// get food position list from robot
vector<CVector2> iAnt_controller::GetFoodPositions() {
    return foodPositions;
}

// get pheromone position list from robot
vector<CVector2> iAnt_controller::GetPheromonePositions() {
    return pheromonePositions;
}

// get fidelity position list
vector<CVector2> iAnt_controller::GetFidelityPositions() {
    return fidelityPositions;
}

// get pheromone for master pheromone list in loop_functions
iAnt_pheromone iAnt_controller::GetTargetPheromone() {
	return sharedPheromone;
}

/* iAnt_controller Control Step Function
 *
 * This function is the primary integration between this class and the ARGoS simulator. ARGoS will
 * call this function once per frame or tick for each currently active robot using this controller.
 * The control scheme for this controller is modeled after a state machine, but it is not a true
 * state machine do to various modifications.
 */
void iAnt_controller::ControlStep() {
    if(foodPositions.size() == 0 && !IsHoldingFood()) {
		targetPosition = setPositionInBounds(nestPosition);

        if((GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
            steeringActuator->SetLinearVelocity(0.0, 0.0);
        } else {
            SetWheelSpeed();
        }
    } else {
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
	// TODO make sure this reset function is actually resetting "EVERYTHING" it needs to...

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
	SetRandomSearchLocation();
	CPFA = DEPARTING;
}

void iAnt_controller::departing() {
    if(IsHoldingFood() == true) {
		if(IsFindingFood() == true) senseLocalResourceDensity();
		targetPosition = setPositionInBounds(nestPosition);
		CPFA = RETURNING;
	} else if(informed == false) {
		if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < travelGiveupProbability) {
			searchTime = 0;
			CPFA = SEARCHING;
		}
	} else if((GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
		searchTime = 0;
		CPFA = SEARCHING;
		informed = false;
	}

	SetWheelSpeed();
}

void iAnt_controller::searching() {
	if(IsHoldingFood() == false) {
		if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < searchGiveupProbability) {
			targetPosition = setPositionInBounds(nestPosition);
			CPFA = RETURNING;
		}
		else if((simTime % 8 == 0) && (GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
			if(informed == false) {
				// Get a random rotation angle and then add it to the getVectorToLight angle. This serves the functionality
				// of a compass and causes the rotation to be relative to the robot's current direction.
				CRadians rotation(RNG->Gaussian(uninformedSearchCorrelation.GetValue())),
						 angle(rotation.UnsignedNormalize() + RobotHeading().UnsignedNormalize());

				targetPosition = setPositionInBounds(CVector2(searchStepSize, angle) + GetPosition());
			}
			else {
				float correlation = exponentialDecay((CRadians::TWO_PI).GetValue(), searchTime++, informedSearchDecay);
				CRadians rotation(bound(correlation, -(CRadians::PI).GetValue(), (CRadians::PI).GetValue())),
						 angle(rotation.UnsignedNormalize() + RobotHeading().UnsignedNormalize());

				targetPosition = setPositionInBounds(CVector2(searchStepSize, angle) + GetPosition());
			}
		}
	} else {
		if(IsFindingFood() == true) senseLocalResourceDensity();
		targetPosition = setPositionInBounds(nestPosition);
		CPFA = RETURNING;
	}

	SetWheelSpeed();
}

void iAnt_controller::returning() {
	if((GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
		if(poissonCDF(resourceDensity, pheromoneRate) > RNG->Uniform(CRange<Real>(0.0, 1.0))) {
			sharedPheromone.Set(iAnt_pheromone(fidelityPosition, simTime, pheromoneDecayRate));
		}

		if(poissonCDF(resourceDensity, siteFidelityRate) > RNG->Uniform(CRange<Real>(0.0, 1.0))) {
			targetPosition = setPositionInBounds(fidelityPosition);
			informed = true;
		}
		else if(targetPheromone.IsActive() == true) {
			targetPosition = setPositionInBounds(targetPheromone.Location());
			informed = true;
		}
		else {
			informed = false;
			SetRandomSearchLocation();
		}

		CPFA = DEPARTING;
	}
	else SetWheelSpeed();
}

void iAnt_controller::senseLocalResourceDensity()
{
	resourceDensity = 0; // DO count the food item robot just found

	for(size_t i = 0; i < foodPositions.size(); i++) {
		if((GetPosition() - foodPositions[i]).SquareLength() < searchRadiusSquared) {
			resourceDensity++;
		}

        // clean location
        //if((GetPosition() - foodPositions[i]).SquareLength() < foodRadiusSquared) {
            //fidelityPosition = foodPositions[i];
        //}
	}

    // messy location
    fidelityPosition = GetPosition();
}

CVector2 iAnt_controller::setPositionInBounds(CVector2 p) {
	if(p.GetX() > forageRangeX.GetMax()) p.SetX(forageRangeX.GetMax());
	else if(p.GetX() < forageRangeX.GetMin()) p.SetX(forageRangeX.GetMin());

	if(p.GetY() > forageRangeY.GetMax()) p.SetY(forageRangeY.GetMax());
	else if(p.GetY() < forageRangeY.GetMin()) p.SetY(forageRangeY.GetMin());

	return p;
}

// set target to a random location
void iAnt_controller::SetRandomSearchLocation() {
	// randomly set the target somewhere in the arena
	targetPosition.SetX(RNG->Uniform(forageRangeX));
	targetPosition.SetY(RNG->Uniform(forageRangeY));

    targetPosition = setPositionInBounds(targetPosition);
}

bool iAnt_controller::CollisionDetection() {
	const CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();
	size_t collisionsDetected = 0;

	for(size_t i = 0; i < proximityReadings.size(); i++) {
		if((proximityReadings[i].Value > 0.0) &&
           (angleTolerance.WithinMinBoundIncludedMaxBoundIncluded(proximityReadings[i].Angle))) {
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
CRadians iAnt_controller::RobotHeading() {
    const CCI_PositioningSensor::SReading& sReading = compassSensor->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /* Convert quaternion to euler */
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* Angle to z-axis represents compass heading */
    return z_angle;
}

void iAnt_controller::SetWheelSpeed() {
	CRadians heading = (RobotHeading() - (targetPosition - GetPosition()).Angle()).SignedNormalize();

	if(CollisionDetection() == true) {
		// 32 = 2 seconds, 16 frames per second (set in XML) by 2
		collisionDelay = simTime + 32;
		/* turn left */
		steeringActuator->SetLinearVelocity(-maxRobotSpeed, maxRobotSpeed);
	} else if((heading <= angleTolerance.GetMin()) && (collisionDelay < simTime)) {
		/* turn left */
		steeringActuator->SetLinearVelocity(-maxRobotSpeed, maxRobotSpeed);
	} else if((heading >= angleTolerance.GetMax()) && (collisionDelay < simTime)) {
		/* turn right */
		steeringActuator->SetLinearVelocity(maxRobotSpeed, -maxRobotSpeed);
	} else {
		/* go straight */
		steeringActuator->SetLinearVelocity(maxRobotSpeed, maxRobotSpeed);
	}
}

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")

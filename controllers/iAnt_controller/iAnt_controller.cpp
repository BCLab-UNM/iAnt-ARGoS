#include "iAnt_controller.h"

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
  holdingFood(false),
  informed(false),
  resourceDensity(0),
  collisionDelay(0),
  simTime(0),
  searchTime(0),
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
* Destructor
********************************************************************************
* Currently not in use.
*******************************************************************************/
iAnt_controller::~iAnt_controller() {}

/*******************************************************************************
* Class Initialization Function
********************************************************************************
* This function uses an ARGoS configuration XML file to setup variables and
* settings. This function is inherited from the CCI_Controller class.
*
* @param node ARGoS XML configuration node
*******************************************************************************/
void iAnt_controller::Init(TConfigurationNode& node) {
  TConfigurationNode cpfa = GetNode(node, "CPFA");
  CDegrees angleInDegrees;

  typedef CCI_DifferentialSteeringActuator CCI_DSA;
  typedef CCI_FootBotProximitySensor       CCI_FBPS;
  typedef CCI_PositioningSensor            CCI_PS;

  steeringActuator = GetActuator<CCI_DSA>("differential_steering");
  proximitySensor  = GetSensor<CCI_FBPS>("footbot_proximity");
  compassSensor    = GetSensor<CCI_PS>("positioning");

  GetNodeAttribute(cpfa, "searchRadius", searchRadiusSquared);
  searchRadiusSquared *= searchRadiusSquared;

  GetNodeAttribute(cpfa, "uninformedSearchCorrelation", angleInDegrees);
  uninformedSearchCorrelation = ToRadians(angleInDegrees);

  GetNodeAttribute(cpfa, "distanceTolerance", distanceTolerance);
  GetNodeAttribute(cpfa, "angleTolerance", angleInDegrees);
  angleTolerance.Set(-ToRadians(angleInDegrees), ToRadians(angleInDegrees));

  GetNodeAttribute(cpfa, "searchGiveupProbability", searchGiveupProbability);
  GetNodeAttribute(cpfa, "travelGiveupProbability", travelGiveupProbability);
  GetNodeAttribute(cpfa, "siteFidelityRate", siteFidelityRate);
  GetNodeAttribute(cpfa, "informedSearchDecay", informedSearchDecay);
  GetNodeAttribute(cpfa, "pheromoneRate", pheromoneRate);
  GetNodeAttribute(cpfa, "pheromoneDecayRate", pheromoneDecayRate);
  targetPheromone.SetDecay(pheromoneDecayRate);
  sharedPheromone.SetDecay(pheromoneDecayRate);

  GetNodeAttribute(cpfa, "searchStepSize", searchStepSize);
  GetNodeAttribute(cpfa, "maxRobotSpeed", maxRobotSpeed);

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
    // d.t.s. = distance tolerance squared
    Real dts = (distanceTolerance * distanceTolerance);

    for(size_t i = 0; i < foodPositions.size(); i++) {
        if((GetPosition() - foodPositions[i]).SquareLength() < dts) {
            return true;
        }
    }

	return false;
}

/*******************************************************************************
* By design, iAnt robots cannot carry more then 1 piece of food at a time. This
* function is used to check the status of the holdingFood boolean flag.
*
* @return TRUE:  The robot is holding food.
*         FALSE: The robot is not holding food.
*******************************************************************************/
bool iAnt_controller::IsHoldingFood() {
	return holdingFood;
}

/*******************************************************************************
* If IsFindingFood() = true AND IsHoldingFood() = false, then this function
* should be called to indicate that the iAnt is picking up a food item.
*******************************************************************************/
void iAnt_controller::PickupFood() {
	holdingFood = true;
}

/*******************************************************************************
* If IsInTheNest() = true AND IsHoldingFood() = true, then this function
* should be called to indicate that the iAnt is dropping off a food item.
*******************************************************************************/
void iAnt_controller::DropOffFood() {
	holdingFood = false;
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
* variably change in size depending upon the number of robots and pheromone
* laying rate. This should be called by the iAnt_loop_functions class.
*******************************************************************************/
void iAnt_controller::SetPheromonePositions(vector<CVector2> pp) {
    pheromonePositions = pp;
}

/*******************************************************************************
* Update the list of all site fidelity positions on the map. This list should
* range in size from 0 to the maximum number of robots on the map. This should
* be called by the iAnt_loop_functions class.
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
* Return the Cartesian X,Y coordinates of the robot using the compassSensor.
*******************************************************************************/
CVector2 iAnt_controller::GetPosition() {
    // 3d Vector Position, x-position, y-position, z-position(height)
    CVector3 position3D = compassSensor->GetReading().Position;

    // we only need the X,Y coordinates
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
//    LOG << GetId() << " :: " << GetPosition() << endl << endl;
/*
    LOG << GetId() << " :: " << forageRangeX << endl << forageRangeY << endl << endl;
    LOG << GetId() << endl << "target: " << targetPosition << endl
        << "nest: " << nestPosition << endl;
*/

    if(foodPositions.size() == 0 && !IsHoldingFood()) {
		targetPosition = SetPositionInBounds(nestPosition);

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
    steeringActuator->Reset();
    proximitySensor->Reset();
    compassSensor->Reset();

    targetPosition   = nestPosition;
    fidelityPosition = nestPosition;

    targetPheromone.Reset(nestPosition, 0);
    sharedPheromone.Reset(nestPosition, 0);

    simTime = 0;

	// Restart the simulation with the CPFA in the REST state.
	CPFA = RETURNING;

	// Reset food data for this controller.
	holdingFood = informed = false;
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
		if(IsFindingFood() == true) SetLocalResourceDensity();
		targetPosition = SetPositionInBounds(nestPosition);
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
			targetPosition = SetPositionInBounds(nestPosition);
			CPFA = RETURNING;
		}
		else if((simTime % 8 == 0) && (GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
			if(informed == false) {
				// Get a random rotation angle and then add it to the getVectorToLight angle. This serves the functionality
				// of a compass and causes the rotation to be relative to the robot's current direction.
				CRadians rotation(RNG->Gaussian(uninformedSearchCorrelation.GetValue())),
						 angle(rotation.UnsignedNormalize() + GetRobotHeading().UnsignedNormalize());

				targetPosition = SetPositionInBounds(CVector2(searchStepSize, angle) + GetPosition());
			}
			else {
				float correlation = GetExponentialDecay((CRadians::TWO_PI).GetValue(), searchTime++, informedSearchDecay);
				CRadians rotation(GetBound(correlation, -(CRadians::PI).GetValue(), (CRadians::PI).GetValue())),
						 angle(rotation.UnsignedNormalize() + GetRobotHeading().UnsignedNormalize());

				targetPosition = SetPositionInBounds(CVector2(searchStepSize, angle) + GetPosition());
			}
		}
	} else {
		if(IsFindingFood() == true) SetLocalResourceDensity();
		targetPosition = SetPositionInBounds(nestPosition);
		CPFA = RETURNING;
	}

	SetWheelSpeed();
}

void iAnt_controller::returning() {
	if((GetPosition() - targetPosition).SquareLength() < distanceTolerance) {
		if(GetPoissonCDF(resourceDensity, pheromoneRate) > RNG->Uniform(CRange<Real>(0.0, 1.0))) {
			sharedPheromone.Set(iAnt_pheromone(fidelityPosition, simTime, pheromoneDecayRate));
		}

		if(GetPoissonCDF(resourceDensity, siteFidelityRate) > RNG->Uniform(CRange<Real>(0.0, 1.0))) {
			targetPosition = SetPositionInBounds(fidelityPosition);
			informed = true;
		}
		else if(targetPheromone.IsActive() == true) {
			targetPosition = SetPositionInBounds(targetPheromone.Location());
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

void iAnt_controller::SetLocalResourceDensity()
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
    fidelityPosition = SetPositionInBounds(GetPosition());
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
	// randomly set the target somewhere in the arena
	targetPosition.SetX(RNG->Uniform(forageRangeX));
	targetPosition.SetY(RNG->Uniform(forageRangeY));

    targetPosition = SetPositionInBounds(targetPosition);
}

bool iAnt_controller::IsCollisionDetected() {
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
CRadians iAnt_controller::GetRobotHeading() {
    const CCI_PositioningSensor::SReading& sReading = compassSensor->GetReading();
    CQuaternion orientation = sReading.Orientation;

    /* Convert quaternion to euler */
    CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* Angle to z-axis represents compass heading */
    return z_angle;
}

void iAnt_controller::SetWheelSpeed() {
	CRadians heading = (GetRobotHeading() - (targetPosition - GetPosition()).Angle()).SignedNormalize();

	if(IsCollisionDetected() == true) {
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

// exponential decay
Real iAnt_controller::GetExponentialDecay(Real quantity, Real time, Real lambda) {
	return (quantity * exp(-lambda * time));
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

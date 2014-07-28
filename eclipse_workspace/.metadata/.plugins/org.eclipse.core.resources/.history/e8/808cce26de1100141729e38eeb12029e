/* Find the class definition here. */
#include "iAnt_controller.h"

/****************************************************************************************************/
/* constructor, destructor, and initialization functions required for the iAnt_controller           */
/****************************************************************************************************/

/*
 * iAnt_controller Class Constructor Function
 *
 * Initialize iAnt_controller class variables. These values SHOULD be overwritten in a call to the
 * iAnt_controller::Init(TConfigurationNode& node) function.
 *
 */
iAnt_controller::iAnt_controller() :
	steeringActuator(NULL),
	proximitySensor(NULL),
	groundSensor(NULL),
	lightSensor(NULL),
	RNG(NULL)
{}

/****************************************************************************************************/

/*
 * iAnt_controller Class Destructor Function
 *
 * Currently unimplemented and unnecessary. This function is kept here for completeness and
 * possible future expansion of the iAnt_controller class.
 *
 */
iAnt_controller::~iAnt_controller() { /* Not in use. */ }

/****************************************************************************************************/

/*
 * iAnt_controller Class Initialization Function
 *
 * Set the iAnt_controller class variables from data in a *.argos XML file. This function is
 * inherited from the CCI_Controller class.
 *
 * @param     node     ARGoS XML configuration node
 *
 */
void iAnt_controller::Init(TConfigurationNode& node) {
	/* Initialize ARGoS sensors and actuators from these categories in the *.argos XML file. */
	steeringActuator = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor  = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
	groundSensor     = GetSensor<CCI_FootBotMotorGroundSensor>("footbot_motor_ground");
	lightSensor      = GetSensor<CCI_FootBotLightSensor>("footbot_light");

	/* Please note that although iAntData has an Init() function, it should NOT be called here.
	 * The node passed to this Init() function refers to the unused <params> tag in the
	 * <controllers> section of the XML file. This function will be called by the
	 * iAnt_loop_functions class during its Init() function where the appropriate values are
	 * stored inside of the <loop_functions> tag in the XML file.
	 *
	 * DO NOT CALL THIS HERE: iAntData.Init(node); */

	/* Initialize the random number generator using the "random_seed" found in the XML file. */
	RNG = CRandom::CreateRNG("argos");
}

/****************************************************************************************************/
/* public helper functions used internally and by the iAnt_loop_functions class                     */
/****************************************************************************************************/

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
bool iAnt_controller::isInTheNest() {
	/* Obtain the current ground sensor readings for this controller object. */
	const CCI_FootBotMotorGroundSensor::TReadings& groundReadings = groundSensor->GetReadings();

	/* The ideal value is 0.8, but we must account for sensor read errors (+/- 0.1). */
	CRange<Real> nestSensorRange(0.7, 0.9);

	/* Assign the ground readings to temporary variables for clarity. */
	Real backLeftWheelReading = groundReadings[2].Value,
		 backRightWheelReading = groundReadings[3].Value;

	/* We only need to check the back side sensors. If these are in the nest so is the front. */
	if(nestSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backLeftWheelReading) &&
	   nestSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backRightWheelReading)) {
		/* The robot is in the nest zone. */
	    return true;
	}

	/* The robot is not in the nest zone. */
	return false;
}

/****************************************************************************************************/

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
bool iAnt_controller::hasFoundFood() {
	/* Obtain the current ground sensor readings for this controller object. */
	const CCI_FootBotMotorGroundSensor::TReadings& groundReadings = groundSensor->GetReadings();

	/* The ideal value is 0.0, but we must account for sensor read errors (+/- 0.1). */
	CRange<Real> foodSensorRange(-0.1, 0.1);

	/* Assign the ground readings to temporary variables for clarity. */
	Real frontRightWheelReading = groundReadings[0].Value,
	     frontLeftWheelReading = groundReadings[1].Value,
	     backLeftWheelReading = groundReadings[2].Value,
		 backRightWheelReading = groundReadings[3].Value;

	/* Pick up a food item if ANY of the four sensors is detecting it. */
	if(foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(frontRightWheelReading) ||
	   foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(frontLeftWheelReading) ||
	   foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backLeftWheelReading) ||
	   foodSensorRange.WithinMinBoundIncludedMaxBoundIncluded(backRightWheelReading)) {
		/* The robot has found a food item. */
	    return true;
	}

	/* The robot has not found a food item. */
	return false;
}

/****************************************************************************************************/

/*
 * iAnt_controller Helper Function
 *
 * This function is used to determine if the robot is currently carrying a food item. The foodData
 * structure is protected and outside classes (e.g. iAnt_qt_user_functions) can use this function
 * to check on the status of any given controller's food item holding status.
 *
 * @return     foodData.isHoldingFoodItem     "true" if the robot is carrying a food item
 *                                            "false" if the robot is not carrying a food item
 *
 */
bool iAnt_controller::isHoldingFood() {
	/* Robot's food item holding status. */
	return iAntData.food.isHoldingFoodItem;
}

/****************************************************************************************************/
/* inherited functions from CCI_Controller used to integrate iAnt_controller into ARGoS             */
/****************************************************************************************************/

/*
 * iAnt_controller Control Step Function
 *
 * This function is the primary integration between this class and the ARGoS simulator. ARGoS will
 * call this function once per frame or tick for each currently active robot using this controller.
 * The control scheme for this controller is modeled after a state machine, but it is not a true
 * state machine do to various modifications.
 *
 */
void iAnt_controller::ControlStep() {

	/* Check for collisions and move out of the way before running the state machine. */
	if(!collisionDetection()) {

		/*
		// If the robot is carrying food, check resource density and take it back to the nest.
		if(foodData.isHoldingFoodItem && CPFA.state != CPFA::SENSE_LOCAL_RESOURCE_DENSITY) {
			CPFA.state = CPFA::SENSE_LOCAL_RESOURCE_DENSITY;
		}
		// Food not found, cancel searching for food with the given probability.
		else if(RNG->Uniform(CRange<Real>(0.0, 1.0)) < CPFA.travelProbability) {
			CPFA.state = CPFA::TRAVEL_TO_NEST;
		}
		// If the simulation is complete, transition to the stop state.
		else if(isSimulationOver) {
			CPFA.state = CPFA::STOP;
		}
		// Otherwise, set the new target location.
		else {
			CPFA.state = CPFA::SET_SEARCH_LOCATION;
		}
		*/

		/* Perform actions based on the modified state machine. */
		switch(iAntData.CPFA.state) {

			/* The robot will select a location to search for food. */
			case iAnt_data_structures::CPFA::SET_SEARCH_LOCATION:
				setSearchLocation();
				break;
			/* The robot will travel to the location it has selected to search from. */
			case iAnt_data_structures::CPFA::TRAVEL_TO_SEARCH_SITE:
				travelToSearchSite();
				break;
			/* The robot will perform an informed walk while searching for food. */
			case iAnt_data_structures::CPFA::PERFORM_INFORMED_WALK:
				performInformedWalk();
				break;
			/* The robot will perform an uninformed walk while searching for food. */
			case iAnt_data_structures::CPFA::PERFORM_UNINFORMED_WALK:
				performUninformedWalk();
				break;
			/* The robot has found food and is checking the local resource density. */
			case iAnt_data_structures::CPFA::SENSE_LOCAL_RESOURCE_DENSITY:
				senseLocalResourceDensity();
				break;
			/* The robot is traveling to the nest after finding food or giving up a search. */
			case iAnt_data_structures::CPFA::TRAVEL_TO_NEST:
				travelToNest();
				break;
			/* We should never be here! Something TERRIBLE has occurred! */
			default:
				LOGERR << "ERROR: void iAnt_controller::ControlStep()" << std::endl
					   << "STATE: [" << iAntData.CPFA.state << "] not found!"<< std::endl;

		} /* end switch */

	} /* end if */

}

/****************************************************************************************************/

/*
 * iAnt_controller Reset Function
 *
 * This function is called when the simulation user presses the reset button on the ARGoS simulator
 * GUI. Most variables in this controller are automatically set or only need to be set once except
 * for the ones in this reset list which are reset to default initialized values.
 *
 */
void iAnt_controller::Reset() {
	/* Restart the simulation with the CPFA in the REST state. */
	iAntData.CPFA.state = iAnt_data_structures::CPFA::SET_SEARCH_LOCATION;

	/* Reset food and pheromone data for this controller. */
	iAntData.food.isHoldingFoodItem = false;
	iAntData.food.hasActivePheromone = false;
}

/****************************************************************************************************/

/*
 * iAnt_controller Destroy Function
 *
 * This function is called after the user exits the simulation GUI. Any file read/write objects
 * or other objects you must delete or otherwise terminate are to be closed or deleted here. This
 * function operates similarly to a destructor function and undoes whatever was done by the call
 * to the iAnt_controller::Init(TConfigurationNode& node) function. Currently unimplemented and
 * unnecessary. This function is kept here for completeness and possible future expansion of the
 * iAnt_controller class.
 *
 */
void iAnt_controller::Destroy() { /* Not in use. */ }

/****************************************************************************************************/
/* private CPFA state machine implementation functions                                              */
/****************************************************************************************************/

/*
 * CPFA::SET_SEARCH_LOCATION State Function
 *
 * When in this state, the robot will determine the next area to move to before it begins a search
 * for food items. The robot will pick a random heading angle from 0 to 2 PI and head in that
 * direction until the travel probability causes the robot to switch to searching mode.
 *
 */
void iAnt_controller::setSearchLocation() {

	/* Always make sure the robot isn't carrying food first. */
	if(iAntData.food.isHoldingFoodItem) {
		iAntData.CPFA.state = iAnt_data_structures::CPFA::SENSE_LOCAL_RESOURCE_DENSITY;
	}

	else {
		/* Set the target vector to a length that will always reach outside of the arena bounds.
		 *
		 * Recall that the arena's coordinate domain is [-arenaSize.GetX()/2.0, arenaSize.GetX()/2.0]
		 * and that its coordinate range is [-arenaSize.GetY()/2.0, arenaSize.GetY()/2.0].
		 *
		 * In other words, head straight out until we run into the wall OR the CPFA's search
		 * probability causes the robot to change its direction and search a location for food. */
		Real length(iAntData.navigation.arenaSize.Length());

		/* Obtain a heading angle from a uniform distribution between 0 and 2 PI. */
		CRadians angle(RNG->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI)));

		/* Set the navigation target to the position given by the length and angle parameters
		 * and do so in relation to the robot's position and NOT the origin of the arena. */
		iAntData.navigation.target = getVectorToPosition(CVector2(length, angle) + iAntData.navigation.position);

		/* Search location has been set, change to travel state. */
		iAntData.CPFA.state = iAnt_data_structures::CPFA::TRAVEL_TO_SEARCH_SITE;
	}
}

/****************************************************************************************************/

/*
 *
 */
void iAnt_controller::travelToSearchSite() {

	if(iAntData.food.isHoldingFoodItem) {
		iAntData.CPFA.state = iAnt_data_structures::CPFA::SENSE_LOCAL_RESOURCE_DENSITY;
	}

	else if(((iAntData.navigation.position - iAntData.navigation.target).SquareLength() < iAntData.navigation.distanceTolerance) ||
	   RNG->Uniform(CRange<Real>(0.0, 1.0)) < iAntData.CPFA.searchProbability) {
		// TO-DO add logic to determine if we perform an uninformed or informed walk in this state!
		iAntData.CPFA.state = iAnt_data_structures::CPFA::PERFORM_UNINFORMED_WALK;
	}

	else {
		setWheelSpeed(getVectorToPosition(iAntData.navigation.target));
	}
}

/****************************************************************************************************/

/*
 *
 */
void iAnt_controller::performInformedWalk() {
	// TO DO, currently unimplemented
}

/****************************************************************************************************/

/*
 *
 */
void iAnt_controller::performUninformedWalk() {

	if(iAntData.food.isHoldingFoodItem) {
		iAntData.CPFA.state = iAnt_data_structures::CPFA::SENSE_LOCAL_RESOURCE_DENSITY;
	}

	else if((iAntData.navigation.position - iAntData.navigation.target).SquareLength() < iAntData.navigation.distanceTolerance) {
		/* Get a random rotation angle and then add it to the getVectorToLight angle. This serves the functionality
		 * of a compass and causes the rotation to be relative to the robot's current direction. */
   		CRadians rotation(RNG->Gaussian(iAntData.CPFA.uninformedSearchCorrelation.GetValue())),
    			 angle(getVectorToLight().Angle().SignedNormalize() + rotation);

   		/* Move from the current position "searchStepSize" distance away after turning "angle" degrees/radians etc. */
   		iAntData.navigation.target = (CVector2(iAntData.navigation.searchStepSize, angle) + iAntData.navigation.position);

   		/* Bounds check: make sure the new position is not outside of the available arena space if the robot is near the edge. */
    	if(iAntData.navigation.target.GetX() > iAntData.navigation.forageRangeX.GetMax()) {
    		iAntData.navigation.target.SetX(iAntData.navigation.forageRangeX.GetMax());
    	} else if(iAntData.navigation.target.GetX() < iAntData.navigation.forageRangeX.GetMin()) {
    		iAntData.navigation.target.SetX(iAntData.navigation.forageRangeX.GetMin());
       	}

   		if(iAntData.navigation.target.GetY() > iAntData.navigation.forageRangeY.GetMax()) {
   			iAntData.navigation.target.SetY(iAntData.navigation.forageRangeY.GetMax());
        } else if(iAntData.navigation.target.GetY() < iAntData.navigation.forageRangeY.GetMin()) {
        	iAntData.navigation.target.SetY(iAntData.navigation.forageRangeY.GetMin());
        }
	}

	else {
        setWheelSpeed(getVectorToPosition(iAntData.navigation.target));
	}

}

/****************************************************************************************************/

/*
 *
 */
void iAnt_controller::senseLocalResourceDensity()
{
	// This may be inaccurate, would potentially need to make this set only if the resourceCount is higher
	// then a certain threshold...
	iAntData.food.pheromonePosition = iAntData.navigation.position;
	iAntData.food.resourceCount = -1; // DON'T count the food item robot just found

	for(size_t i = 0; i < iAntData.food.foodPositions.size(); i++) {
		if((iAntData.navigation.position - iAntData.food.foodPositions[i]).SquareLength() < iAntData.navigation.searchRadiusSquared) {
			iAntData.food.resourceCount++;
		}
	}

	iAntData.CPFA.state = iAnt_data_structures::CPFA::TRAVEL_TO_NEST;
}

/****************************************************************************************************/

/*
 *
 */
void iAnt_controller::travelToNest() {
	/* If the robot has arrived inside the nest zone, transition to the "start" state. */
	if(isInTheNest()) {
		iAntData.CPFA.state = iAnt_data_structures::CPFA::SET_SEARCH_LOCATION;
    }

	/* Otherwise, set the robot's heading toward the nest zone and move towards it. */
    else {
    	setWheelSpeed(getVectorToLight());
    }
}

/****************************************************************************************************/
/* private helper functions used for motion control and navigation                                  */
/****************************************************************************************************/

/*
 *
 */
bool iAnt_controller::collisionDetection() {
	const CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();
	CVector2 accumulator;

	for(size_t i = 0; i < proximityReadings.size(); ++i) {
        accumulator += CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
	}

	accumulator /= proximityReadings.size();
	CRadians angle = accumulator.Angle();

	if(iAntData.navigation.angleTolerance.WithinMinBoundIncludedMaxBoundIncluded(angle) &&
	   accumulator.Length() < iAntData.navigation.distanceTolerance) {
		steeringActuator->SetLinearVelocity(iAntData.navigation.maxSpeed, iAntData.navigation.maxSpeed);
		return false; // collision not detected
	} else {
		if(angle.GetValue() > 0.0) {
			steeringActuator->SetLinearVelocity(iAntData.navigation.maxSpeed, 0.0);
		} else {
			steeringActuator->SetLinearVelocity(0.0, iAntData.navigation.maxSpeed);
		}
		return true; // collision detected
	}
}

/*
 *
 */
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

/*
 *
 */
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

/*
 *
 */
CVector2 iAnt_controller::getVectorToLight() {
	const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
	CVector2 accumulator;

	for(size_t i = 0; i < readings.size(); ++i) {
	    accumulator += CVector2(readings[i].Value, readings[i].Angle);
	}

	return accumulator;
}

/*
 *
 */
CVector2 iAnt_controller::getVectorToPosition(const CVector2& targetPosition) {
    const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
    CVector2 accumulator;
    // we will construct a triangle using these points: A, B, C
    CVector2 A(iAntData.navigation.nestPosition), B(iAntData.navigation.position), C(targetPosition);
    CRadians rotationTowardsTarget(lawOfCosines(A, B, C));

    for(size_t i = 0; i < readings.size(); ++i) {
        accumulator += CVector2(readings[i].Value, readings[i].Angle + rotationTowardsTarget);
    }

    return accumulator;
}

/*
 *
 */
void iAnt_controller::setWheelSpeed(const CVector2& heading) {
	CRadians headingAngle = heading.Angle().SignedNormalize();
	enum turnStatus { NO_TURN = 0, TURN = 1 } turnStatus;

   if(Abs(headingAngle) < iAntData.navigation.angleTolerance.GetMax()) {
	   turnStatus = NO_TURN;
   } else if(Abs(headingAngle) > iAntData.navigation.angleTolerance.GetMax()) {
	   turnStatus = TURN;
   }

   /* Wheel speeds based on current turning state */
   Real speed1, speed2;

   switch(turnStatus) {
      case NO_TURN: {
         /* Just go straight */
         speed1 = iAntData.navigation.maxSpeed;
         speed2 = iAntData.navigation.maxSpeed;
         break;
      }
      case TURN: {
         /* Opposite wheel speeds */
         speed1 = -iAntData.navigation.maxSpeed;
         speed2 =  iAntData.navigation.maxSpeed;
         break;
      }
   }

   Real leftWheelSpeed, rightWheelSpeed;

   if(headingAngle > CRadians::ZERO) {
      /* Turn Left */
      leftWheelSpeed  = speed1;
      rightWheelSpeed = speed2;
   } else {
      /* Turn Right */
      leftWheelSpeed  = speed2;
      rightWheelSpeed = speed1;
   }

   steeringActuator->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);
}

REGISTER_CONTROLLER(iAnt_controller, "iAnt_controller")

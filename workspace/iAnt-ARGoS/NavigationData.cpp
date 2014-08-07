#include "NavigationData.h"

NavigationData::NavigationData() :
	steeringActuator(NULL), // controls the robot's motor speeds
	proximitySensor(NULL),  // detects nearby objects to prevent collision
	groundSensor(NULL),     // detects food items & nest (color changes)
	lightSensor(NULL),      // detects nest-light for navigation control
	distanceTolerance(0.0),
	angleTolerance(-CRadians::ZERO, CRadians::ZERO),
	arenaSize(0.0, 0.0),
	forageRangeX(-0.0, 0.0),
	forageRangeY(-0.0, 0.0),
	position(0.0, 0.0),
	target(0.0, 0.0),
	nestPosition(0.0, 0.0),
	nestRadiusSquared(0.0),
	searchStepSize(0.0),
	searchRadiusSquared(0.0),
	maxSpeed(0.0)
{}

NavigationData::~NavigationData() {
	// TODO Auto-generated destructor stub
}
void NavigationData::SetSensorsAndActuators(CCI_DifferentialSteeringActuator *steeringActuator,
		                                    CCI_FootBotProximitySensor       *proximitySensor,
		                    	            CCI_FootBotMotorGroundSensor     *groundSensor,
		                    	            CCI_FootBotLightSensor           *lightSensor) {
	this->steeringActuator = steeringActuator;
	this->proximitySensor  = proximitySensor;
	this->groundSensor     = groundSensor;
	this->lightSensor      = lightSensor;
}


bool NavigationData::CollisionDetection() {
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
}

/*
 *
 */
CRadians NavigationData::LawOfCosines(CVector2& A, CVector2& B, CVector2& C) {
    // the length of each side of the calculated triangle
    Real a(sqrt(((B.GetX() - A.GetX()) * (B.GetX() - A.GetX())) + ((B.GetY() - A.GetY()) * (B.GetY() - A.GetY())))),
         b(sqrt(((B.GetX() - C.GetX()) * (B.GetX() - C.GetX())) + ((B.GetY() - C.GetY()) * (B.GetY() - C.GetY())))),
         c(sqrt(((A.GetX() - C.GetX()) * (A.GetX() - C.GetX())) + ((A.GetY() - C.GetY()) * (A.GetY() - C.GetY()))));

    // determine whether we must add or subtract the rotation angle
    Real sign(GetSignOfRotationAngle(A, B, C));

    // formula for the law of cosines
    return CRadians(sign * acos(((a * a) + (b * b) - (c * c)) / (2.0 * a * b)));
}

/*
 *
 */
Real NavigationData::GetSignOfRotationAngle(CVector2& A, CVector2& B, CVector2& C) {
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
CVector2 NavigationData::GetVectorToLight() {
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
CVector2 NavigationData::GetVectorToPosition(const CVector2& targetPosition) {
    const CCI_FootBotLightSensor::TReadings& readings = lightSensor->GetReadings();
    CVector2 accumulator;
    // we will construct a triangle using these points: A, B, C
    CVector2 A(nestPosition), B(position), C(targetPosition);
    CRadians rotationTowardsTarget(LawOfCosines(A, B, C));

    for(size_t i = 0; i < readings.size(); ++i) {
        accumulator += CVector2(readings[i].Value, readings[i].Angle + rotationTowardsTarget);
    }

    return accumulator;
}

/*
 *
 */
void NavigationData::SetWheelSpeed(const CVector2& heading) {
	CRadians headingAngle = heading.Angle().SignedNormalize();
	enum turnStatus { NO_TURN = 0, TURN = 1 } turnStatus;

   if(Abs(headingAngle) < angleTolerance.GetMax()) {
	   turnStatus = NO_TURN;
   } else if(Abs(headingAngle) > angleTolerance.GetMax()) {
	   turnStatus = TURN;
   }

   /* Wheel speeds based on current turning state */
   Real speed1, speed2;

   switch(turnStatus) {
      case NO_TURN: {
         /* Just go straight */
         speed1 = maxSpeed;
         speed2 = maxSpeed;
         break;
      }
      case TURN: {
         /* Opposite wheel speeds */
         speed1 = -maxSpeed;
         speed2 =  maxSpeed;
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

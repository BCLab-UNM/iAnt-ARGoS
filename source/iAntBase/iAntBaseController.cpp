#include "iAntBaseController.h"

/**
 * Constructor for the iAntBaseController. Several important variables are defined here.
 * <p>
 * TODO: update xml configuration file to allow these to be adjusted from configuration without recompiling.
 */
iAntBaseController::iAntBaseController() :
    LF(argos::CSimulator::GetInstance().GetLoopFunctions()),
    WaitTime(0),
    TargetDistanceTolerance(0.03),
    SearchStepSize(0.08),
    RobotForwardSpeed(16.0),
    RobotRotationSpeed(16.0),
    TicksToWaitWhileMoving(0.0),
    CurrentMovementState(STOP)
{
    // calculate the forage range and compensate for the robot's radius of 0.085m
    argos::CVector3 ArenaSize = LF.GetSpace().GetArenaSize();
    argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
    argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);
    GoStraightAngleRangeInDegrees.Set(-37.5, 37.5);
}

argos::CRadians iAntBaseController::GetHeading() {
    /* in ARGoS, the robot's orientation is represented by a quaternion */
    const argos::CCI_PositioningSensor::SReading& readings = compassSensor->GetReading();
    argos::CQuaternion orientation = readings.Orientation;

    /* convert the quaternion to euler angles */
    argos::CRadians z_angle, y_angle, x_angle;
    orientation.ToEulerAngles(z_angle, y_angle, x_angle);

    /* the angle to the z-axis represents the compass heading */
    return z_angle;
}

argos::CVector2 iAntBaseController::GetPosition() {
    /* the robot's compass sensor gives us a 3D position */
    argos::CVector3 position3D = compassSensor->GetReading().Position;
    /* return the 2D position components of the compass sensor reading */
    return argos::CVector2(position3D.GetX(), position3D.GetY());
}

argos::CVector2 iAntBaseController::GetTarget() {
    return TargetPosition;
}

void iAntBaseController::SetTarget(argos::CVector2 t) {

    argos::Real x(t.GetX()), y(t.GetY());

    if(x > ForageRangeX.GetMax()) x = ForageRangeX.GetMax();
    else if(x < ForageRangeX.GetMin()) x = ForageRangeX.GetMin();

    if(y > ForageRangeY.GetMax()) y = ForageRangeY.GetMax();
    else if(y < ForageRangeY.GetMin()) y = ForageRangeY.GetMin();

    TargetPosition = argos::CVector2(x, y);

}

void iAntBaseController::SetStartPosition(argos::CVector3 sp) {
    StartPosition = sp;
}

argos::CVector3 iAntBaseController::GetStartPosition() {
    return StartPosition;
}

size_t iAntBaseController::GetMovementState() {
    return CurrentMovementState;
}

void iAntBaseController::SetNextMovement() {

    if(MovementStack.size() == 0 && CurrentMovementState == STOP) {

        argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();
        argos::CRadians headingToTarget = (TargetPosition - GetPosition()).Angle();
        argos::CRadians headingToTargetError = (GetHeading() - headingToTarget).SignedNormalize();
        argos::CRadians zero(0.0);

        if(distanceToTarget > TargetDistanceTolerance) {
            PushMovement(FORWARD, distanceToTarget);
            if(headingToTargetError < zero) {
                PushMovement(LEFT, -ToDegrees(headingToTargetError).GetValue());
            } else {
                PushMovement(RIGHT, ToDegrees(headingToTargetError).GetValue());
            }
        } else {
            PushMovement(STOP, 0.0);
        }
    } else {
        PopMovement();
    }

}

void iAntBaseController::SetTargetAngleDistance(argos::Real newAngleToTurnInDegrees) {
    // s = arc_length = robot_radius * turning_angle
    // NOTE: the footbot robot has a radius of 0.085 m... or 8.5 cm...
    // adjusting with + 0.02 m, or + 2 cm, increases accuracy...
    argos::Real s = 0.105 * newAngleToTurnInDegrees;
    TicksToWaitWhileMoving = std::ceil((SimulationTicksPerSecond() * s) / RobotRotationSpeed);
}

void iAntBaseController::SetTargetTravelDistance(argos::Real newTargetDistance) {
    // convert meters into cm
    argos::Real d = newTargetDistance * 100.0;
    TicksToWaitWhileMoving = std::ceil((SimulationTicksPerSecond() * d) / RobotForwardSpeed);
}

void iAntBaseController::SetLeftTurn(argos::Real newAngleToTurnInDegrees) {
    if(newAngleToTurnInDegrees > 0.0) {
        SetTargetAngleDistance(newAngleToTurnInDegrees);
        CurrentMovementState = LEFT;
    } else if(newAngleToTurnInDegrees < 0.0) {
        SetTargetAngleDistance(-newAngleToTurnInDegrees);
        CurrentMovementState = RIGHT;  
    } else {
        Stop();
    }
}

void iAntBaseController::SetRightTurn(argos::Real newAngleToTurnInDegrees) {
    if(newAngleToTurnInDegrees > 0.0) {
        SetTargetAngleDistance(newAngleToTurnInDegrees);
        CurrentMovementState = RIGHT;
    } else if(newAngleToTurnInDegrees < 0.0) {
        SetTargetAngleDistance(-newAngleToTurnInDegrees);
        CurrentMovementState = LEFT;
    } else {
        Stop();
    }
}

void iAntBaseController::SetMoveForward(argos::Real newTargetDistance) {
    if(newTargetDistance > 0.0) {
        SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = FORWARD;
    } else if(newTargetDistance < 0.0) {
        SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = BACK;
    } else {
        Stop();
    }
}

void iAntBaseController::SetMoveBack(argos::Real newTargetDistance) {
    if(newTargetDistance > 0.0) {
        SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = BACK;
    } else if(newTargetDistance < 0.0) {
        SetTargetTravelDistance(newTargetDistance);
        CurrentMovementState = FORWARD;
    } else {
        Stop();
    }
}

void iAntBaseController::PushMovement(size_t moveType, argos::Real moveSize) {
    Movement newMove = { moveType, moveSize };
    MovementStack.push(newMove);
}

void iAntBaseController::PopMovement() {
    Movement nextMove = MovementStack.top();
    MovementStack.pop();

    switch(nextMove.type) {

        case STOP: {
            Stop();
            break;
        }

        case LEFT: {
            SetLeftTurn(nextMove.magnitude);
            break;
        }

        case RIGHT: {
            SetRightTurn(nextMove.magnitude);
            break;
        }

        case FORWARD: {
            SetMoveForward(nextMove.magnitude);
            break;
        }

        case BACK: {
            SetMoveBack(nextMove.magnitude);
            break;
        }

    }
}

bool iAntBaseController::CollisionDetection() {

    argos::CVector2 collisionVector = GetCollisionVector();
    argos::Real collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
    bool isCollisionDetected = false;

    // argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;

    if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
       && collisionVector.Length() > 0.0) {

        Stop();
        isCollisionDetected = true;
        while(MovementStack.size() > 0) MovementStack.pop();

        PushMovement(FORWARD, SearchStepSize);

        if(collisionAngle <= 0.0) {
            SetLeftTurn(37.5 - collisionAngle);
        } else {
            SetRightTurn(37.5 + collisionAngle);
        }
    }

    return isCollisionDetected;

}

argos::CVector2 iAntBaseController::GetCollisionVector() {
    /* Get readings from proximity sensor */
    const argos::CCI_FootBotProximitySensor::TReadings& proximityReadings = proximitySensor->GetReadings();

    /* Sum them together */
    argos::CVector2 collisionVector;

    for(size_t i = 0; i < proximityReadings.size(); ++i) {
        collisionVector += argos::CVector2(proximityReadings[i].Value, proximityReadings[i].Angle);
    }

    collisionVector /= proximityReadings.size();

    return collisionVector;
}

void iAntBaseController::Stop() {
    SetTargetTravelDistance(0.0);
    SetTargetAngleDistance(0.0);
    TicksToWaitWhileMoving = 0.0;
    CurrentMovementState = STOP;
}

void iAntBaseController::Move() {

    if(Wait() == true) return;

    CollisionDetection();

    /* move based on the movement state flag */
    switch(CurrentMovementState) {

        /* stop movement */
        case STOP: {
            wheelActuator->SetLinearVelocity(0.0, 0.0);
            SetNextMovement();
            break;
        }

        /* turn left until the robot is facing an angle inside of the TargetAngleTolerance */
        case LEFT: {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {
                wheelActuator->SetLinearVelocity(-RobotRotationSpeed, RobotRotationSpeed);
            }
            break;
        }

        /* turn right until the robot is facing an angle inside of the TargetAngleTolerance */
        case RIGHT: {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {
                wheelActuator->SetLinearVelocity(RobotRotationSpeed, -RobotRotationSpeed);
            }
            break;
        }

        /* move forward until the robot has traveled the specified distance */
        case FORWARD: {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {
                wheelActuator->SetLinearVelocity(RobotForwardSpeed, RobotForwardSpeed);             
            }
            break;
        }

        /* move backward until the robot has traveled the specified distance */
        case BACK: {
            if((TicksToWaitWhileMoving--) <= 0.0) {
                Stop();
            } else {
                wheelActuator->SetLinearVelocity(-RobotForwardSpeed, -RobotForwardSpeed);
            }
            break;
        }
    }
}

bool iAntBaseController::Wait() {

    bool wait = false;

    if(WaitTime > 0) {
        WaitTime--;
        wait = true;
    }

    return wait;
}

void iAntBaseController::Wait(size_t wait_time_in_seconds) {

    WaitTime += (wait_time_in_seconds * SimulationTicksPerSecond());

}

size_t iAntBaseController::SimulationTick() {
    return LF.GetSpace().GetSimulationClock();
}

size_t iAntBaseController::SimulationTicksPerSecond() {
    return LF.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick();
}

argos::Real iAntBaseController::SimulationSecondsPerTick() {
    return LF.GetSimulator().GetPhysicsEngine("default").GetSimulationClockTick();
}

argos::Real iAntBaseController::SimulationTimeInSeconds() {
    return (argos::Real)(SimulationTick()) * SimulationSecondsPerTick();
}

bool iAntBaseController::IsAtTarget() {
    argos::Real distanceToTarget = (TargetPosition - GetPosition()).Length();
    return (distanceToTarget < TargetDistanceTolerance) ? (true) : (false);
}

//REGISTER_CONTROLLER(iAntBaseController, "iAntBaseController")
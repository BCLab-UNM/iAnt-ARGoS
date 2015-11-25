#ifndef IANTBASECONTROLLER_H
#define IANTBASECONTROLLER_H

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/simulator/loop_functions.h>
#include <cmath>
#include <stack>

/**
 * iAntBaseController
 * @author Antonio Griego
 */
class iAntBaseController : public argos::CCI_Controller {

    public:

        iAntBaseController();

        /* iAnt navigation functions */
        argos::CRadians GetHeading();
        argos::CVector2 GetPosition();
        argos::CVector2 GetTarget();
        void SetTarget(argos::CVector2 t);
        void SetStartPosition(argos::CVector3 sp);
        argos::CVector3 GetStartPosition();
        size_t GetMovementState();

        void Stop();
        void Move();
        bool Wait();
        void Wait(size_t wait_time_in_seconds);

        /* iAnt time calculation functions */
        size_t SimulationTick();
        size_t SimulationTicksPerSecond();
        argos::Real SimulationSecondsPerTick();
        argos::Real SimulationTimeInSeconds();

        bool IsAtTarget();

    protected:

        size_t WaitTime;

        argos::Real TargetDistanceTolerance;
        argos::Real SearchStepSize;

        argos::CRange<argos::Real> ForageRangeX;
        argos::CRange<argos::Real> ForageRangeY;
        argos::CRange<argos::Real> GoStraightAngleRangeInDegrees;

    	// iAnt base controller movement parameters
    	argos::Real RobotForwardSpeed;
    	argos::Real RobotRotationSpeed;
    	argos::Real TicksToWaitWhileMoving;

        // foot-bot components: sensors and actuators
        argos::CCI_PositioningSensor* compassSensor;
        argos::CCI_DifferentialSteeringActuator* wheelActuator;
        argos::CCI_FootBotProximitySensor* proximitySensor;

        // controller state variables
        enum MovementState {
            STOP    = 0,
            LEFT    = 1,
            RIGHT   = 2,
            FORWARD = 3,
            BACK    = 4
        } CurrentMovementState;

    private:

        argos::CLoopFunctions& LF;

        argos::CVector3 StartPosition;
        argos::CVector2 TargetPosition;

        /* movement definition variables */
        struct Movement {
            size_t type;
            argos::Real magnitude;
        };

        //MovementState CurrentMovementState;
        std::stack<Movement> MovementStack;

        /* private navigation helper functions */
        void SetNextMovement();
        void SetTargetAngleDistance(argos::Real newAngleToTurnInDegrees);
        void SetTargetTravelDistance(argos::Real newTargetDistance);
        void SetLeftTurn(argos::Real newTargetAngle);
        void SetRightTurn(argos::Real newTargetAngle);
        void SetMoveForward(argos::Real newTargetDistance);
        void SetMoveBack(argos::Real newTargetDistance);
        void PushMovement(size_t moveType, argos::Real moveSize);
        void PopMovement();

        /* collision detection functions */
        bool CollisionDetection();
        argos::CVector2 GetCollisionVector();

};

#endif /* IANTBASECONTROLLER_H */
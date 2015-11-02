#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <source/iAnt_loop_functions.h>
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;
using namespace std;

class iAnt_loop_functions;

/*****
 * The brain of each iAnt robot which implements the Central Place Foraging Algorithm (CPFA).
 *****/
class iAnt_controller : public CCI_Controller, public CLoopFunctions {

    public:

        /* constructor and destructor */
        iAnt_controller();
        virtual ~iAnt_controller() {}

        /* CCI_Controller Inherited Functions */
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();

        /* public helper functions */
        bool     IsHoldingFood();
        bool     IsInTheNest();
        CVector2 GetPosition();
        CVector3 GetStartPosition();
        bool     Wait();
        void     Wait(size_t wait_time_in_seconds);

        //bool     Turn();
        //bool     Move();

    private:

        /* foot-bot components: sensors and actuators */
        CCI_PositioningSensor*            compass;
        CCI_DifferentialSteeringActuator* wheels;
        CCI_FootBotProximitySensor*       proximitySensor;

        /* iAnt controller parameters */
        size_t           maxTrailSize;
        Real             distanceTolerance;
        Real             searchStepSize;
        Real             robotForwardSpeed;
        Real             robotRotationSpeed;
        CRange<CRadians> angleToleranceInRadians;

        /* robot internal variables & statistics */
        CRandom::CRNG*       RNG;
        iAnt_loop_functions& loopFunctions;

        CVector3             startPosition;
        CVector2             targetPosition;
        CVector2             targetWaypoint;
        CVector2             fidelityPosition;

        vector<CVector2>     trailToShare;
        vector<CVector2>     trailToFollow;
        vector<CRay3>        myTrail;

        bool   isHoldingFood;
        bool   isInformed;
        bool   isUsingSiteFidelity;
        bool   isGivingUpSearch;

        size_t searchTime;
        size_t waitTime;

        size_t collisionDelay;
        size_t resourceDensity;

    private:

        /* iAnt CPFA state variable */
        enum CPFA { DEPARTING = 0, SEARCHING = 1, RETURNING = 2 } CPFA;

        /* iAnt CPFA state functions */
        void Departing();
        void Searching();
        void Returning();

        /* CPFA helper functions */
        void SetHoldingFood();
        void SetRandomSearchLocation();
        void SetLocalResourceDensity();
        void SetFidelityList(CVector2 newFidelity);
        void SetFidelityList();
        bool SetTargetPheromone();

        /* mathematical helper functions */
        Real GetExponentialDecay(Real value, Real time, Real lambda);
        Real GetBound(Real x, Real min, Real max);
        Real GetPoissonCDF(Real k, Real lambda);

        /* navigation helper functions */
        CRadians GetHeading();
        CRadians GetCollisionHeading();
        bool     IsCollisionDetected();
        void     ApproachTheTarget();
        void     SetTargetInBounds(CVector2 newTarget);


        /* graphics helper functions */
        void UpdateTargetRayList();
};

#endif /* IANT_CONTROLLER_H_ */

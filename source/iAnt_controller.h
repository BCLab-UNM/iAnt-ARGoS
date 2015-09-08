#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <source/iAnt_loop_functions.h>

using namespace argos;
using namespace std;

class iAnt_loop_functions;

/*****
 * The brain of each iAnt robot which implements the Central Place Foraging Algorithm (CPFA).
 *****/
class iAnt_controller : public CCI_Controller {

    public:

        /* constructor and destructor */
        iAnt_controller();
        virtual ~iAnt_controller() {}

        /* CCI_Controller Inherited Functions */
        virtual void Init(TConfigurationNode& node);
        virtual void ControlStep();
        virtual void Reset();

        /* public helper functions */
        bool     IsHoldingFood() { return isHoldingFood; }
        bool     IsInTheNest();
        void     SetLoopFunctions(iAnt_loop_functions* lf) { loopFunctions = lf; }
        CVector2 GetPosition();
        CVector3 GetStartPosition() { return startPosition; }
        CVector2 GetTarget();

    private:

        /* foot-bot components: sensors and actuators */
        CCI_PositioningSensor*            compass;
        CCI_DifferentialSteeringActuator* motorActuator;
        CCI_FootBotProximitySensor*       proximitySensor;

        /* iAnt controller parameters */
        Real             distanceTolerance;
        Real             searchStepSize;
        Real             robotForwardSpeed;
        Real             robotRotationSpeed;
        CRange<CRadians> angleToleranceInRadians;

        /* robot internal variables & statistics */
        CRandom::CRNG*       RNG;
        iAnt_loop_functions* loopFunctions;
        CVector3             startPosition;
        CVector2             target;
        CVector2             fidelity;
        vector<CVector2>     trailToShare;
        vector<CVector2>     trailToFollow;

        bool   isHoldingFood;
        bool   isInformed;
        bool   isUsingSiteFidelity;
        size_t searchTime;
        size_t waitTime;
        size_t collisionDelay;
        size_t resourceDensity;

    private:

        /* iAnt CPFA state variable */
        enum CPFA { DEPARTING, SEARCHING, RETURNING, SHUTDOWN } CPFA;

        /* iAnt CPFA state functions */
        void departing();
        void searching();
        void returning();
        void shutdown();

        /* CPFA helper functions */
        void SetHoldingFood();
        void SetRandomSearchLocation();
        void SetLocalResourceDensity();
        void SetFidelityList(CVector2 newFidelity);
        void SetFidelityList();
        bool SetTargetPheromone();
        Real GetExponentialDecay(Real value, Real time, Real lambda);
        Real GetBound(Real x, Real min, Real max);
        Real GetPoissonCDF(Real k, Real lambda);

        /* navigation helper functions */
        CRadians GetHeading();
        bool     IsCollisionDetected();
        void     ApproachTheTarget();
        void     SetTargetInBounds(CVector2 newTarget);

};

#endif /* IANT_CONTROLLER_H_ */

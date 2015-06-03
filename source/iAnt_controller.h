#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

#include "iAnt_data.h"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

using namespace argos;
using namespace std;

/*****
 * The brains of the iAnt robot. This controller object is a component of
 * each robot that is placed in the arena for experiments. The implementation
 * of the iAnt Central Place Foraging Algorithm (CPFA) is in this code.
 *****/
class iAnt_controller : public CCI_Controller {

    public:

        /* constructor and destructor */
        iAnt_controller();
        ~iAnt_controller() {}

        /* CCI_Controller Inherited Functions */
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();

        /* public helper functions */
        bool       IsHoldingFood();
        bool       IsInTheNest();
        void       SetData(iAnt_data* dataPointer);
        iAnt_data* GetData();
        CVector2   GetPosition();
        CVector2   GetTarget();

    private:

        /* foot-bot components: sensors and actuators */
        CCI_PositioningSensor*            compass;
        CCI_DifferentialSteeringActuator* motorActuator;
        CCI_FootBotProximitySensor*       proximitySensor;

        /* iAnt controller parameters */
        Real             DistanceTolerance;
        Real             SearchStepSize;
        Real             RobotForwardSpeed;
        Real             RobotRotationSpeed;
        CRange<CRadians> AngleToleranceInRadians;

        /* robot internal variables & statistics */
        CRandom::CRNG*   RNG;
        iAnt_data*       data;
        CVector2         target;
        CVector2         fidelity;
        vector<CVector2> trailToShare;
        vector<CVector2> trailToFollow;
        bool             isHoldingFood;
        bool             isInformed;
        bool             isUsingSiteFidelity;
        size_t           searchTime;
        size_t           waitTime;
        size_t           collisionDelay;
        size_t           resourceDensity;

        /* iAnt CPFA state variable */
        enum CPFA { INACTIVE, DEPARTING, SEARCHING, RETURNING, SHUTDOWN } CPFA;

        /* iAnt CPFA state functions */
        void inactive();
        void departing();
        void searching();
        void returning();
        void shutdown();

        /* CPFA helper functions */
        void SetHoldingFood();
        void SetRandomSearchLocation();
        void SetLocalResourceDensity();
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

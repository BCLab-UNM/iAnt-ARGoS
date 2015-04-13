#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

#include "iAnt_pheromone.h"
#include <loop_functions/iAnt_loop_functions.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;
using namespace std;

class iAnt_loop_functions;

class iAnt_controller : public CCI_Controller {

  public:

    /***************************************************************************
    * Constructor And Destructor Functions
    ***************************************************************************/
    iAnt_controller();
    ~iAnt_controller() {}

    /***************************************************************************
    * Robot Status Check Functions
    ***************************************************************************/
    bool   IsInTheNest();
    bool   IsFindingFood();
    bool   IsHoldingFood();
    bool   IsInformed();
    string Get_CPFA_ID();

    /***************************************************************************
    * Robot Action Functions
    ***************************************************************************/
    void PickupFood();
    void DropOffFood();
    void SetInformed(bool i);

    /***************************************************************************
    * Robot Setter Functions
    ***************************************************************************/
    void SetLoopFunctions(iAnt_loop_functions *lf);
    void SetFoodPositions(vector<CVector2> fp);
    void SetPheromonePositions(vector<CVector2> pp);
    void SetFidelityPositions(vector<CVector2> fp);
    void SetTime(size_t t);
    void SetFramesPerSecond(size_t fps);
    void SetNestPosition(CVector2 np);
    void SetNestRadiusSquared(Real nrs);
    void SetFoodRadiusSquared(Real frs);
    void SetTargetPheromone(iAnt_pheromone tp);
    void SetForageRange(CRange<Real> x, CRange<Real> y);

    /***************************************************************************
    * Robot Getter Functions
    ***************************************************************************/
    CVector2         GetPosition();
    CVector2         GetFidelityPosition();
    CVector2         GetNestPosition();
    Real             GetNestRadius();
    Real             GetFoodRadius();
    vector<CVector2> GetFoodPositions();
    vector<CVector2> GetPheromonePositions();
    vector<CVector2> GetFidelityPositions();
    iAnt_pheromone   GetSharedPheromone();
    iAnt_loop_functions* GetLoopFunctions();

    /***************************************************************************
    * CCI_Controller Inherited Functions
    ***************************************************************************/
    void Init(TConfigurationNode& node);
    void ControlStep();
    void Reset();

  private:

    /***************************************************************************
    * Actuators, Sensors, and ARGoS Classes
    ***************************************************************************/
    CCI_DifferentialSteeringActuator *steeringActuator;
    CCI_FootBotProximitySensor       *proximitySensor;
    CCI_PositioningSensor            *compassSensor;
    iAnt_loop_functions              *loopFunctions;

    /***************************************************************************
    * Robot Status Variables
    ***************************************************************************/
    bool   isHoldingFood;
    bool   isInformed;
    size_t resourceDensity;
    size_t collisionDelay;
    size_t simTime;
    size_t searchTime;
    size_t framesPerSecond;
    Real   maxRobotSpeed;

    /***************************************************************************
    * Robot Position Variables
    ***************************************************************************/
    CVector2         nestPosition;
    CVector2         targetPosition;
    CVector2         fidelityPosition;
    vector<CVector2> foodPositions;
    vector<CVector2> pheromonePositions;
    vector<CVector2> fidelityPositions;
    iAnt_pheromone   targetPheromone; // pheromone I'm following right now
    iAnt_pheromone   sharedPheromone; // pheromone I'm sharing with the colony

    /***************************************************************************
    * Robot Navigation Variables
    ***************************************************************************/
    CRange<Real>     forageRangeX;
    CRange<Real>     forageRangeY;
    CRange<CRadians> angleTolerance;
    Real             nestRadiusSquared;
    Real             foodRadiusSquared;
    Real             searchRadiusSquared;
    Real             distanceTolerance;
    CRandom::CRNG*   RNG;

    /***************************************************************************
    * CPFA Variables
    ***************************************************************************/
    Real     travelGiveupProbability;
    Real     searchGiveupProbability;
    Real     searchStepSize;
    Real     informedSearchDecay;
    Real     siteFidelityRate;
    Real     pheromoneRate;
    Real     pheromoneDecayRate;
    CRadians uninformedSearchCorrelation;

    enum CPFA { INACTIVE, DEPARTING, SEARCHING, RETURNING, SHUTDOWN } CPFA;

    /***************************************************************************
    * CPFA Functions
    ***************************************************************************/
    void inactive();
    void departing();
    void searching();
    void returning();
    void shutdown();

    /***************************************************************************
    * CPFA And Navigation Helper Functions
    ***************************************************************************/
    bool     IsCollisionDetected();
    void     SetLocalResourceDensity();
    void     SetRandomSearchLocation();
    void     SetRobotMotors();
    void     SetTargetPosition(CVector2 p);
    CVector2 SetPositionInBounds(CVector2 p);
    CRadians GetRobotHeading();
    Real     GetExponentialDecay(Real quantity, Real time, Real lambda);
    Real     GetBound(Real x, Real min, Real max);
    Real     GetPoissonCDF(Real k, Real lambda);
};

#endif /* IANT_CONTROLLER_H_ */

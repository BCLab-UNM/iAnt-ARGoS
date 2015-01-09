#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

#include "iAnt_pheromone.h"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;
using namespace std;

class iAnt_controller : public CCI_Controller {

    public:

        iAnt_controller();
        ~iAnt_controller();

        bool IsInTheNest();   // true or false: Am I in the nest?
        bool IsFindingFood(); // true or false: Am I finding food (i.e. currently over a food item location)?
        bool IsHoldingFood(); // true or false: AM I holding a food item?
        void PickupFood();    // the robot picks up a food item
        void DropOffFood();   // the robot is drops off a food item

        void UpdateFoodList(vector<CVector2> newFoodPositions); // update food position list
        void UpdatePheromoneList(vector<CVector2> newPheromonePositions); // update pheromone positions
        void UpdateFidelityList(vector<CVector2> newFidelityPositions);   // update fidelity positions
        void UpdatePosition(CVector2 newPosition);	// update iAnt's position
        void UpdateTime(long int newTime);			// update simulation time
        CVector2 Position();						// return the robot's position
        CVector2 FidelityPosition();                // return the robot's fidelity position
        Real     PheromoneDecayRate();              // return pheromoneDecayRate

        void SetNestPosition(CVector2 np);          // set new nest position
        void SetNestRadiusSquared(Real r);          // set new nest radius
        void SetFoodRadiusSquared(Real rs);         // set squaqred radius of food items
        void SetTargetPheromone(iAnt_pheromone p);	// update target pheromone
        void SetForageRange(CRange<Real> X, CRange<Real> Y); // setup forage boundary

        CVector2         GetNestPosition();         // get nest position
        Real             GetNestRadius();           // get nest radius
        Real             GetFoodRadius();           // get food radius
        vector<CVector2> GetFoodPositions();        // get food position list
        vector<CVector2> GetPheromonePositions();   // get pheromone position list
        vector<CVector2> GetFidelityPositions();    // get fidelity position list
        iAnt_pheromone   GetTargetPheromone();		// get target pheromone

        /* CCI_Controller inherited functions */
        void Init(TConfigurationNode& node); // initialize variables based on XML file
        void ControlStep(); // main logic control for the robot
        void Reset();       // when Argos3 GUI is reset, reset variables
        void Destroy();     // after experiment, clean up memory etc.

    private:

        CCI_DifferentialSteeringActuator *steeringActuator; // controls the robot's motor speeds
        CCI_FootBotProximitySensor       *proximitySensor;  // detects nearby objects to prevent collision
        CCI_FootBotMotorGroundSensor     *groundSensor;     // detects food items & nest (color changes)
        CCI_PositioningSensor            *compassSensor;    // used for compass/heading

        CRandom::CRNG *RNG; // random number generator

        vector<CVector2> foodPositions;      // food item positions, updated by loop_functions
        vector<CVector2> pheromonePositions; // pheromones positions, updated by loop_functions
        vector<CVector2> fidelityPositions;  // fidelity positions, updated by loop_functions

        bool     holdingFood;      // Is the robot carrying food? yes = true, no = false
        bool     informed;         // is site fidelity or pheromones active
        long int collisionDelay;   // delay after collision detection
        int      resourceDensity;  // number of food items around the last found food position

        CVector2 position;         // robot's position on the arena
        CVector2 target;           // robot's current target or location to move to
        CVector2 fidelityPosition; // robot's remembered fidelity location
        CVector2 nestPosition;     // the position of the center of the nest
        CRange<Real> forageRangeX; // cartesian X forage boundary
        CRange<Real> forageRangeY; // cartesian Y forage boundary

        Real nestRadiusSquared; // radius of the nest
        Real foodRadiusSquared; // radius of food items, squared
        Real searchRadiusSquared; // radius of search for resourceDensity
        Real distanceTolerance; // distance to trigger collision detection
        Real travelGiveupProbability; // %-chance of traveling, from [0.0, 1.0]
        Real searchGiveupProbability; // %-chance of searching, from [0.0, 1.0]
        Real searchStepSize; // vector length for each search "step"
        Real maxSpeed; // maximum motor speed, configured in XML
        Real informedSearchDecay;
        Real siteFidelityRate; // % chance of using site fidelity
        Real pheromoneRate; // % chance of laying pheromones
        Real pheromoneDecayRate; // %-rate that pheromones decay [0.0, 10.0]

        long int simTime; // frame count for simulation
        long int searchTime; // time spent searching

        CRadians uninformedSearchCorrelation; // radian angle turned during searching [0.0, 4.0PI]

        CRange<CRadians> angleTolerance; // angle tolerance range to go straight

        iAnt_pheromone targetPheromone; // pheromone trail iAnt is following
        iAnt_pheromone sharedPheromone; // pheromone iAnt is sharing

        /* state machine parameters, NOTE: the algorithm used is NOT a true state machine model */

        enum CPFA {
            INACTIVE,
            DEPARTING,
            SEARCHING,
            RETURNING,
        } CPFA;

        /* CPFA implementation functions */

        void inactive();
        void departing();
        void searching();
        void returning();

        /* CPFA helper functions */

        void     senseLocalResourceDensity();
        CVector2 setPositionInBounds(CVector2 rawPosition);

        /* private helper functions for motion control and navigation */

        void		SetRandomSearchLocation();	// set target to a random location
        bool		CollisionDetection();		// detect collisions and turn appropriately
        CRadians	RobotHeading();				// get robot compass direction
        void		SetWheelSpeed();			// set wheel speeds based on heading and target position
};

#endif /* IANT_CONTROLLER_H_ */

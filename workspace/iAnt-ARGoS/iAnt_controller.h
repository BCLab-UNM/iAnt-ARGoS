#ifndef IANT_CONTROLLER_H_
#define IANT_CONTROLLER_H_

/* contains several structs which encapsulates data variables for the controller */
#include "iAnt_data_structures.h"
/* base class for controller objects */
#include <argos3/core/control_interface/ci_controller.h>
/* updates the motors' speed settings */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* collision detection sensors */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* detect color changes on the ground, used for food & nest detection */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* used in navigation to locate targets, light is fixed at the nest */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* class for random number generator objects */
#include <argos3/core/utility/math/rng.h>
/* provide LOG and LOGERR print out capability to the Argos GUI screen */
#include <argos3/core/utility/logging/argos_log.h>

/* access to Argos3 classes and objects */
using namespace argos;
/* access to std::endl and std::vector, etc. */
using namespace std;

/*******************************************************************************************
 * Argos iAnt Controller Class
 *******************************************************************************************
 * This class defines the basic parameters and behavior functions for an iAnt Robot.
 * Currently, simulated iAnt robots are based off of the foot-bot design that comes
 * standard with the Argos3 simulator. In the future, it would be beneficial to write
 * custom made plug-ins for the physical iAnt robots (TO-DO).
 *******************************************************************************************/

class iAnt_controller : public CCI_Controller {

	/* TODO
	 * get rid of the necessity to use the friend feature here... */
	/* the PreStep() function in iAnt_loop_functions needs deeper access to this class */
	friend class iAnt_loop_functions;

public:

	/* constructor and destructor functions */
	iAnt_controller();
	~iAnt_controller();

	/* helper functions for this class, iAnt_loop_functions, and iAnt_qt_user_functions */
    bool isInTheNest();   // true or false: Am I in the nest?
	bool hasFoundFood();  // true or false: Have I found food?
	bool isHoldingFood(); // true or false: AM I holding a food item?

	/* inherited functions from the CCI_Controller */
	void Init(TConfigurationNode& node); // initialize variables based on XML file
    void ControlStep();                  // main logic control for the robot
    void Reset();                        // when Argos3 GUI is reset, reset variables
    void Destroy();                      // after experiment, clean up memory etc.

private:

	/* robot actuator and sensor components */
	CCI_DifferentialSteeringActuator *steeringActuator; // controls the robot's motor speeds
	CCI_FootBotProximitySensor       *proximitySensor;  // detects nearby objects to prevent collision
	CCI_FootBotMotorGroundSensor     *groundSensor;     // detects food items & nest (color changes)
	CCI_FootBotLightSensor           *lightSensor;      // detects nest-light for navigation control

	/* random number generator used for random walking, etc. */
	CRandom::CRNG* RNG;

	/* data variables for controller: CPFA, navigation, food */
	iAnt_data_structures iAntData;
	vector<pheromoneWaypoint> pheromoneLocations; // pheromone list
	map<UInt8,pheromoneWaypoint> pendingWaypointList; // pending pheromone list

	/* CPFA state machine implementation functions */
	void setSearchLocation();
	void travelToSearchSite();
	void performInformedWalk();
	void performUninformedWalk();
	void senseLocalResourceDensity();
	void travelToNest();

	/* helper functions used for motion control and navigation */
	bool     collisionDetection();                                          // detect collisions and turn appropriately
	CRadians lawOfCosines(CVector2& A, CVector2& B, CVector2& C);           // helper for getVectorToPosition()
	Real     getSignOfRotationAngle(CVector2& A, CVector2& B, CVector2& C); // helper for lawOfCosines()
	CVector2 getVectorToLight();                                            // calculate heading towards the nest-light
	CVector2 getVectorToPosition(const CVector2& targetPosition);           // calculate heading towards robot target
	void     setWheelSpeed(const CVector2& heading);                        // set wheel speeds based on desired heading

};

#endif /* IANT_CONTROLLER_H_ */

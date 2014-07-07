#ifndef IANT_DATA_STRUCTURES_H_
#define IANT_DATA_STRUCTURES_H_

/* Access XML loading functionality. */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* ARGoS 2D vector class. */
#include <argos3/core/utility/math/vector2.h>
/* ARGOS angle definition class. */
#include <argos3/core/utility/math/angles.h>

/* access to Argos3 classes and objects */
using namespace argos;
/* access to std::endl and std::vector, etc. */
using namespace std;

class iAnt_data_structures {

	/* iAnt_controller and iAnt_loop_functions needs deeper access to this class */
	friend class iAnt_loop_functions;
	friend class iAnt_controller;

public:

	iAnt_data_structures() {}
	~iAnt_data_structures() {}

	// XML parsing function
    void Init(TConfigurationNode& node);

private:

	/* robot [C]entral [P]lace [F]oraging [A]lgorithm variables */
	struct CPFA {

		/* state machine parameters, NOTE: the algorithm used is NOT a true state machine model */
		enum state {
	         SET_SEARCH_LOCATION = 0,
	         TRAVEL_TO_SEARCH_SITE,
	         PERFORM_INFORMED_WALK,
	         PERFORM_UNINFORMED_WALK,
	         SENSE_LOCAL_RESOURCE_DENSITY,
	         TRAVEL_TO_NEST,
		} state;
                                              /* NOTE: until my understanding improves, these comments may not be accurate */
		Real     travelProbability;           // %-chance of traveling, from [0.0, 1.0]
		Real     searchProbability;           // %-chance of searching, from [0.0, 1.0]
		CRadians uninformedSearchCorrelation; // radian angle turned during searching [0.0, 4.0PI]
		Real     informedSearchDecay;         // %-rate that informed search decays [0.0, 5.0]
		Real     siteFidelityRate;            // %-chance that robot remembers a site [0.0, 20.0]
		Real     pheromoneRate;               // %-chance of laying a pheromone [0.0, 20.0]
		Real     pheromoneDecayRate;          // %-rate that pheromones decay [0.0, 10.0]

		CPFA();                              // constructor function
		void Init(TConfigurationNode& node); // XML configuration initialization

	} CPFA;


	/* robot navigation control variables */
	struct navigation {

	    Real             distanceTolerance;   // distance to trigger collision detection
	    CRange<CRadians> angleTolerance;      // angle tolerance range to go straight
	    CVector2         arenaSize;           // rectangular grid size "GetX()" by "GetY()"
		CRange<Real>     forageRangeX;        // Cartesian X domain of arena [-x, x]
		CRange<Real>     forageRangeY;        // Cartesian Y range of arena [-y, y]
	    CVector2         position;            // robot's current position in the arena
	    CVector2         target;              // robot's current target in the arena
	    CVector2         nestPosition;        // the position of the center of the nest
	    Real             nestRadiusSquared;   // the nest area radius
	    Real             searchStepSize;      // vector length for each search "step"
	    Real             searchRadiusSquared; // food density search radius around robot
		Real             maxSpeed;            // maximum motor speed, configured in XML

		navigation();                        // constructor function
		void Init(TConfigurationNode& node); // XML configuration initialization

	} navigation;

	/* robot food and foraging variables */
	struct food {

		bool             isHoldingFoodItem;  // true or false: Is the robot carrying a food item?
		bool             hasActivePheromone; // true or false: Is the robot aware of an active pheromone?
		CVector2         pheromonePosition;  // current position of any pheromone known by this robot
		size_t           resourceCount;      // the number of resources found around pheromonePosition
		vector<CVector2> foodPositions;      // list of all food positions within the arena
		Real             foodRadiusSquared;  // radius of food items on the arena floor

		food();                              // constructor function
		void Init(TConfigurationNode& node); // initialization function

	} food;

};

#endif /* IANT_DATA_STRUCTURES_H_ */

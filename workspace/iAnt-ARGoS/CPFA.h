#ifndef CPFA_H_
#define CPFA_H_

#include "PheromoneWaypoint.h"
#include "FoodData.h"
#include "NavigationData.h"
#include <argos3/core/utility/math/rng.h>

class CPFA {

private:

	/* CPFA variables,
     * NOTE: until my understanding improves, below comments may not be accurate. */
	Real           travelProbability;           // %-chance of traveling, from [0.0, 1.0]
	Real           searchProbability;           // %-chance of searching, from [0.0, 1.0]
	CRadians       uninformedSearchCorrelation; // radian angle turned during searching [0.0, 4.0PI]
	Real           informedSearchDecay;         // %-rate that informed search decays [0.0, 5.0]
	Real           siteFidelityRate;            // %-chance that robot remembers a site [0.0, 20.0]
	Real           pheromoneRate;               // %-chance of laying a pheromone [0.0, 20.0]
	Real           pheromoneDecayRate;          // %-rate that pheromones decay [0.0, 10.0]

	PheromoneWaypoint pheromone;
	FoodData food;
	NavigationData navigation;
	CRandom::CRNG *RNG;

	/* Primary state definitions for the CPFA. */
	enum state {
        SET_SEARCH_LOCATION = 0,
        TRAVEL_TO_SEARCH_SITE,
        PERFORM_INFORMED_WALK,
        PERFORM_UNINFORMED_WALK,
        SENSE_LOCAL_RESOURCE_DENSITY,
        TRAVEL_TO_NEST
	} state;

public:
	CPFA();
	~CPFA();

};

#endif /* CPFA_H_ */

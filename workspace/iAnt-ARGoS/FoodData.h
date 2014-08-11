#ifndef FOODDATA_H_
#define FOODDATA_H_

/* Implements food source location tracking. */
#include "PheromoneWaypoint.h"
/* Access XML loading functionality. */
#include <argos3/core/utility/configuration/argos_configuration.h>

using namespace argos;

class FoodData {

private:

	bool              hasFood;            // Currently holding food? yes = true
	size_t            totalFoodCollected; // Count of total food collected.
	PheromoneWaypoint pheromone;          // Food source location tracking.

public:

	FoodData();
	~FoodData();

	void Init(TConfigurationNode& node) {}
	void Reset() {}

	bool HasFood();
	bool HasPheromone();
	CVector2 GetPheromone();

};

#endif /* FOODDATA_H_ */

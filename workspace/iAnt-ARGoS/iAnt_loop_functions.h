#ifndef IANT_LOOP_FUNCTIONS_H_
#define IANT_LOOP_FUNCTIONS_H_

#include "iAnt_controller.h"
#include "iAnt_pheromone.h"
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;
using namespace std;

class iAnt_loop_functions : public CLoopFunctions {

//friend class iAnt_controller;

private:

	CFloorEntity *floorEntity; // object for the floor graphics

	CRandom::CRNG *RNG; // random number generator

	long int tick; // a counter for simulation frames

	size_t foodItemCount; // number of food items on the field

	vector<CVector2> foodPositions; // positions for all food items on the field
	vector<iAnt_pheromone> pheromoneList; // list of pheromones to share with iAnts

	iAnt_pheromone pendingPheromone;

	CVector2 nestPosition; // center of the circular nest

	Real foodRadiusSquared; // radius of circular food object squared
	Real nestRadiusSquared; // radius of circular nest squared

	CRange<Real> forageRangeX; // Cartesian X domain of arena [-x, x]
	CRange<Real> forageRangeY; // Cartesian Y range of arena [-y, y]

public:

	iAnt_loop_functions();
	~iAnt_loop_functions() {}

	// inherited functions from CLoopFunctions
	void Init(TConfigurationNode& node);
	CColor GetFloorColor(const CVector2& c_position_on_plane);
	void PreStep();
	void PostStep();
	void Reset();
};

#endif /* IANT_LOOP_FUNCTIONS_H_ */

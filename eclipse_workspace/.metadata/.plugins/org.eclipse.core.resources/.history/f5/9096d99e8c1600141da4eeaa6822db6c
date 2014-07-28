#ifndef IANT_LOOP_FUNCTIONS_H_
#define IANT_LOOP_FUNCTIONS_H_

#include "iAnt_controller.h"
#include "iAnt_data_structures.h"
#include "iAnt_genetic_algorithm.h"
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;
using namespace std;

class iAnt_loop_functions : public CLoopFunctions {

private:

    CFloorEntity          *floorEntity;      // object for the floor graphics
	CRandom::CRNG         *RNG;              // random number generator

	iAnt_data_structures   iAntData;         // data structure interface used with all controllers (robots)
	iAnt_genetic_algorithm iAntDataAnalysis; // analyzes controllers and updates CPFA

public:

	iAnt_loop_functions();
	~iAnt_loop_functions() {}

	// inherited functions from CLoopFunctions class
	void   Init(TConfigurationNode& node);
	CColor GetFloorColor(const CVector2& c_position_on_plane);
	void   PreStep();
	void   PostStep();
	void   Reset();

};

#endif /* IANT_LOOP_FUNCTIONS_H_ */

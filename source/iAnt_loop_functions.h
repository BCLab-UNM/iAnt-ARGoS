#ifndef IANT_LOOP_FUNCTIONS_H_
#define IANT_LOOP_FUNCTIONS_H_

#include <source/iAnt_controller.h>
#include <vector>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;
using namespace std;

/*****
 * The loop functions class provides "hooks" into the simulation right before
 * and right after each tick (or frame) of the simulation. The primary use of
 * this class will be to maintain global status variables.
 *****/
class iAnt_loop_functions : public CLoopFunctions {
	public:

		iAnt_loop_functions() {}
		~iAnt_loop_functions() {}

		void Init(TConfigurationNode& node);
		void PreStep();
		void PostStep();
        void PostExperiment();
		void Reset();

        bool IsExperimentFinished();

		CColor GetFloorColor(const CVector2& p) { return CColor::WHITE; }

    private:

        vector<iAnt_controller*>  iAnts;
        CRandom::CRNG            *RNG;
        iAnt_data                 data;
};

#endif /* IANT_LOOP_FUNCTIONS_H_ */

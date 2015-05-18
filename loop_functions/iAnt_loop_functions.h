#ifndef IANT_LOOP_FUNCTIONS_H_
#define IANT_LOOP_FUNCTIONS_H_

#include <controllers/iAnt_controller.h>
#include <vector>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;
using namespace std;

/*****
 * Simulation hook functions.
 *****/
class iAnt_loop_functions : public CLoopFunctions {
	public:

		iAnt_loop_functions()  {}
		~iAnt_loop_functions() {}

		void Init(TConfigurationNode& node);
		void PreStep();
		void PostStep();
        void PostExperiment();
		void Reset();

        bool IsExperimentFinished();

		CColor GetFloorColor(const CVector2& p) { return CColor::WHITE; }

    private:

        vector<iAnt_controller*> iAnts;
        CRandom::CRNG*           RNG;
        iAnt_data                data;
};

#endif /* IANT_LOOP_FUNCTIONS_H_ */

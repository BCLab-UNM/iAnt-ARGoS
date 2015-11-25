#ifndef DSA_LOOP_FUNCTIONS_H_
#define DSA_LOOP_FUNCTIONS_H_

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
class DSA_loop_functions : public CLoopFunctions {
	public:

		DSA_loop_functions() {}
		~DSA_loop_functions() {}

		size_t TicksPerSecond;

		void Init(TConfigurationNode& node);
		void PreStep();
		void PostStep();
        void PostExperiment();
		void Reset();
        bool IsExperimentFinished();


        void ReadFile();
        void CopyPatterntoTemp();
		void StoreStringPattern(string i_robotPath);
	
		CColor GetFloorColor(const CVector2& p) { return CColor::WHITE; }

	protected:

        /* iAnt simulation data */
        size_t SimTime;
        size_t MaxSimTime;
        size_t TicksPerSecond;
        size_t ResourceDensityDelay;
        size_t N_robots;

        size_t RandomSeed;
        size_t SimCounter;
        size_t MaxSimCounter;
        size_t VariableFoodPlacement;
        size_t OutputData;

        size_t DrawDensityRate;
        size_t DrawIDs;
        size_t DrawTrails;
        size_t DrawTargetRays;

        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;

        /* iAnt_controller data */
        Real         NestRadius;
        Real         NestRadiusSquared;
        Real         NestElevation;
        Real         SearchRadius;
        Real         FoodRadius;
        Real         FoodRadiusSquared;
        CRange<Real> ForageRangeX;
        CRange<Real> ForageRangeY;
        CVector2     NestPosition;

    private:

        vector<DSA_controller*>   DSAnts;
        CRandom::CRNG             *RNG;

        size_t					  N_robots;
        vector<string> 			  robotPattern;
};

#endif /* DSA_LOOP_FUNCTIONS_H_ */
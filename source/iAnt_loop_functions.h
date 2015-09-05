#ifndef IANT_LOOP_FUNCTIONS_H_
#define IANT_LOOP_FUNCTIONS_H_

#include <source/iAnt_controller.h>
#include <source/iAnt_data.h>
#include <vector>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

using namespace argos;
using namespace std;

class iAnt_controller;

/*****
 * The loop functions class provides "hooks" into the simulation right before and right after each tick (or frame) of
 * the simulation. The primary use of this class will be to maintain global status variables.
 *****/
class iAnt_loop_functions : public CLoopFunctions {

	friend class iAnt_controller;
	friend class iAnt_qt_user_functions;

	public:

		iAnt_loop_functions();
		~iAnt_loop_functions() {}

		void Init(TConfigurationNode& node);
		void PreStep();
		void PostStep();
        void PostExperiment();
		void Reset();
        bool IsExperimentFinished();
		CColor GetFloorColor(const CVector2& p) { return CColor::WHITE; }
		vector<CVector2>* GetFoodList() { return &FoodList; }

        /* public helper functions */
        void UpdatePheromoneList();
        void SetFoodDistribution();

	protected:

        /* iAnt simulation data */
        size_t SimTime;
        size_t MaxSimTime;
        size_t TicksPerSecond;
        size_t ResourceDensityDelay;

        size_t RandomSeed;
        size_t SimCounter;
        size_t MaxSimCounter;
        size_t VariableSeed;
        size_t OutputData;

        size_t TrailDensityRate;
        size_t DrawTrails;
        size_t DrawTargetRays;

        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;

        /* CPFA variables */
        Real     ProbabilityOfSwitchingToSearching;
        Real     ProbabilityOfReturningToNest;
        CRadians UninformedSearchVariation;
        Real     RateOfInformedSearchDecay;
        Real     RateOfSiteFidelity;
        Real     RateOfLayingPheromone;
        Real     RateOfPheromoneDecay;

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

        /* position vectors */
        vector<CVector2>       FoodList;
        vector<CColor>         FoodColoringList;
        vector<CVector2>       FidelityList;
        vector<iAnt_pheromone> PheromoneList;
        vector<CRay3>          TargetRayList;

    private:

        CRandom::CRNG* RNG;
        iAnt_data      data;

        /* private helper functions */
        void RandomFoodDistribution();
        void ClusterFoodDistribution();
        void PowerLawFoodDistribution();
        bool IsOutOfBounds(CVector2 p, size_t length, size_t width);
        bool IsCollidingWithNest(CVector2 p);
        bool IsCollidingWithFood(CVector2 p);
};

#endif /* IANT_LOOP_FUNCTIONS_H_ */

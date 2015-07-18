#ifndef IANT_LOOP_FUNCTIONS_H_
#define IANT_LOOP_FUNCTIONS_H_

#include <string>
#include <fstream>
#include <controllers/iAnt_controller.h>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;
using namespace std;

class iAnt_controller;

class iAnt_loop_functions : public CLoopFunctions {

	public:

		iAnt_loop_functions();
		~iAnt_loop_functions();

		/* inherited from CLoopFunctions */
		void   Init(TConfigurationNode& node);
		CColor GetFloorColor(const CVector2& c_position_on_plane);
		void   PreStep();
		void   PostStep();
        bool   IsExperimentFinished();
        void   PostExperiment();
		void   Reset();

        /* setter/getter functions */
        vector<CVector2> GetFidelityPositions();
        vector<CVector2> GetFoodPositions();
        vector<CVector2> GetPheromonePositions();

	private:

        void SetFoodDistribution();
        void RandomFoodDistribution();
        void RandomFoodDistribution(size_t f);
        void ClusterFoodDistribution();
        void PowerLawFoodDistribution();

        bool IsCollidingWithNest(CVector2 p);
        bool IsCollidingWithFood(CVector2 p);
        bool IsOutOfBounds(CVector2 p, size_t length, size_t width);

        Real GetNestRadius();
        Real GetFoodRadius();

        vector<iAnt_pheromone> UpdatePheromoneList(iAnt_controller *c);
        vector<CVector2>       UpdatePheromonePositions(iAnt_controller *c);
        vector<CVector2>       UpdateFoodPositions(iAnt_controller *c);
        vector<CVector2>       UpdateFidelityPositions();

        TConfigurationNode     *initNode;
		CFloorEntity           *floorEntity;        // object for the floor graphics
		CRandom::CRNG          *RNG;                // random number generator
		size_t                  simTime;            // a counter for simulation frames
		size_t                  simCounter;         // a counter for simulation runs
		size_t                  foodItemCount;      // number of food items on the field
        size_t                  ticks_per_second;
        size_t                  ticks_per_simulation;
        size_t                  random_seed;

        vector<iAnt_controller*> iAnts;
		vector<CVector2>        foodPositions;      // food item positions on the field
        vector<CVector2>        pheromonePositions; // pheromone positions on the field
        vector<CVector2>        fidelityPositions;  // fidelity positions on the field
		vector<iAnt_pheromone>  pheromoneList;      // list of pheromones to share with iAnts
		CVector2                nestPosition;       // center of the circular nest

		Real                    foodRadiusSquared;  // radius of circular food object squared
		Real                    nestRadiusSquared;  // radius of circular nest squared

        CVector3                arenaSize;
		CRange<Real>            forageRangeX;       // Cartesian X domain of arena [-x, x]
		CRange<Real>            forageRangeY;       // Cartesian Y range of arena [-y, y]

        bool                    variableSeed;       // random seed changes after reset
        bool                    outputData;
		size_t                  foodDistribution;   // 0="random", 1="cluster", 2="power law"
        size_t                  numberOfClusters;
        size_t                  clusterWidthX;
        size_t                  clusterLengthY;
        size_t                  powerRank;
};

#endif /* IANT_LOOP_FUNCTIONS_H_ */

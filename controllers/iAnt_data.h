#ifndef IANT_DATA_H_
#define IANT_DATA_H_

#include "iAnt_pheromone.h"
#include "iAnt_food_type.h"
#include <vector>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

using namespace std;
using namespace argos;

class iAnt_data {

    public:

        iAnt_data();

        /* public helper functions */
        void UpdatePheromoneList();
        void SetFoodDistribution();

        /* iAnt simulation data */
        size_t SimTime;
        size_t TicksPerSecond;
        size_t RandomSeed;
        size_t VariableSeed;
        size_t SimCounter;
        size_t OutputData;
        size_t TrailDensityRate;
        size_t DrawTrails;
        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;
        vector<iAnt_food_type>        food_details;      // food item positions on the field
        CVector2 FoodItemInHold;
		CRandom::CRNG *RNG;

        /* CPFA variables and settings */
        Real     ProbabilityOfSwitchingToSearching;
        Real     ProbabilityOfReturningToNest;
        CRadians UninformedSearchVariation;
        Real     RateOfInformedSearchDecay;
        Real     RateOfSiteFidelity;
        Real     RateOfLayingPheromone;
        Real     RateOfPheromoneDecay;

        /* iAnt_controller data */
        Real                   TurnProbability;
        Real                   PushProbability;
        Real                   PullProbability;
        Real                   WaitProbability;
        Real                   NestRadius;
        Real                   NestRadiusSquared;
        Real                   NestElevation;
        Real                   FoodRadius;
        Real                   FoodRadiusSquared;
        CRange<Real>           ForageRangeX;
        CRange<Real>           ForageRangeY;
        CVector2               NestPosition;
        vector<CVector2>       FoodList;
        vector<CVector2>       FidelityList;
        vector<iAnt_pheromone> PheromoneList;

    private:

        /* private helper functions */
        void RandomFoodDistribution();
        void ClusterFoodDistribution();
        void PowerLawFoodDistribution();
        bool IsOutOfBounds(CVector2 p, size_t length, size_t width);
        bool IsCollidingWithNest(CVector2 p);
        bool IsCollidingWithFood(CVector2 p);
};

#endif /* IANT_DATA_H_ */

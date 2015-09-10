#ifndef IANT_DATA_H_
#define IANT_DATA_H_

//#include "iAnt_pheromone.h"
#include "iAnt_nest.h"

#include <vector>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
//#include <argos3/core/utility/math/ray3.h>

using namespace argos;
using namespace std;

class iAnt_data {

    public:

        iAnt_data();

        /* public helper functions */
        void UpdatePheromoneList(); //qilu 07/05/2015
        void SetFoodDistribution();
        
        /* iAnt simulation data */
		size_t ArenaX; //qilu 06/07
        size_t ArenaY;
        size_t SimTime;
        size_t MaxSimTime; //qilu 07/19
		size_t currNumCollision; //qilu 08/19
        size_t lastNumCollision; //qilu 08/19
        size_t lastNumCollectedFood; //qilu 08/19
        size_t currNumCollectedFood; //qilu 08/19
        size_t curr_time_in_minutes; //qilu 08/19
        size_t last_time_in_minutes;//qilu 08/19
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
        size_t PowerLawCopies; //qilu 08/14/2015

		CRandom::CRNG *RNG;

        /* MPFA variables and settings */
        Real     ProbabilityOfSwitchingToSearching;
        Real     ProbabilityOfReturningToNest;
        CRadians UninformedSearchVariation;
        Real     RateOfInformedSearchDecay;
        Real     RateOfSiteFidelity;
        Real     RateOfLayingPheromone;
        Real     RateOfPheromoneDecay;

        /* iAnt_controller data */
/*
        Real                   TurnProbability;
        Real                   PushProbability;
        Real                   PullProbability;
        Real                   WaitProbability;
*/
        Real                   NestRadius;
        Real                   NestRadiusSquared;
        Real                   NestElevation;
        Real                   SearchRadius;
        Real                   FoodRadius;
        Real                   FoodRadiusSquared;
        CRange<Real>           ForageRangeX;
        CRange<Real>           ForageRangeY;
        CVector2         NestPosition_0;//qilu 06/04
        CVector2         NestPosition_1;//qilu
        CVector2         NestPosition_2;//qilu
        CVector2         NestPosition_3;//qilu
        
        //CVector2         NestPosition_4;//qilu 06/07
        //CVector2         NestPosition_5;//qilu
        //CVector2         NestPosition_6;//qilu
        //CVector2         NestPosition_7;//qilu
    
        //CVector2         NestPosition_8;//qilu 06/04
        //CVector2         NestPosition_9;//qilu
        //CVector2         NestPosition_10;//qilu
        //CVector2         NestPosition_11;//qilu
        //CVector2         NestPosition_12;//qilu 06/07
        //CVector2         NestPosition_13;//qilu
        //CVector2         NestPosition_14;//qilu
        //CVector2         NestPosition_15;//qilu
    
        vector<iAnt_nest> nests; //qilu 07/05 
        CVector2         targetPosition; //qilu 06/04 needed?
        /* position vectors */
        vector<CVector2>       FoodList;
        vector<CColor>         FoodColoringList;
        //vector<CVector2>       FidelityList;
		map<string, CVector2>  FidelityList; //qilu 07/17
        vector<iAnt_pheromone> PheromoneList;
        vector<CRay3>          TargetRayList;
		vector<size_t>			CollisionList;//qilu 08/19
        vector<size_t>			ForageList; //qilu 08/19

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

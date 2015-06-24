#ifndef IANT_DATA_H_
#define IANT_DATA_H_

#include <vector>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray3.h>

using namespace argos;
using namespace std;

class iAnt_data {

    public:

        iAnt_data();

        /* public helper functions */
        void SetFoodDistribution();

        /* iAnt simulation data */
    
        ////////ADDED//////////
        size_t DrawTargetRays;
    
        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;

        /* iAnt_controller data */
        Real                   NestRadius;
        Real                   NestRadiusSquared;
        Real                   NestElevation;
        Real                   FoodRadius;
        Real                   FoodRadiusSquared;
        CRange<Real>           ForageRangeX;
        CRange<Real>           ForageRangeY;
        CVector2               NestPosition;

        /* position vectors */
        vector<CVector2>       FoodList;
    
        ///////ADDED////////////////
        vector<CRay3>          TargetRayList;

        /* random number generator */
        CRandom::CRNG* RNG;

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
